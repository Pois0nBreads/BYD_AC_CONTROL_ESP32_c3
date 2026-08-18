/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/twai.h"

#include "ble_multi_conn.h"
#include "byd_ac_can_handle.h"

#define BLE_AC_CLOSE                    0x00 //关闭空调命令
#define BLE_AC_AUTO                     0x01 //自动空调命令
#define BLE_AC_FVENTILATE               0x02 //通风空调命令
#define BLE_AC_FRONT_DEFROSTER          0x03 //除雾空调命令

#define BLE_AC_L_TEMP_UP                0x11 //左区温度加命令
#define BLE_AC_L_TEMP_DOWN              0x12 //左区温度减命令
#define BLE_AC_R_TEMP_UP                0x13 //右区温度加命令
#define BLE_AC_R_TEMP_DOWN              0x14 //右区温度减命令
#define BLE_AC_AREA_OPEN                0x15 //分控打开
#define BLE_AC_AREA_CLOSE               0x16 //分控关闭

#define BLE_AC_FAN_SPEED_UP             0x21 //风速加命令
#define BLE_AC_FAN_SPEED_DOWN           0x22 //风速减命令
#define BLE_AC_INTERNAL_CIRCULATION     0x23 //内循环命令
#define BLE_AC_EXTERNAL_CIRCULATION     0x24 //外循环命令

#define BLE_AC_FAN_FORWARD              0x31 //风向  前吹标记命令
#define BLE_AC_FAN_FORWARD_DOWN         0x32 //风向前下吹标记命令
#define BLE_AC_FAN_DOWN                 0x33 //风向  下吹标记命令
#define BLE_AC_FAN_DOWN_UP              0x34 //风向上下吹标记命令
#define BLE_AC_FAN_UP                   0x35 //风向上  吹标记命令
#define BLE_AC_FAN_FORWARD_UP           0x37 //风向前上吹标记命令

#define BLE_AC_RAER_DEFROSTER_OPEN      0x41 //后除雾打开命令
#define BLE_AC_RAER_DEFROSTER_CLOSE     0x42 //后除雾关闭命令

#define BLE_AC_RAER_COMPRESSOR_OPEN     0x51 //压缩机打开命令
#define BLE_AC_RAER_COMPRESSOR_CLOSE    0x52 //压缩机关闭命令

#define BLINK_GPIO GPIO_NUM_8
#define CAN_RX GPIO_NUM_2
#define CAN_TX GPIO_NUM_3
#define TAG "MAIN"

void twai_receive_task(void *arg);
void sendHeartPacket(void *arg);
esp_err_t initChip();
esp_err_t initTwai();

// 数据接收回调（收到手机控制指令时触发）
static void on_ble_receive(uint16_t conn_id, const uint8_t *data, uint16_t len) {
    ESP_LOG_BUFFER_HEX("MAIN: Received: ", data, len);
    
    // 示例：回复确认消息
    // uint8_t ping[] = {0xFF, 0xFF, 0xFF, 0xFF};
    // ble_multi_conn_send_notify(conn_id, ping, sizeof(ping));
    if (len != 2)
        return;
    if (data[0] != 0x55)
        return;
    switch (data[1]) {
        case BLE_AC_CLOSE:                    //关闭空调命令
            byd_can_ac_control_mode(BYD_AC_CONTROL_CLOSE);
            break;
        case BLE_AC_AUTO  :                   //自动空调命令
            byd_can_ac_control_mode(BYD_AC_CONTROL_AUTO);
            break;
        case BLE_AC_FVENTILATE:               //通风空调命令
            byd_can_ac_control_mode(BYD_AC_CONTROL_VENTILATE);
            break;
        case BLE_AC_FRONT_DEFROSTER:           //除雾空调命令
            byd_can_ac_control_mode(BYD_AC_CONTROL_FRONT_DEFROSTER);
            break;
        case BLE_AC_L_TEMP_UP:                 //左区温度加命令
            byd_can_ac_left_temp_up();
            break;
        case BLE_AC_L_TEMP_DOWN:               //左区温度减命令
            byd_can_ac_left_temp_down();
            break;
        case BLE_AC_R_TEMP_UP:                 //右区温度加命令
            byd_can_ac_right_temp_up();
            break;
        case BLE_AC_R_TEMP_DOWN:               //右区温度减命令
            byd_can_ac_right_temp_down();
            break;
        case BLE_AC_FAN_SPEED_UP:              //风速加命令
            byd_can_ac_fan_speed_up();
            break;
        case BLE_AC_FAN_SPEED_DOWN:            //风速减命令
            byd_can_ac_fan_speed_down();
            break;
        case BLE_AC_AREA_OPEN:                 //分控打开命令
            byd_can_ac_area_open();
            break;
        case BLE_AC_AREA_CLOSE:                //分控关闭命令
            byd_can_ac_area_close();
            break;
        case BLE_AC_INTERNAL_CIRCULATION:      //内循环命令
            byd_can_ac_internal_cycle_mode();
            break;
        case BLE_AC_EXTERNAL_CIRCULATION:      //外循环命令
            byd_can_ac_external_cycle_mode();
            break;
        case BLE_AC_FAN_FORWARD:               //风向  前吹标记命令
            byd_can_ac_fan_mode(BYD_AC_FAN_FORWARD);
            break;
        case BLE_AC_FAN_FORWARD_DOWN:          //风向前下吹标记命令
            byd_can_ac_fan_mode(BYD_AC_FAN_FORWARD_DOWN);
            break;
        case BLE_AC_FAN_DOWN:                  //风向  下吹标记命令
            byd_can_ac_fan_mode(BYD_AC_FAN_DOWN);
            break;
        case BLE_AC_FAN_DOWN_UP:               //风向上下吹标记命令
            byd_can_ac_fan_mode(BYD_AC_FAN_DOWN_UP);
            break;
        case BLE_AC_FAN_UP:                    //风向上  吹标记命令
            byd_can_ac_fan_mode(BYD_AC_FAN_UP);
            break;
        case BLE_AC_FAN_FORWARD_UP:            //风向前上吹标记命令
            byd_can_ac_fan_mode(BYD_AC_FAN_FORWARD_UP);
            break;
        case BLE_AC_RAER_DEFROSTER_OPEN:       //后除雾打开命令
            byd_can_ac_defroster_open();
            break;
        case BLE_AC_RAER_DEFROSTER_CLOSE:      //后除雾关闭命令
            byd_can_ac_defroster_close();
            break;
        case BLE_AC_RAER_COMPRESSOR_OPEN:      //压缩机打开命令
            byd_can_ac_compressor_open();
            break;
        case BLE_AC_RAER_COMPRESSOR_CLOSE:     //压缩机关闭命令
            byd_can_ac_compressor_close();
            break;
        default:
            break;
    }
    
    // TODO: 解析控制指令，更新设备状态，然后调用 ble_multi_conn_send_indicate 上报新状态
    // 例如：
    // uint8_t new_state = parse_command(data, len);
    // uint8_t status_report[] = {0x01, new_state};
    // ble_multi_conn_send_indicate(conn_id, status_report, sizeof(status_report));
}

void app_main(void) {
    ESP_LOGI(TAG, "Hello world!\n");
    if (initChip() != ESP_OK
        || initTwai() != ESP_OK
        || ble_multi_conn_init(4) != ESP_OK ) {
        ESP_LOGE(TAG, "Init failed, Restart ESP32");
        esp_restart();
        return;
    }

    // 设置接收回调
    ble_multi_conn_set_receive_callback(on_ble_receive);
    // 主循环：打印连接数量，并定期发送心跳（Notify）
    xTaskCreatePinnedToCore(sendHeartPacket, "BLE_HeTx", 4096, NULL, 8, NULL, tskNO_AFFINITY); 

    gpio_reset_pin(BLINK_GPIO);
    // 设置该引脚为输出模式
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    bool blink_int = false;
    while (1) {
        gpio_set_level(BLINK_GPIO, blink_int = !blink_int);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void sendHeartPacket(void *arg)  {
    uint8_t ping[] = {0xFF, 0xFF, 0xFF, 0xFF};
    while (1) {
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        // 向所有已连接设备发送心跳indicate
        for (int i = 0; i < 4; i++) {
            uint16_t cid = ble_multi_conn_get_conn_id_by_index(i);
            if (cid != 0xFFFF) {
                ble_multi_conn_send_indicate(cid, ping, sizeof(ping));
            }
        }
    }
}

void twai_receive_task(void *arg) {
    byd_ac_state_t ac_state;
    twai_message_t msg;
    uint8_t click[] = {0x55, 0xAA, 0x55, 0xAA};
    while (1) {
        ESP_ERROR_CHECK(twai_receive(&msg, portMAX_DELAY));
        if (msg.rtr != 0)
            continue;

        // int j;
        // printf("ReceiveID: 0x%lX Data: ", msg.identifier);
        // for (j = 0; j < msg.data_length_code; j++)
        //     printf("%02X ", msg.data[j]);
        // printf("\n");
        
        //方控按钮报文
        //如果是方控模式短按报文，解析后上报给蓝牙
        if (msg.identifier == 0x4A8 
            && msg.data_length_code == 8 
            && msg.data[0] == 0x01 
            && msg.data[1] == 0x10) {
            for (int i = 0; i < 4; i++) {
                uint16_t cid = ble_multi_conn_get_conn_id_by_index(i);
                if (cid != 0xFFFF) {
                    ble_multi_conn_send_indicate(cid, click, sizeof(click));
                }
            }
            return;
        }

        byd_ac_can_handle(&msg, &ac_state);
        if (!ac_state.isAcFrame)
            continue;
        uint8_t data[] = {
            0xAA,
            ac_state.ig2 ? 0x01 : 0x00,
            ac_state.left_temp,
            ac_state.right_temp,
            ac_state.fan_speed,
            ac_state.fan_mode,
            ac_state.ac_mode,
            ac_state.cycle_mode,
            ac_state.defroster_open ? 0x01 : 0x00,
            ac_state.compressor_open ? 0x01 : 0x00,
            ac_state.area_open ? 0x01 : 0x00,
        };
        for (int i = 0; i < 4; i++) {
            uint16_t cid = ble_multi_conn_get_conn_id_by_index(i);
            if (cid != 0xFFFF) {
                ble_multi_conn_send_indicate(cid, data, sizeof(data));
            }
        }
    }
    vTaskDelete(NULL);
}

esp_err_t initTwai() {
    //CAN接口基本配置
    ESP_LOGI(TAG, "Init TWAI Driver");
    twai_general_config_t g_config = {
        .mode = TWAI_MODE_NORMAL , //TWAI_MODE_NORMAL / TWAI_MODE_NO_ACK / TWAI_MODE_LISTEN_ONLY
        .tx_io = CAN_TX, //IO号
        .rx_io = CAN_RX, //IO号
        .clkout_io = TWAI_IO_UNUSED, //io号，不用为-1
        .bus_off_io = TWAI_IO_UNUSED,//io号，不用为-1
        .tx_queue_len = 5, //发送队列长度，0-禁用发送队列
        .rx_queue_len = 5,//接收队列长度
        .alerts_enabled = TWAI_ALERT_NONE,  //警告标志 TWAI_ALERT_ALL 可开启所有警告
        .clkout_divider = 0,//1 to 14 , 0-不用
        .intr_flags = ESP_INTR_FLAG_LEVEL1//中断优先级
    };
    //过滤器配置
    twai_filter_config_t f_config = {
        .acceptance_code = 0, //验证代码
        .acceptance_mask = 0xFFFFFFFF, //验证掩码 0xFFFFFFFF表示全部接收
        .single_filter = true//true：单过滤器模式 false：双过滤器模式
    };
    //CAN接口时序配置官方提供了1K to 1Mbps的常用配置
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS(); //TWAI_TIMING_CONFIG_500KBITS()

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "TWAI Driver installed");
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI Driver started\n");

    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, 8, NULL, tskNO_AFFINITY); 
    return ESP_OK;
}


esp_err_t initChip() {
    /* Print chip information */
    ESP_LOGI(TAG, "Init Chip Info");
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");
    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG, "silicon revision v%d.%d, ", major_rev, minor_rev);
    uint32_t flash_size;
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGE(TAG, "Get flash size failed");
        return ESP_ERR_FLASH_BASE;
    }
    ESP_LOGI(TAG, "%" PRIu32 "MB %s flash", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    ESP_LOGI(TAG, "Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
    return ESP_OK;
}