#include "byd_ac_can_handle.h"
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_log.h"
#include <stdio.h>

#define TAG "BYD_AC_CAN_Handle"


#define BYD_STATUS_CAN_ID        0x12D //车身状态
#define BYD_AC_STATUS_CAN_ID     0x13B //空调状态
#define BYD_AC_STATUS_CAN_ID2    0x2DB //空调状态2
#define BYD_AC_CONTROL_1_CAN_ID  0x4ED //空调控制 {内外循环, 空调模式, 温度加减}
#define BYD_AC_CONTROL_2_CAN_ID  0x1DE //空调控制 {压缩机控制, 风向控制, 风速控制}

static bool ig2 = false;        //车辆是否启动
static uint8_t fan_speed = 0;   //当前风速
static uint8_t hex2 = 0x55;     //0xAA
static uint8_t ac_state_2db = 0x59;    //空调2DB报文，显示空调状态
static uint8_t cycle_mode = BYD_AC_INTERNAL_CIRCULATION;    //内循环状态
static bool defroster_open = false;    //后除雾状态

static void changeHex2(void) {
    if (hex2 == 0x55)
        hex2 = 0xAA;
    else
        hex2 = 0x55;
}

/**
 * 内部函数
 */
//处理中控锁0x12D报文
static esp_err_t byd_status_frame_handle(twai_message_t *msg) {
    if (msg->data_length_code != 8)
        return ESP_OK;
        
    if ((msg->data[4] & 0x0C) == 0x0C) {
        ig2 = true;
    } else {
        ig2 = false;
    }
    return ESP_OK;
}

//处理空调0x13B报文
static esp_err_t byd_ac_frame_handle(twai_message_t *msg, byd_ac_state_t *info) {
    if (msg->data_length_code != 8)
        return ESP_OK;
    info->left_temp = msg->data[4];
    info->right_temp = msg->data[5];
    fan_speed = (msg->data[2] >> 4) & 0x0F;
    info->fan_speed = (msg->data[2] >> 4) & 0x0F;
    info->fan_mode = msg->data[2] & 0x0F;
    if (fan_speed == 0) {               //关闭
        info->ac_mode = BYD_AC_CLOSE_FLAG;           
    } else if (ac_state_2db == 0x65 ) {  //通风
        info->ac_mode = BYD_AC_VENTILATE_FLAG;
    } else if (ac_state_2db == 0xA9 ) { //除雾
        info->ac_mode = BYD_AC_FRONT_DEFROSTER_FLAG; 
    } else {
        if ((msg->data[1] >> 3) & 1) { //自动
            info->ac_mode = BYD_AC_AUTO_FLAG;       
        } else {                       //手动
            info->ac_mode = BYD_AC_MANUEL_FLAG;          
        }
    }
    cycle_mode = (msg->data[1] >> 4) & 0x03;
    info->cycle_mode = (msg->data[1] >> 4) & 0x03;
    defroster_open = msg->data[0] & 1;
    info->defroster_open = msg->data[0] & 1;
    info->compressor_open = (msg->data[0] >> 1) & 1;
    return ESP_OK;
}

//处理空调0x2DB报文
static esp_err_t byd_ac_frame_handle_2db(twai_message_t *msg, byd_ac_state_t *info) {
    if (msg->data_length_code != 8)
        return ESP_OK;
    ac_state_2db = msg->data[0];
    return ESP_OK;
}

//接收0x4ED hex2 状态
static esp_err_t byd_ac_contorl_1_frame_handle(twai_message_t *msg, byd_ac_state_t *info) {
    if (msg->data_length_code != 8)
        return ESP_OK;
    hex2 = msg->data[2];
    return ESP_OK;
}

//发送空调控制报文
static esp_err_t byd_ac_frame_send(uint32_t id, uint8_t *data) {
    if (!ig2)
        return ESP_OK;

    twai_message_t message = {
        .identifier = id,            // 消息ID, 本例中使用标准帧ID
        .extd = 0,                      // 0: 标准帧 (11位ID), 1: 扩展帧 (29位ID)
        .rtr = 0,                       // 0: 数据帧, 1: 远程帧
        .data_length_code = 8,          // 数据长度，单位: 字节
        // .ss = 0,                      // 默认0，自动重试
    };
    memcpy(message.data, data, 8);
    return twai_transmit(&message, pdMS_TO_TICKS(100));
}

/////////////////////////////////////////////////
/**
 * 公共函数
 */
//报文筛选入口
esp_err_t byd_ac_can_handle(twai_message_t *msg, byd_ac_state_t *info) {
    esp_err_t result = ESP_OK;
    switch (msg->identifier) {
        case BYD_STATUS_CAN_ID:
            info->isAcFrame = false;
            result = byd_status_frame_handle(msg);
            break;
        case BYD_AC_STATUS_CAN_ID:
            info->isAcFrame = true;
            info->ig2 = ig2;
            result = byd_ac_frame_handle(msg, info);
            break;
        case BYD_AC_STATUS_CAN_ID2:
            info->isAcFrame = false;
            result = byd_ac_frame_handle_2db(msg, info);
            break;
        case BYD_AC_CONTROL_1_CAN_ID:
            info->isAcFrame = false;
            result = byd_ac_contorl_1_frame_handle(msg, info);
            break;
        default:
            break;
    }
    return result;
}

//0x4ED BYD_AC_CONTROL_1_CAN_ID
//左区温度加 需要交替Hex2 BYD_AC_CONTROL_1_CAN_ID
void byd_can_ac_left_temp_up(void) {
    changeHex2();
    uint8_t data[8] = {
        0xFF, 0x09, hex2, 0xFF, 
        0xFF, 0xFF, 0xFF, 0x00,
    };
    byd_ac_frame_send(BYD_AC_CONTROL_1_CAN_ID, data);
}

//左区温度减 需要交替Hex2 BYD_AC_CONTROL_1_CAN_ID
void byd_can_ac_left_temp_down(void) {
    changeHex2();
    uint8_t data[8] = {
        0xFF, 0x0A, hex2, 0xFF,
        0xFF, 0xFF, 0xFF, 0x00,
    };
    byd_ac_frame_send(BYD_AC_CONTROL_1_CAN_ID, data);
}

//右区温度加 需要交替Hex2 BYD_AC_CONTROL_1_CAN_ID
void byd_can_ac_right_temp_up(void) {
    changeHex2();
    uint8_t data[8] = {
        0xFF, 0x09, hex2, 0xFF, 
        0xFF, 0xFF, 0xFF, 0x00,
    };
    byd_ac_frame_send(BYD_AC_CONTROL_1_CAN_ID, data);
}

//右区温度减 需要交替Hex2 BYD_AC_CONTROL_1_CAN_ID
void byd_can_ac_right_temp_down(void) {
    changeHex2();
    uint8_t data[8] = {
        0xFF, 0x0A, hex2, 0xFF, 
        0xFF, 0xFF, 0xFF, 0x00,
    };
    byd_ac_frame_send(BYD_AC_CONTROL_1_CAN_ID, data);
}

//打开后除雾 BYD_AC_CONTROL_1_CAN_ID
void byd_can_ac_defroster_open(void) {
    if (defroster_open == true)
        return;
    changeHex2();
    uint8_t data[8] = {
        0xFF, 0x04, hex2, 0xFF, 
        0xFF, 0xFF, 0xFF, 0x00, 
    };
    byd_ac_frame_send(BYD_AC_CONTROL_1_CAN_ID, data);
}

//关闭后除雾 BYD_AC_CONTROL_1_CAN_ID
void byd_can_ac_defroster_close(void) {
    if (defroster_open == false)
        return;
    changeHex2();
    uint8_t data[8] = {
        0xFF, 0x04, hex2, 0xFF, 
        0xFF, 0xFF, 0xFF, 0x00, 
    };
    byd_ac_frame_send(BYD_AC_CONTROL_1_CAN_ID, data);
}

//设置内循环 BYD_AC_CONTROL_1_CAN_ID
void byd_can_ac_internal_cycle_mode(void) {
    if (cycle_mode == BYD_AC_INTERNAL_CIRCULATION)
        return;
    changeHex2();
    uint8_t data[8] = {
        0xFF, 0x23, hex2, 0xFF, 
        0xFF, 0xFF, 0xFF, 0x00, 
    };
    byd_ac_frame_send(BYD_AC_CONTROL_1_CAN_ID, data);
}

//设置外循环 BYD_AC_CONTROL_1_CAN_ID
void byd_can_ac_external_cycle_mode(void) {
    if (cycle_mode == BYD_AC_EXTERNAL_CIRCULATION)
        return;
    changeHex2();
    uint8_t data[8] = {
        0xFF, 0x23, hex2, 0xFF, 
        0xFF, 0xFF, 0xFF, 0x00, 
    };
    byd_ac_frame_send(BYD_AC_CONTROL_1_CAN_ID, data);
}


//0x1DE BYD_AC_CONTROL_2_CAN_ID
//鼓风机风速加 BYD_AC_CONTROL_2_CAN_ID
void byd_can_ac_fan_speed_up() {
    if (fan_speed == 0)
        return;
    uint8_t speed = fan_speed + 1;
    if (speed > 7)
        speed = 7;
    uint8_t data[8] = {
        (speed << 4) & 0xF0, 
        0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 
    };
    byd_ac_frame_send(BYD_AC_CONTROL_2_CAN_ID, data);
}

//鼓风机风速减 BYD_AC_CONTROL_2_CAN_ID
void byd_can_ac_fan_speed_down() {
    if (fan_speed == 0)
        return;
    uint8_t speed = fan_speed - 1;
    if (speed < 1)
        speed = 1;
    uint8_t data[8] = {
        (speed << 4) & 0xF0, 
        0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 
    };
    byd_ac_frame_send(BYD_AC_CONTROL_2_CAN_ID, data);
}

//设置风向 BYD_AC_CONTROL_2_CAN_ID
void byd_can_ac_fan_mode(uint8_t mode) {
    uint8_t data[8] = {
        mode & 0x0F, 
        0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 
    };
    byd_ac_frame_send(BYD_AC_CONTROL_2_CAN_ID, data);
}

//打开压缩机 BYD_AC_CONTROL_2_CAN_ID
void byd_can_ac_compressor_open(void) {
    uint8_t data[8] = {
        0x00, 0x02, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 
    };
    byd_ac_frame_send(BYD_AC_CONTROL_2_CAN_ID, data);
}

//关闭压缩机 BYD_AC_CONTROL_2_CAN_ID
void byd_can_ac_compressor_close(void) {
    uint8_t data[8] = {
        0x00, 0x01, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 
    };
    byd_ac_frame_send(BYD_AC_CONTROL_2_CAN_ID, data);
}



//设置自动空调模式 需要交替Hex2 BYD_AC_CONTROL_1_CAN_ID & BYD_AC_CONTROL_2_CAN_ID
void byd_can_ac_control_mode(uint8_t mode) {
    uint8_t data[8] = {
        0xFF, 0x00, 0x00, 0xFF, 
        0xFF, 0xFF, 0xFF, 0x00, 
    };
    switch (mode) {
        case BYD_AC_CONTROL_CLOSE:
            data[1] = 0X01;
            break;
        case BYD_AC_CONTROL_AUTO:
            data[1] = 0X02;
            break;
        case BYD_AC_CONTROL_FRONT_DEFROSTER:
            data[1] = 0X03;
            break;
        case BYD_AC_CONTROL_VENTILATE:
            data[0] = 0X00;
            data[1] = 0X00;
            data[2] = 0X00;
            data[3] = 0X00;
            data[4] = 0X00;
            data[5] = 0X00;
            data[6] = 0X00;
            data[7] = 0X04;
            byd_ac_frame_send(BYD_AC_CONTROL_2_CAN_ID, data);
            return;
        default:
            return;
    }
    changeHex2();
    data[2] = hex2;
    byd_ac_frame_send(BYD_AC_CONTROL_1_CAN_ID, data);
}
