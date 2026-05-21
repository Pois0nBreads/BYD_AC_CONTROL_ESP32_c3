#include "driver/twai.h"

#define BYD_AC_INTERNAL_CIRCULATION     0x02 //内循环标记
#define BYD_AC_EXTERNAL_CIRCULATION     0x01 //外循环标记

#define BYD_AC_FAN_FORWARD              0x01 //风向  前吹标记命令
#define BYD_AC_FAN_FORWARD_DOWN         0x02 //风向前下吹标记命令
#define BYD_AC_FAN_DOWN                 0x03 //风向  下吹标记命令
#define BYD_AC_FAN_DOWN_UP              0x04 //风向上下吹标记命令
#define BYD_AC_FAN_UP                   0x05 //风向上  吹标记命令
#define BYD_AC_FAN_ALL                  0x06 //风向  全吹标记命令
#define BYD_AC_FAN_FORWARD_UP           0x07 //风向前上吹标记命令

#define BYD_AC_CLOSE_FLAG               0x01 //自动空调关闭标记
#define BYD_AC_AUTO_FLAG                0x02 //自动空调自动标记
#define BYD_AC_FRONT_DEFROSTER_FLAG     0x03 //自动空调除霜标记
#define BYD_AC_VENTILATE_FLAG           0x04 //自动空调通风标记
#define BYD_AC_MANUEL_FLAG              0x05 //自动空调手动标记

#define BYD_AC_CONTROL_CLOSE            0x01 //自动空调关闭命令
#define BYD_AC_CONTROL_AUTO             0x02 //自动空调自动命令
#define BYD_AC_CONTROL_FRONT_DEFROSTER  0x03 //自动空调除霜命令
#define BYD_AC_CONTROL_VENTILATE        0x04 //自动空调通风命令

typedef struct {
    bool isAcFrame;         /**<是否空调报文*/
    bool ig2;               /**<车辆是否处于IG2*/
    uint8_t left_temp;      /**<左分区温度*/
    uint8_t right_temp;     /**<右分区温度*/
    uint8_t fan_speed;      /**<风速*/
    uint8_t fan_mode;       /**<风向*/
    uint8_t ac_mode;        /**<空调模式*/
    uint8_t cycle_mode;     /**<循环模式*/
    bool defroster_open;    /**<后除雾器是否打开*/
    bool compressor_open;   /**<压缩机是否打开*/
} byd_ac_state_t;

esp_err_t byd_ac_can_handle(twai_message_t *msg, byd_ac_state_t *info);

void byd_can_ac_left_temp_up(void);
void byd_can_ac_left_temp_down(void);
void byd_can_ac_right_temp_up(void);
void byd_can_ac_right_temp_down(void);
void byd_can_ac_fan_speed_up(void);
void byd_can_ac_fan_speed_down(void);
void byd_can_ac_defroster_open(void);
void byd_can_ac_defroster_close(void);
void byd_can_ac_compressor_open(void);
void byd_can_ac_compressor_close(void);
void byd_can_ac_fan_mode(uint8_t mode);
void byd_can_ac_control_mode(uint8_t mode);
void byd_can_ac_internal_cycle_mode(void);
void byd_can_ac_external_cycle_mode(void);