/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "Remote.h"

#include "main.h"

#include "bsp_usart.h"
#include "string.h"



//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;


//取正函数
static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
void sbus_to_rc(uint8_t DmaBufNmb);

//remote control data 
//遥控器控制变量
RC_ctrl_t rc_ctrl;
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];


/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    usart3_rx_dma_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //禁止使用go to语句！！！！！！
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        return 1;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        return 1;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        return 1;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        return 1;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        return 1;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        return 1;
    }
    return 0;

}

void slove_RC_lost(void)
{
    usart3_rx_dma_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    usart3_rx_dma_restart(SBUS_RX_BUF_NUM);
}


//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
void sbus_to_rc(uint8_t DmaBufNmb)
{

    rc_ctrl.rc.ch[0] = (sbus_rx_buf[DmaBufNmb][0] | (sbus_rx_buf[DmaBufNmb][1] << 8)) & 0x07ff;         //!< Channel 0
    rc_ctrl.rc.ch[1] = ((sbus_rx_buf[DmaBufNmb][1] >> 3) | (sbus_rx_buf[DmaBufNmb][2] << 5)) & 0x07ff;  //!< Channel 1
    rc_ctrl.rc.ch[2] = ((sbus_rx_buf[DmaBufNmb][2] >> 6) | (sbus_rx_buf[DmaBufNmb][3] << 2) |           //!< Channel 2
                         (sbus_rx_buf[DmaBufNmb][4] << 10)) &0x07ff;
    rc_ctrl.rc.ch[3] = ((sbus_rx_buf[DmaBufNmb][4] >> 1) | (sbus_rx_buf[DmaBufNmb][5] << 7)) & 0x07ff;  //!< Channel 3
    rc_ctrl.rc.s[0] = ((sbus_rx_buf[DmaBufNmb][5] >> 4) & 0x0003);                                      //!< Switch left
    rc_ctrl.rc.s[1] = ((sbus_rx_buf[DmaBufNmb][5] >> 4) & 0x000C) >> 2;                                 //!< Switch right
    rc_ctrl.mouse.y = -(sbus_rx_buf[DmaBufNmb][6] | (sbus_rx_buf[DmaBufNmb][7] << 8));                     //!< Mouse X axis
    rc_ctrl.mouse.x = -(sbus_rx_buf[DmaBufNmb][8] | (sbus_rx_buf[DmaBufNmb][9] << 8));                     //!< Mouse Y axis
    rc_ctrl.mouse.z = sbus_rx_buf[DmaBufNmb][10] | (sbus_rx_buf[DmaBufNmb][11] << 8);                   //!< Mouse Z axis
    rc_ctrl.mouse.press_l = sbus_rx_buf[DmaBufNmb][12];                                                 //!< Mouse Left Is Press ?
    rc_ctrl.mouse.press_r = sbus_rx_buf[DmaBufNmb][13];                                                 //!< Mouse Right Is Press ?
    rc_ctrl.key.v = sbus_rx_buf[DmaBufNmb][14] | (sbus_rx_buf[DmaBufNmb][15] << 8);                     //!< KeyBoard value
    rc_ctrl.rc.ch[4] = sbus_rx_buf[DmaBufNmb][16] | (sbus_rx_buf[DmaBufNmb][17] << 8);                  //NULL

    rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;
}





//键盘应用功能
//键盘区域分为长按触发的功能区和点按触发的功能区
//其中 W、A、S、D、SHIFT、CTRL、F 为长按触发的功能区
//其他按键为点按功能区

//长按触发区域的键值检测

bool_t CheakKeyPress(uint16_t Key)
{
    if ((rc_ctrl.key.v & Key) == 0)
        return 0;
    
    return 1;
}



//边沿触发检测
uint16_t KeyFormerChannal = 0;
uint16_t KeyJumpChannal = 0;

bool_t CheakKeyPressOnce(uint16_t Key)
{
    if ((rc_ctrl.key.v & Key) == 0){
        KeyJumpChannal = (rc_ctrl.key.v ^ KeyFormerChannal);
        KeyFormerChannal = rc_ctrl.key.v;
        return 0;
    }
    else{
        KeyJumpChannal = (rc_ctrl.key.v ^ KeyFormerChannal);
        KeyFormerChannal = rc_ctrl.key.v;
        if ((KeyJumpChannal & Key) == 0){
            return 0;
        }
        else{
            return 1;
        }
    }
}


// 归一化摇杆值
fp32 RemoteChannalRightX()
{
    return (rc_ctrl.rc.ch[1] / 660.0f);
}
fp32 RemoteChannalRightY()
{
    return (-rc_ctrl.rc.ch[0] / 660.0f);
}
fp32 RemoteChannalLeftX()
{
    return (rc_ctrl.rc.ch[3] / 660.0f);
}
fp32 RemoteChannalLeftY()
{
    return (-rc_ctrl.rc.ch[2] / 660.0f);
}
fp32 RemoteDial()
{
    return (rc_ctrl.rc.ch[4] / 660.0f);
}

// 归一化鼠标移动
fp32 MouseMoveX()
{
    return (rc_ctrl.mouse.x / 32768.0f);
}
fp32 MouseMoveY()
{
    return (rc_ctrl.mouse.y / 32768.0f);
}

// 鼠标左右键
bool_t MousePressLeft()
{
    return rc_ctrl.mouse.press_l;
}
bool_t MousePressRight()
{
    return rc_ctrl.mouse.press_r;
}

// 拨杆位置检测
bool_t SwitchRightUpSide()
{
    return (rc_ctrl.rc.s[0] == RC_SW_UP);
}
bool_t SwitchRightMidSide()
{
    return (rc_ctrl.rc.s[0] == RC_SW_MID);
}
bool_t SwitchRightDownSide()
{
    return (rc_ctrl.rc.s[0] == RC_SW_DOWN);
}
bool_t SwitchLeftUpSide()
{
    return (rc_ctrl.rc.s[1] == RC_SW_UP);
}
bool_t SwitchLeftMidSide()
{
    return (rc_ctrl.rc.s[1] == RC_SW_MID);
}
bool_t SwitchLeftDownSide()
{
    return (rc_ctrl.rc.s[1] == RC_SW_DOWN);
}

fp32 NormalizedLimit(fp32 input) {
    if (input > 1.0f) {
        input = 1.0f;
    }
    else if (input < -1.0f) {
        input = -1.0f;
    }
    return input;
}

