/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history    2021-7-23   增加了串级pid的函数和接口
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V2.0.0     Jly-23-2021     Qylann          1. 增加部分函数
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"


enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          cascade pid struct data init
  * @param[out]     c_pid: cascade PID struct data point
  * @param[in]      oPID: 0: kp, 1: ki, 2:kd
  * @param[in]      iPID: 0: kp, 1: ki, 2:kd
  * @param[in]      oPID_max_out: outside pid max out
  * @param[in]      oPID_max_iout: outside pid max iout
  * @param[in]      iPID_max_out: inside pid max out
  * @param[in]      iPID_max_iout: inside pid max iout
  * @retval         none
  */
/**
  * @brief          cascade pid struct data init
  * @param[out]     c_pid: 差分PID结构数据指针
  * @param[in]      oPID: 0: kp, 1: ki, 2:kd
  * @param[in]      iPID: 0: kp, 1: ki, 2:kd
  * @param[in]      oPID_max_out: 外环pid最大输出
  * @param[in]      oPID_max_iout: 外环pid最大积分输出
  * @param[in]      iPID_max_out: 内环pid最大输出
  * @param[in]      iPID_max_iout: 内环pid最大积分输出
  * @retval         none
  */
extern void cascade_PID_init(cascade_pid_t *c_pid, const fp32 oPID[3], const fp32 iPID[3], fp32 oPID_max_out, fp32 oPID_max_iout, fp32 iPID_max_out, fp32 iPID_max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          cascade pid calculate 
  * @param[out]     c_pid: cascade PID struct data point
  * @param[in]      o_ref: outside feedback data 
  * @param[in]      i_ref: inside feedback data 
  * @param[in]      s_set: outside set point
  * @retval         cascade pid out
  */
/**
  * @brief          差分pid计算
  * @param[out]     c_pid: 差分PID结构数据指针
  * @param[in]      o_ref: 外环反馈数据
  * @param[in]      i_ref: 内环反馈数据
  * @param[in]      s_set: 外环设定值
  * @retval         差分pid输出
  */
extern fp32 cascade_PID_calc(cascade_pid_t *c_pid, fp32 o_ref, fp32 i_ref, fp32 o_set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

/**
  * @brief          cascade pid out clear
  * @param[out]     c_pid: cascade PID struct data point
  * @retval         none
  */
/**
  * @brief          差分pid输出清除
  * @param[out]     c_pid: 差分PID结构数据指针
  * @retval         none
  */
extern void cascade_PID_clear(cascade_pid_t *c_pid);

#endif
