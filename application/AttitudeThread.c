#include "AttitudeThread.h"

#include "main.h"

#include "cmsis_os.h"

#include "bsp_can.h"

#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bmi088driver.h"
#include "pid.h"

#include "MahonyAHRS.h"
#include "math.h"
#include <string.h>

#include "arm_math.h"
#include "user_lib.h"

#include "InterruptService.h"

#include "CanPacket.h"
#include "Setting.h"


#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定


/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp);

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void);


void AHRS_init(fp32 quat[4], fp32 accel[3]);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3]);
void get_angle(fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

extern SPI_HandleTypeDef hspi1;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};



volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;


bmi088_real_data_t bmi088_real_data;


static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;


fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad
fp32 INS_palstance[3] = {0.0f, 0.0f, 0.0f};

uint32_t IMU_Timer;

const fp32 *get_gyro_data_point(void);
const fp32 *get_INS_angle_point(void);
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU);
void GetCurrentQuaternion(fp32 q[4]);


/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void AttitudeThread(void const *pvParameters)
{
    //wait a time
    osDelay(INS_TASK_INIT_TIME);
    while(BMI088_init())
    {
        osDelay(100);
    }

    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);

    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);

    AHRS_init(INS_quat, bmi088_real_data.accel);

    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }


    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;//必须要初始化DMA之后才能去使能DMA，否则会出现只进一次DMA中断情况，此处留意

    while (1)
    {
        AHRS_update(INS_quat, 0.001f, bmi088_real_data.gyro, bmi088_real_data.accel);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
//        IMU_Timer = GetSystemTimer();
//        CanSendMessage(&COMMUNICATE_CANPORT, IMU_PACKET_TIME_ID, 4, (uint8_t *)&IMU_Timer);
//        CanSendMessage(&COMMUNICATE_CANPORT, IMU_PACKET_DATA0_ID, 8, (uint8_t *)&INS_quat[0]);
//        CanSendMessage(&COMMUNICATE_CANPORT, IMU_PACKET_DATA1_ID, 8, (uint8_t *)&INS_quat[2]);
        osDelay(1);//解算频率只需要高于云台任务频率即可
    }
}

void AHRS_init(fp32 quat[4], fp32 accel[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

const fp32 *get_gyro_data_point(void)
{
    return bmi088_real_data.gyro;
}
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}
void GetCurrentQuaternion(fp32 q[4])
{
    memcpy(q, INS_quat, sizeof(INS_quat));
}


#ifdef IMU_DIRECTION_xyz_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[0];
    IMU->PitchSpeed = INS_palstance[1];
    IMU->YawSpeed = INS_palstance[2];
}
#endif
#ifdef IMU_DIRECTION_yrxz_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, gyro[1], -gyro[0], gyro[2], accel[1], -accel[0], accel[2], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[1];
    IMU->PitchSpeed = -INS_palstance[0];
    IMU->YawSpeed = INS_palstance[2];
}
#endif
#ifdef IMU_DIRECTION_rxryz_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, -gyro[0], -gyro[1], gyro[2], -accel[0], -accel[1], accel[2], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = -INS_palstance[0];
    IMU->PitchSpeed = -INS_palstance[1];
    IMU->YawSpeed = INS_palstance[2];
}
#endif
#ifdef IMU_DIRECTION_ryxz_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, -gyro[1], gyro[0], gyro[2], -accel[1], accel[0], accel[2], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = -INS_palstance[1];
    IMU->PitchSpeed = INS_palstance[0];
    IMU->YawSpeed = INS_palstance[2];
}
#endif
#ifdef IMU_DIRECTION_zryx_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, gyro[2], -gyro[1], gyro[0]-0.0055f, accel[2], -accel[1], accel[0], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[2];
    IMU->PitchSpeed = -INS_palstance[1];
    IMU->YawSpeed = INS_palstance[0];
}
#endif
#ifdef IMU_DIRECTION_yzx_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, gyro[1], gyro[2], gyro[0], accel[1], accel[2], accel[0], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[1];
    IMU->PitchSpeed = INS_palstance[2];
    IMU->YawSpeed = INS_palstance[0];
}
#endif
#ifdef IMU_DIRECTION_rzyx_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, -gyro[2], gyro[1], gyro[0], -accel[2], accel[1], accel[0], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = -INS_palstance[2];
    IMU->PitchSpeed = INS_palstance[1];
    IMU->YawSpeed = INS_palstance[0];
}
#endif
#ifdef IMU_DIRECTION_ryrzx_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, -gyro[1], -gyro[2], gyro[0], -accel[1], -accel[2], accel[0], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = -INS_palstance[1];
    IMU->PitchSpeed = -INS_palstance[2];
    IMU->YawSpeed = INS_palstance[0];
}
#endif
#ifdef IMU_DIRECTION_xzry_XYZ
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[2], -gyro[1], accel[0], accel[2], -accel[1], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[0];
    IMU->PitchSpeed = INS_palstance[2];
    IMU->YawSpeed = -INS_palstance[1];
}
#endif





/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, 45.0f);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > 45.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_Accel_Pin)
    {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_Gyro_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    //开启陀螺仪的DMA传输
    if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    //开启加速度计的DMA传输
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
		
    if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}


void DMA2_Stream0_IRQHandler(void)
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //陀螺仪读取完毕
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_SET);
					
						BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro, INS_palstance);
        }

        //accel read over
        //加速度计读取完毕
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_SET);
					
						BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }
        //temperature read over
        //温度读取完毕
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
						HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_SET);
					
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
						imu_temp_control(bmi088_real_data.temp);
        }

        imu_cmd_spi_dma();//温度、加速度同时读取
    }
}
