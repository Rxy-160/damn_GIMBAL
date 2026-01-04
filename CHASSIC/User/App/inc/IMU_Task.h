#ifndef __IMU_TASK_H
#define __IMU_TASK_H

#include "main.h"
#include "tim.h"
#include "QuaternionEKF.h"
#include "pid_temp.h"
#include "bsp_imu_pwm.h"
#include "kalman_filter.h"
#include "BMI088driver.h"

typedef struct
{
	float gyro_correct[3];
	float gyro[3];
	float accel[3];
	float temp;
	
	float q[4];

	float pitch;
	float roll;
	float yaw;
    float YawTotalAngle;

    uint8_t attitude_flag;
    uint32_t correct_times;
}IMU_Data_t;

void INS_Task(IMU_Data_t *IMU, pid_type_def *imu_temp_pid);

#endif
