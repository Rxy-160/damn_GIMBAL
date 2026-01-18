#include "IMU_Task.h"

#define correct_Time_define 10000    //上电去0飘 1000次取平均
#define temp_times 300       //探测温度阈值
#define Destination_TEMPERATURE 40.5f

/**
  * @brief          bmi088温度控制
  * @param[in]      argument: NULL
  * @retval         none
  */
void IMU_Temperature_Ctrl(IMU_Data_t *IMU, pid_type_def *imu_temp_pid)
{

	uint16_t tempPWM;
	//pid calculate. PID计算
	PID_calc(imu_temp_pid, IMU->temp, Destination_TEMPERATURE);
	if (imu_temp_pid->out < 0.0f)
	{
		imu_temp_pid->out = 0.0f;
	}
	tempPWM = (uint16_t)imu_temp_pid->out;
	SPI1_imu_pwm_set(tempPWM);
	
}

void INS_Task(IMU_Data_t *IMU, pid_type_def *imu_temp_pid)
{
    static uint32_t count = 0;

    // ins update
    if ((count % 1) == 0)
    {
        BMI088_read(IMU->gyro, IMU->accel, &IMU->temp);
        
        if(IMU->attitude_flag==2)  //ekf的姿态解算
        {                //0.8-0.99
					QEKF_INS.accLPFcoef = 0.9f;
			IMU->gyro[0]-=IMU->gyro_correct[0];   //减去陀螺仪0飘
			IMU->gyro[1]-=IMU->gyro_correct[1];
			IMU->gyro[2]-=IMU->gyro_correct[2];
          
			//===========================================================================
			//ekf姿态解算部分
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
			IMU_QuaternionEKF_Update(
				IMU->gyro[0],IMU->gyro[1],IMU->gyro[2],
				IMU->accel[0],IMU->accel[1],IMU->accel[2]);
			
			//=============================================================================== 
			//ekf获取姿态角度函数
			IMU->pitch=Get_Pitch();//获得pitch
			IMU->roll=Get_Roll();//获得roll
			IMU->yaw=Get_Yaw();//获得yaw
            IMU->YawTotalAngle=Get_YawTotalAngle();
			memcpy(IMU->q, QEKF_INS.q, 16);
			
			//==============================================================================
        }
        else if(IMU->attitude_flag==1)   //状态1 开始1000次的陀螺仪0飘初始化
        {
#ifdef User_Debug
            //gyro correct
            IMU->gyro_correct[0]+= IMU->gyro[0];
            IMU->gyro_correct[1]+= IMU->gyro[1];
            IMU->gyro_correct[2]+= IMU->gyro[2];

            IMU->correct_times++;
            if(IMU->correct_times>=correct_Time_define)
            {
				IMU->gyro_correct[0]/=correct_Time_define;
				IMU->gyro_correct[1]/=correct_Time_define;
				IMU->gyro_correct[2]/=correct_Time_define;

                IMU->attitude_flag=2; //go to 2 state
            }
#endif
//////获得零飘
#ifdef User_Release
            IMU->gyro_correct[0] = 0.00033012402127496898f;
            IMU->gyro_correct[1] = 0.00514394789934158325f;
            IMU->gyro_correct[2] = 0.00126171665508300066f;
            IMU->attitude_flag=2; //go to 2 state
#endif
        }
    }
// temperature control
    if ((count % 10) == 0)
    {
        // 100hz 的温度控制pid
        IMU_Temperature_Ctrl(IMU, imu_temp_pid);
        static uint32_t temp_Ticks=0;
#ifdef User_Debug
        if((fabsf(IMU->temp-Destination_TEMPERATURE-0.5f)<0.5f)&&IMU->attitude_flag==0) //接近额定温度之差小于0.5° 开始计数
#endif
#ifdef User_Release
        if(IMU->attitude_flag==0)//快速初始化
#endif
        {
          temp_Ticks++;
          if(temp_Ticks>temp_times)   //计数达到一定次数后 才进入0飘初始化 说明温度已经达到目标
          {
              IMU->attitude_flag=1;  //go to correct state
          }
        }
    }
    count++;
}

