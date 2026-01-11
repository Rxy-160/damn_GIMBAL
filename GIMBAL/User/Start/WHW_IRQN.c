/*
 * @Author: 王举人 11546637+wang-juren@user.noreply.gitee.com
 * @Date: 2024-11-12 20:51:02
 * @LastEditors: 王举人 11546637+wang-juren@user.noreply.gitee.com
 * @LastEditTime: 2024-11-16 10:21:36
 * @FilePath: \Horizon_Infantry\User\App\WHW_IRQN.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: 王举人 11546637+wang-juren@user.noreply.gitee.com
 * @Date: 2024-11-12 20:51:02
 * @LastEditors: 王举人 11546637+wang-juren@user.noreply.gitee.com
 * @LastEditTime: 2024-11-15 22:47:11
 * @FilePath: \Horizon_Infantry\User\App\WHW_IRQN.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 * 
 * 
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *            佛祖保佑     永不宕机     永无BUG
 * 
 *        佛曰:  
 *                写字楼里写字间，写字间里程序员；  
 *                程序人员写程序，又拿程序换酒钱。  
 *                酒醒只在网上坐，酒醉还来网下眠；  
 *                酒醉酒醒日复日，网上网下年复年。  
 *                但愿老死电脑间，不愿鞠躬老板前；  
 *                奔驰宝马贵者趣，公交自行程序员。  
 *                别人笑我忒疯癫，我笑自己命太贱；  
 *                不见满街漂亮妹，哪个归得程序员？
 */

/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-07-04 15:42:35
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-07 11:27:00
 */

#include "WHW_IRQN.h"
#include "Robot.h"

uint8_t move_G, move_S, move_C, move_P;
float t1,t2,dt;
static uint8_t TX[12] = {0xff,0xf1,0xfd,0x90,0x86,0xa7,0xff,0xf1,0xfd,0x90,0x86,0xa7};
//绝对延时
void DWT_DelayUs(uint32_t us)
{
    uint64_t start = DWT_GetTimeline_us();   /* 取当前绝对时间（µs） */
    uint64_t end   = start + us;             /* 计算终点时间 */

    /* 不超过 1 ms 直接忙等，保证精度 */
    if (us <= 1000u)
    {
        while (DWT_GetTimeline_us() < end) { /* 空转 */ }
    }
    else
    {
        /* 超过 1 ms 就间歇让出 CPU，防止任务饿死 */
        while (DWT_GetTimeline_us() < end)
        {
            vTaskDelay(1);  /* 约 1 ms（configTICK_RATE_Hz 默认 1000） */
        }
    }
}

//34ms,画UI任务
void StartRobotUITask(void const * argument)
{
    portTickType currentTimeRobotUI;
    currentTimeRobotUI = xTaskGetTickCount();

    //初始化UI界面
//    RobotUI_Static_Init();

    for (;;)
    {
//        RobotUI_Dynamic(RUI_ROOT_STATUS.RM_DBUS,
//                        RUI_V_CONTAL.SHOOT_Bask.Shoot_Number,
//                        IMU_Data.pitch,
//                        CAPDATE.GET.CAP_VOLT,
//                        ALL_MOTOR.DJI_3508_Shoot_M.DATA.Angle_now,
//                        ALL_MOTOR.DJI_3508_Shoot_L.DATA.Speed_now,
//                        ALL_MOTOR.DJI_3508_Shoot_R.DATA.Speed_now,
//                        &VISION_V_DATA);
//        osDelayUntil(&currentTimeRobotUI, 1);
//				vTaskDelayUntil(&currentTimeRobotUI, 1);
//			vTaskDelay (1);
    }
}

//运动控制任务
void StartMoveTask(void const * argument)
{
    portTickType currentTimeMove;
    currentTimeMove = xTaskGetTickCount();

//    //功率限制初始化
//    Power_control_init(&model);

//    //初始朝前的电机刻度
    RUI_V_CONTAL.CG.YAW_INIT_ANGLE = INIT_ANGLE;

//    //Pitch轴限幅
////	/////////注意p轴初始化
    RUI_V_CONTAL.HEAD.Pitch_MAX = 548.411;
    RUI_V_CONTAL.HEAD.Pitch_MIN = -321.443;

    for (;;)
    {
        /*底盘*/
//        RobotTask(1, &WHW_V_DBUS, &RUI_V_CONTAL, &User_data,
//                  &CAPDATE, &VISION_V_DATA, &RUI_ROOT_STATUS, &ALL_MOTOR,&IMU_Data);
//        move_C = chassis_task(&RUI_V_CONTAL,
//                              &RUI_ROOT_STATUS, &User_data, &model,
//                              &CAPDATE.GET, &ALL_MOTOR);
				
        /*云台*/
        RobotTask(2, &WHW_V_DBUS, &RUI_V_CONTAL, &User_data,
                 &CAPDATE, &VISION_V_DATA, &RUI_ROOT_STATUS, &ALL_MOTOR,&IMU_Data);
        move_G = gimbal_task(&RUI_V_CONTAL, &RUI_ROOT_STATUS, &ALL_MOTOR, &IMU_Data);
////双板通讯
			CANGimbalTX(&WHW_V_DBUS );/////双板通讯can发送

//			        DWT_Delay(/*&currentTimeRobotUI,*/ 50);
			
//	DWT_Delay_us(1000);		
//					vTaskDelayUntil(&currentTimeMove,1000);
//			vTaskDelay (1);
    }
}

//对抗控制任务(电容,发射)
void StartDefiantTask(void const * argument)
{
    portTickType currentTimeDefiant;
    currentTimeDefiant = xTaskGetTickCount();
MOTOR_PID_Shoot_INIT(&ALL_MOTOR);
    //发射机构初始化
//    RUI_V_CONTAL.SHOOT.Shoot_Speed = WIPE_MAX_SPEED;
//    RUI_V_CONTAL.SHOOT.Single_Angle = \;
		ATTACK_F_Init( &ALL_MOTOR );
    for(;;)
    {
        /*电容*/
//        Power_CAP_CAN_TX(&hcan2, 0x308, &CAPDATE.SET, &User_data);
					ATTACK_F_Ctl( &WHW_V_DBUS ,&ALL_MOTOR );
//        /*发射*/
//        RobotTask(4, &WHW_V_DBUS, &RUI_V_CONTAL, &User_data,
//                  &CAPDATE, &VISION_V_DATA, &RUI_ROOT_STATUS, &ALL_MOTOR,&IMU_Data);
//        move_S = shoot_task(&RUI_V_CONTAL, &RUI_ROOT_STATUS,&ALL_MOTOR);
//        DWT_Delay(/*&currentTimeRobotUI,*/ 50);
//					vTaskDelayUntil(&currentTimeDefiant,1);
        osDelayUntil(&currentTimeDefiant, 2);
//			vTaskDelay (1);
    }
}

//陀螺仪解算与自瞄发送任务
void StartIMUTask(void const * argument)
{
    portTickType currentTimeIMU;
    currentTimeIMU = xTaskGetTickCount();

    static uint32_t dt_pc = 0;
    static uint32_t INS_DWT_Count = 0;

    //陀螺仪初始化
    const float imu_temp_PID[3] = TEMPERATURE_PID;
    PID_init(&imu_temp, PID_POSITION, imu_temp_PID,
             TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
//    IMU_QuaternionEKF_Init(10, 0.001f, 10000000, 1, 0.001f,0); //ekf初始化
	    IMU_QuaternionEKF_Init(10, 0.001f, 10000000, 1, 0.001f,0); //ekf初始化

    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    while(BMI088_init()){}

    for(;;)
    {
        INS_Task(&IMU_Data, &imu_temp);
        dt_pc = (uint32_t)DWT_GetDeltaT(&INS_DWT_Count);
				Quaternion_testing(&IMU_Data);
			//视觉发送
        ControltoVision(&VISION_V_DATA.SEND ,sd_v_buff, 1,&User_data,&WHW_V_DBUS,&IMU_Data ,&VISION_V_DATA);
//        DWT_Delay(/*&currentTimeRobotUI,*/ 50);
//				vTaskDelayUntil(&currentTimeIMU,1);
        osDelayUntil(&currentTimeIMU, 1);
			vTaskDelay(1);
    }
}

//整车监控任务
void StartRootTask(void const * argument)
{
    portTickType currentTimeRoot;
    currentTimeRoot = xTaskGetTickCount();

    //使用基准电压来校准
//    init_vrefint_reciprocal();

    for(;;)
    {
        RUI_F_ROOT(&RUI_ROOT_STATUS, &WHW_V_DBUS, &ALL_MOTOR, &CAPDATE.GET);
        voltage = get_battery_voltage();
//        DWT_Delay(/*&currentTimeRobotUI,*/ 50);
//vTaskDelayUntil(&currentTimeRoot,5);
        osDelayUntil(&currentTimeRoot, 5);
			vTaskDelay(1);
    }
}

void BSP_TIM_IRQHandler(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM7) {
		TX[0]++;
		CANSPI_SEND(&hspi2, 0x201, TX);
	}
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //CAN接收函数
 *	@parameter:		    //
 *	@time:				//22-11-23 20:42
 *	@ReadMe:			//
 *  @LastUpDataTime:    //2023-04-20 02:52    bestrui
 *  @UpData：           //更新成共用体
 *  @LastUpDataTime:    //2023-05-06 20:23    bestrui
 *  @UpData：           //更新判断逻辑
 ************************************************************万能分隔符**************************************************************/
uint8_t test[8];
    uint8_t rx_data[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	CAN_RxHeaderTypeDef can_rx;

    HAL_CAN_GetRxMessage(hcan , CAN_RX_FIFO0 , &can_rx , rx_data);
    
	if (hcan == &hcan1)		
	{
		//CAN1
		switch (can_rx.StdId)
		{
//            case 0x205://拨弹
//                RUI_F_MOTOR_CAN_RX_3508RM(&ALL_MOTOR.DJI_3508_Shoot_M.DATA, rx_data);
//				memcpy(test, rx_data, 8);
//                break;

//            case 0x202://摩擦轮左
//                RUI_F_MOTOR_CAN_RX_3508RM(&ALL_MOTOR.DJI_3508_Shoot_L.DATA, rx_data);
//                break;

//            case 0x205://摩擦轮右
////                RUI_F_MOTOR_CAN_RX_3508RM(&ALL_MOTOR.DJI_3508_Shoot_R.DATA, rx_data);
//						                WHW_F_MOTOR_CAN_RX_6020RM(&ALL_MOTOR.m_dm4310_y_t, rx_data);

//                break;
            case 0x301://云台Yaw
						    dm4310_RXdata(&ALL_MOTOR.m_dm4310_y_t,rx_data);
                break;

            case 0x302://云台Pitch
                dm4310_RXdata(&ALL_MOTOR.m_dm4310_p_t, rx_data);
//					                WHW_F_MOTOR_CAN_RX_6020RM(&ALL_MOTOR.DJI_6020_Yaw.DATA, rx_data);
                break;
//            case 0x07:
//							dm4310_fbdata(&ALL_MOTOR.m_dm4310_p_t.DATA,rx_data);
//						break;
//						////双板 
						case CHASSIC_kong:
             GimbalRXResolve(rx_data,CHASSIC_kong);
						break;
						case 0x203://拨弹
		RUI_F_MOTOR_CAN_RX_2006RM(&ALL_MOTOR.DJI_3508_Shoot_M.DATA, rx_data);
		break;

        }
			
	}
	if (hcan == &hcan2)		
	{
		//CAN2
		switch (can_rx.StdId)
		{
            case 0x201://摩擦1
                RUI_F_MOTOR_CAN_RX_3508RM(&ALL_MOTOR.DJI_3508_Shoot_L.DATA, rx_data);
                break;

            case 0x202://摩擦2
                RUI_F_MOTOR_CAN_RX_3508RM(&ALL_MOTOR.DJI_3508_Shoot_R.DATA, rx_data);
                break;


//            case 0x204://底盘4
//                RUI_F_MOTOR_CAN_RX_3508RM(&ALL_MOTOR.DJI_3508_Chassis_4.DATA, rx_data);
//                break;

//            case 0x308://电容
//                Power_CAP_CAN_RX(&CAPDATE, rx_data);
//                break;
        }
	}
}

void WHW_MCP2515_Callback(SPI_HandleTypeDef *hspi, uCAN_MSG *rxMessage)
{
    if (hspi->Instance == SPI2) {
        uint8_t CAN_SPI_2_Data[8];
        memcpy(CAN_SPI_2_Data, &rxMessage->array[6], 8);
        switch (rxMessage->frame.id)
        {
            case 0x201:

                break;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SPI_CAN_2_EXIT_Pin) {
        uint8_t temp = WHW_CANSPI_Receive(&hspi2, &rxMessage1);
        if(temp)
            WHW_MCP2515_Callback(&hspi2, &rxMessage1);
        WHW_MCP2515_IRQHandler(&hspi2);
    }
}

#define  BUFFER_SIZE_6  (255)
#define  BUFFER_SIZE_1  (20)
#define  BUFFER_SIZE_3  (37)
		uint8_t data_length_6;

void BSP_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance ==USART3)//遥控接收串口
    {
        if (RESET != __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
        {
			uint16_t temp = 0;
            __HAL_UART_CLEAR_IDLEFLAG(&huart3);  //清除空闲中断标志（否则会一直不断进入中断）
			temp = huart3.Instance -> SR; // 清除SR状态寄存器
			temp = huart3.Instance -> DR; // 清除DR数据寄存器，用来清除中断
            // 下面进行空闲中断相关处理
            HAL_UART_DMAStop(&huart3);//暂时停止本次DMA传输，进行数据处理
            
            if(BUFFER_SIZE_3 - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx) == 18)
                RUI_F_DUBS_Resovled(DBUS_RX_DATA, &WHW_V_DBUS,&RUI_V_DBUS_UNION);

            HAL_UART_Receive_DMA(&huart3, (uint8_t *)DBUS_RX_DATA,37);  //重启开始DMA传输
        }
        
    }

    if(huart->Instance ==USART6)//裁判系统串口
    {
					    Read_Data_first(&ALL_RX , &User_data , data_length_6);//测试函数：待修改
            HAL_UART_Receive_DMA(&huart6,(uint8_t *)ALL_RX.Data,255);  //重启开始DMA传输

        if (RESET != __HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart6);  //清除空闲中断标志（否则会一直不断进入中断）
            // 下面进行空闲中断相关处理
//					
//            HAL_UART_DMAStop(&huart6);//暂时停止本次DMA传输，进行数据处理
//            
//            data_length_6  = BUFFER_SIZE_6 - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);//计算接收到的数据长度
//		    memset((uint8_t*)ALL_RX.Data,0,data_length_6);//清零接收缓冲区
        }
    }

    if(huart->Instance ==USART1)//调试串口
    {
		//数据处理
//1				 HAL_UART_Receive_DMA(&huart1, (uint8_t *)VISION_V_DATA.OriginData, sizeof(VISION_V_DATA.OriginData));
		uint8_t data_length_1;
        if (RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
        {
					//视觉记得注释回来
				 VISION_F_Cal(VISION_V_DATA.OriginData,0,&VISION_V_DATA);
				 HAL_UART_Receive_DMA(&huart1, (uint8_t *)VISION_V_DATA.OriginData, sizeof(VISION_V_DATA.OriginData));

//            __HAL_UART_CLEAR_IDLEFLAG(&huart1);  //清除空闲中断标志（否则会一直不断进入中断）
//            // 下面进行空闲中断相关处理
//            HAL_UART_DMAStop(&huart1);//暂时停止本次DMA传输，进行数据处理
//            
//            data_length_1 = BUFFER_SIZE_1 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);//计算接收到的数据长度
//		    memset((uint8_t *)RX, 0, data_length_1);

//            HAL_UART_Receive_DMA(&huart1, (uint8_t *)RX, 20);  //重启开始DMA传输
        }
    }
}

