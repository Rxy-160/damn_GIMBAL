#include "Power_Ctrl.h"

float P_predict;//

void Power_control_init(model_t *model)
{
    model->PID_Buffer.Kp = 2;
    model->PID_Buffer.Ki = 0;
    model->PID_Buffer.Kd = 0;
    model->PID_Buffer.ILt = 0;
    model->PID_Buffer.AlLt = 100;

    model->toque_coefficient = 1.99688994e-6f;// (20/16384)*(0.3)*(187/3591)/9.55 力矩电流系数
    model->a = 1.23e-07f;// k1
    model->k2 = 1.453e-07f;// k2
    model->constant = 4.081f;// a 增大这个系数可以减小功率，反之增加
	
		model->g1 = 1.5756155501e-02f;//kt：M3508鼙鼓的转矩常数Nm/A,对应机械功率项，与转速和电流乘积成正比
    model->g2 = 1.1584598349e-01f;//kr：M3508鼙鼓和C620电调的电阻Ω，对应铜损项，与电流平方成正比
    model->g3 = 1.9202168378e-05f;//k_iron：M3508电机铁损系数 (W/(rad/s)²)，对应铁损项（磁滞/涡流损耗），与转速平方成正比
    model->g4 = 2.1291187956e+00f;//k0：M3508电机和C620电调的静态功率W，对应固定损耗项，与转速和电流无关

    model->rpm_to_rad = 2.0f * 3.1415926f / 60.0f;//RPM转rad/s
	
}



float initial_power;

/**
  * @author: 楠
  * @performance: 电机功率计算函数
  * @parameter: 电机结构体
  * @time: 24-4-1
  * @ReadMe: 依靠电机功率模型计算电机功率
 */
float get_initial_power(DJI_MOTOR_Typedef *MOTOR, model_t *model)
{
float w = MOTOR->DATA.Speed_now * model->rpm_to_rad;   // rad/s
    float I = MOTOR->PID_S.Output * 20 / 16384;

    float power = model->g1 * w * I
                + model->g2 * I * I
                + model->g3 * w * w
                + model->g4;

    if(power < 0) power = 0;

    return power;}

/**
  * @author: 楠
  * @performance: 电机功率限制函数
  * @parameter: 电机结构体
  * @time: 24-4-1
  * @ReadMe: 对电机功率进行缩放进行功率再分配
 */
void chassis_power_limit(DJI_MOTOR_Typedef *MOTOR, uint8_t p, model_t *model)
{
    if(p < 0)   return;

    float speed_rpm = (float)MOTOR->DATA.Speed_now;
	  //                  转矩常数          速度
    float b = model->toque_coefficient * speed_rpm;
	  //                    k2*w2                               
    float c = model->k2 * speed_rpm * speed_rpm - model->scaled_give_power[p-1] + model->constant;
    //                                                    最大持续电功率瞬时值
   //                                该值已按整车/底盘总功率预算、优先级比例及安全裕度完成缩放分配
	
    if(MOTOR->PID_S.Output > 0)  // 根据原力矩方向选择计算公式
    {
        float temp = (-b + Sqrt(b * b - 4 * model->a * c)) / (2 * model->a);
        if(temp > 16000)
        {
            MOTOR->PID_S.Output = 16000;
        }else{
            MOTOR->PID_S.Output = temp;
        }
    }else 
		{
        float temp = (-b - Sqrt(b * b - 4 * model->a * c)) / (2 * model->a);
        if (temp < -16000)
        {
            MOTOR->PID_S.Output = -16000;
        }
				else
				{
            MOTOR->PID_S.Output = temp;
        }
    }
}





float SectionLimit_f(float max, float min, float data)
{
    float temp = 0.0f;
    if(max >= min)
    {
        if(data >= max)
        {
            return max;
        }
        else if( data <= min)
        {
            return min;
        }
        else
        {
            return data;
        }
    }
    else
    {
        temp = min ;
        min = max;
        max = temp;

        if(data >= max)
        {
            return max;
        }
        else if( data <= min)
        {
            return min;
        }
        else
        {
            return data;
        }
    }
}

/**
  * @author: 楠
  * @performance: 缓冲能量PID计算
  * @parameter: PID 缓冲能量 要求剩余的最低缓冲能量
  * @time: 24-4-1
  * @ReadMe:
 */
void PID_buffer(PID_buffer_t *PID_buffer, float power_buffer, float temp)
{
    PID_buffer->Error[0] = temp - power_buffer;
    /*比例输出*/
    PID_buffer->P_out = (PID_buffer->Error[0] * PID_buffer->Kp);
    /*积分输出*/
    PID_buffer->I_out += (PID_buffer->Error[0] * PID_buffer->Ki);
    /*积分限幅*/
    PID_buffer->I_out = SectionLimit_f(PID_buffer->ILt, -PID_buffer->ILt, PID_buffer->I_out);
    /*微分输出*/
    PID_buffer->D_out = -(PID_buffer->Error[0] - PID_buffer->Error[1]) * PID_buffer->Kd;
    /*数据迭代*/
    PID_buffer->Error[1] = PID_buffer->Error[0];
    /*角度环总输出*/
    PID_buffer->All_out = (PID_buffer->P_out + PID_buffer->I_out + PID_buffer->D_out);
    /*总输出限幅*/
    PID_buffer->All_out = SectionLimit_f(PID_buffer->AlLt, -PID_buffer->AlLt, PID_buffer->All_out);
}

void chassis_power_distribute(DJI_MOTOR_Typedef *motor[4],
                              float I_cmd[4],
                              float P_limit,
                              model_t *model)
{
    float A = 0.0f;
    float B = 0.0f;
    float C = 4 - P_limit;

    for(int i = 0; i < 4; i++)
    {
        float w = motor[i]->DATA.Speed_now * model->rpm_to_rad;

        // ⚠️ 和你前面统一：必须一致！！
        float I = I_cmd[i] * 20.0f / 16384.0f;

        A += model->g2 * I * I;
        B += model->g1 * w * I;
        C += model->g3 * w * w;
    }

    // ✅ 功率预测（是否需要限功率）
    float P_predict = A + B + C + P_limit;

    if(P_predict <= P_limit)
    {
        return; // 不超功率，不做任何处理
    }

    float s = 1.0f;

    if(A < 1e-6f)
    {
        s = 0.0f;
    }
    else
    {
        float discriminant = B * B - 4.0f * A * C;

        if(discriminant >= 0.0f)
        {
            s = (-B + sqrtf(discriminant)) / (2.0f * A);
        }
        else
        {
            s = 0.0f;
        }
    }

    if(s > 1.0f) s = 1.0f;
    if(s < 0.0f) s = 0.0f;

    // ✅ 统一缩放
    for(int i = 0; i < 4; i++)
    {
        I_cmd[i] *= s;
    }
}

float I_cmd[4];

    //*可编辑部分*begin*//
    const int16_t PowerCompensation = 15;  //正常模式下的功率补偿
    const uint16_t SuperMaxPower = 200;	    //超级电容下的功率补偿
    const uint16_t capValt = 170;	        //强制退出的电压阈值
    //*可编辑部分*end*//

    uint16_t max_power_limit = 50;//80;  //最大功率限制
    float input_power = 0;		    // 输入功率（裁判系统）
    float chassis_max_power = 0;
    float initial_give_power[4];    // 初始功率由PID计算以及电机数据得到
    float initial_total_power = 0;

/**
  * @author: 楠
  * @performance: 功率控制总函数
  * @parameter: 电容标志位（是否开启）
  * @time: 24-4-1
  * @ReadMe: 放在底盘PID解算后即可
 */
uint8_t chassis_power_control(CONTAL_Typedef *RUI_V_CONTAL_V,
                           User_Data_T *usr_data,
                           model_t *model,
                           CAP_RXDATA *CAP_GET,
                           MOTOR_Typdef *MOTOR)
{
//                             机器人底盘功率上限
    if(usr_data->robot_status.chassis_power_limit != 0 )
    {
        max_power_limit = usr_data->robot_status.chassis_power_limit;	// 得到最大功率限制
    }
    float chassis_power = P_predict;//usr_data->power_heat_data.chassis_power_reserved;		// 得到底盘功率
    float chassis_power_buffer = usr_data->power_heat_data.buffer_energy;	// 得到缓冲能量

    /*没电容时开启*/     //                           缓冲能量？
    PID_buffer(&model->PID_Buffer, chassis_power_buffer, 25);  // 缓冲能量闭环
		
///裁判系统输入功率      最大功率限制（裁）- 缓冲能量闭环输出
    input_power = (float)max_power_limit - model->PID_Buffer.All_out;  // 加入缓冲能量
///////////////电容电压           电容电压最低阈值
    if(CAP_GET->capVolt     >     (float)capValt     )
    {
        if(RUI_V_CONTAL_V->BOTTOM.CAP == 0)
        {
            // 功率设置略大于最大输入功率，提高电容能量利用率
            chassis_max_power = input_power + (float)PowerCompensation;
        }else
        {
            // 开启电容
            chassis_max_power = input_power + (float)SuperMaxPower;
        }
    }
    else
    {
        // 电容电量低或电容离线时无补偿
        chassis_max_power = input_power;
    }

    //得到初始电机功率
		/////////当前功率
	initial_give_power[0] = get_initial_power(&MOTOR->DJI_3508_Chassis_1, model);
	initial_give_power[1] = get_initial_power(&MOTOR->DJI_3508_Chassis_2, model);
	initial_give_power[2] = get_initial_power(&MOTOR->DJI_3508_Chassis_3, model);
	initial_give_power[3] = get_initial_power(&MOTOR->DJI_3508_Chassis_4, model);
		

	for(uint8_t i = 0; i < 4; i++)
    {
        if (initial_give_power[i] < 0) // 不考虑负功(反向电动势)
            continue;
        initial_total_power += initial_give_power[i]; // 获得底盘总功率
				///总功4个轮功和
    }
		I_cmd[0] = MOTOR->DJI_3508_Chassis_1.PID_S.Output;
    I_cmd[1] = MOTOR->DJI_3508_Chassis_2.PID_S.Output;
    I_cmd[2] = MOTOR->DJI_3508_Chassis_3.PID_S.Output;
    I_cmd[3] = MOTOR->DJI_3508_Chassis_4.PID_S.Output;
//////////    if (initial_total_power > chassis_max_power) // 确定是否大于最大功率
//////////			{                   //     最大功率        /总功       （<1）
//////////        float power_scale = chassis_max_power / initial_total_power;

//////////        for(uint8_t i = 0; i < 4; i++)
//////////        {
//////////            model->scaled_give_power[i] = initial_give_power[i] * power_scale; // 获得缩放功率
//////////        }

//////////        //对每个电机分别进行功率限制
//////////        chassis_power_limit(&MOTOR->DJI_3508_Chassis_1, 1, model);
//////////        chassis_power_limit(&MOTOR->DJI_3508_Chassis_2, 2, model);
//////////        chassis_power_limit(&MOTOR->DJI_3508_Chassis_3, 3, model);
//////////        chassis_power_limit(&MOTOR->DJI_3508_Chassis_4, 4, model);
//////////    }
 DJI_MOTOR_Typedef *motor_ptr[4] = {
        &MOTOR->DJI_3508_Chassis_1,
        &MOTOR->DJI_3508_Chassis_2,
        &MOTOR->DJI_3508_Chassis_3,
        &MOTOR->DJI_3508_Chassis_4
    };

		chassis_power_distribute(motor_ptr,I_cmd,chassis_max_power ,model);
		MOTOR->DJI_3508_Chassis_1.PID_S.Output = I_cmd[0];
    MOTOR->DJI_3508_Chassis_2.PID_S.Output = I_cmd[1];
    MOTOR->DJI_3508_Chassis_3.PID_S.Output = I_cmd[2];
    MOTOR->DJI_3508_Chassis_4.PID_S.Output = I_cmd[3];

    return RUI_DF_READY;
}



////功率计接收解算函数
//void CAN_POWER_Rx(Power_Typedef* Power, uint8_t *rx_data)
//{
//    int16_t raw_shunt = (int16_t)((int16_t)rx_data[0] << 8 | rx_data[1]);
//    int16_t raw_bus   = (int16_t)((int16_t)rx_data[2] << 8 | rx_data[3]);
//    int16_t raw_curr  = (int16_t)((int16_t)rx_data[4] << 8 | rx_data[5]);
//    int16_t raw_pwr   = (int16_t)((int16_t)rx_data[6] << 8 | rx_data[7]);

//    Power->shunt_volt = (float)raw_shunt / 1000.0f;
//    Power->bus_volt   = (float)raw_bus   / 1000.0f;
//    Power->current    = (float)raw_curr  / 1000.0f;
//    //Power->power      = (float)raw_pwr   / 100.0f;
//    Power->power      = Power->bus_volt * Power->current;
//}
