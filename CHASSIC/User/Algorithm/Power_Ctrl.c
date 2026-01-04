#include "Power_Ctrl.h"

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

    int32_t speed_rpm = (int32_t)MOTOR->DATA.Speed_now;
       //             速度环输出（电流值）          转矩常数             当前速度                               
    initial_power = MOTOR ->PID_S.Output * model->toque_coefficient * (float)speed_rpm +
                    model->k2 * (float)speed_rpm * (float)speed_rpm +
                    model->a * MOTOR->PID_S.Output * MOTOR->PID_S.Output + model->constant;
///////////                         电流                     电流                
    return initial_power;
}

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
//                             机器人底盘功率上限
    if(usr_data->robot_status.chassis_power_limit != 0 )
    {
        max_power_limit = usr_data->robot_status.chassis_power_limit;	// 得到最大功率限制
    }
    float chassis_power = usr_data->power_heat_data.chassis_power_reserved;		// 得到底盘功率
    float chassis_power_buffer = usr_data->power_heat_data.buffer_energy;	// 得到缓冲能量

    /*没电容时开启*/     //                           缓冲能量？
    PID_buffer(&model->PID_Buffer, chassis_power_buffer, 25);  // 缓冲能量闭环
		
///裁判系统输入功率      最大功率限制（裁）- 闭环输出
    input_power = (float)max_power_limit - model->PID_Buffer.All_out;  // 加入缓冲能量
///////////////电容电压           电容电压最低阈值
    if(CAP_GET->CAP_VOLT    >     (float)capValt     )
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

    if (initial_total_power > chassis_max_power) // 确定是否大于最大功率
			{                   //     最大功率        /总功       （<1）
        float power_scale = chassis_max_power / initial_total_power;

        for(uint8_t i = 0; i < 4; i++)
        {
            model->scaled_give_power[i] = initial_give_power[i] * power_scale; // 获得缩放功率
        }

        //对每个电机分别进行功率限制
        chassis_power_limit(&MOTOR->DJI_3508_Chassis_1, 1, model);
        chassis_power_limit(&MOTOR->DJI_3508_Chassis_2, 2, model);
        chassis_power_limit(&MOTOR->DJI_3508_Chassis_3, 3, model);
        chassis_power_limit(&MOTOR->DJI_3508_Chassis_4, 4, model);
    }
    return RUI_DF_READY;
}
