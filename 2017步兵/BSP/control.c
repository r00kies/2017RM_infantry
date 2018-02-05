#include "control.h"
#include "math.h"
#include "MPU6050.h"
#include "usart1.h"
#include "DEBUG.h"
float Time_dt;
float pid_roll;
float pid_pitch;
float pid_yaw;
float Motor_Ail=0.0;					 
float Motor_Ele=0.0;					
float Motor_Thr=0.0;					 
float Motor_Rud=0.0;					 
#define PWM_Period 1000
PID_Struct PID_Rammer_moto;
PID_Struct PID_Yaw_V_Struct;			    
PID_Struct PID_Pitch_V_Struct;
PID_Struct PID_Roll_V_Struct;
PID_Struct PID_Yaw_P_Struct;						   
PID_Struct PID_Pitch_P_Struct;
PID_Struct PID_Roll_P_Struct;
PID_Struct Chassis_Position_Control;//底盘pid
/*************************
      初始化PID参数
*************************/
void PID_Init(PID_Struct *PID)
{
  PID->expectation       = 0.0;            //遥控给的期望值
  PID->Err_k			       = 0.0;            //当前误差值e(k)
  PID->Err_k_1		       = 0.0;            //k-1时刻误差值e(k-1)
  PID->Err_k_2		       = 0.0;            //k-2时刻误差值e(k-2)
  PID->SumErr              = 0.0;		//误差和
  PID->Kp				   = 0.0;           //比例系数，通过串口在线调PID参数再写入Flash
  PID->Ti				   = 0.0;           //积分系数，通过串口在线调PID参数再写入Flash
  PID->Td				   = 0.0;           //微分系数，通过串口在线调PID参数再写入Flash
  PID->Ouput_deltaUk       = 0.0;		    //PID计算后的输出量U(k) - U(k-1)
  PID->Ouput_deltaUk_Max   = PWM_Period;		    //限制输出量最大值
  PID->Ouput_deltaUk_Min   = -PWM_Period;		    //限制输出量最小值
  PID->PID_Integral_Max    = 300.0;		     //限制积分项最大值
  PID->PID_Integral_Min    = -300.0;			//限制积分项最小值
}
void PID_GimBal_Init(PID_Struct *PID)
{
  PID->expectation       = 0.0;            //遥控给的期望值
  PID->Err_k			       = 0.0;            //当前误差值e(k)
  PID->Err_k_1		       = 0.0;           //k-1时刻误差值e(k-1)
  PID->Err_k_2		       = 0.0;           //k-2时刻误差值e(k-2)
  PID->SumErr              = 0.0;			//误差和
  PID->Kp				   = 0.0;           //比例系数，通过串口在线调PID参数再写入Flash
  PID->Ti				   = 0.0;           //积分系数，通过串口在线调PID参数再写入Flash
  PID->Td				   = 0.0;           //微分系数，通过串口在线调PID参数再写入Flash
  PID->Ouput_deltaUk       = 0.0;		    //PID计算后的输出量U(k) - U(k-1)
  PID->Ouput_deltaUk_Max   = 5000;		    //限制输出量最大值
  PID->Ouput_deltaUk_Min   = -5000;		    //限制输出量最小值
  PID->PID_Integral_Max    = 4000.0;		     //限制积分项最大值
  PID->PID_Integral_Min    = -4000.0;			//限制积分项最小值
}
void PID_Rammer_moto_Init(PID_Struct *PID)
{
  PID->expectation         = 0.0;           //遥控给的期望值
  PID->Err_k			   = 0.0;           //当前误差值e(k)
  PID->Err_k_1		       = 0.0;           //k-1时刻误差值e(k-1)
  PID->Err_k_2		       = 0.0;           //k-2时刻误差值e(k-2)
  PID->SumErr              = 0.0;			//误差和
  PID->Kp				   = 5;           //比例系数，通过串口在线调PID参数再写入Flash
  PID->Ti				   = 1;           //积分系数，通过串口在线调PID参数再写入Flash
  PID->Td				   = 0.0;           //微分系数，通过串口在线调PID参数再写入Flash
  PID->Ouput_deltaUk       = 0.0;		    //PID计算后的输出量U(k) - U(k-1)
  PID->Ouput_deltaUk_Max   = 1000;		    //限制输出量最大值
  PID->Ouput_deltaUk_Min   = -1000;		    //限制输出量最小值
  PID->PID_Integral_Max    = 1000.0;		//限制积分项最大值
  PID->PID_Integral_Min    = -1000.0;		//限制积分项最小值
}

void pid_init_struct(void)
{
	PID_GimBal_Init(&PID_Yaw_V_Struct);
//	PID_Init(&PID_Roll_V_Struct);
  PID_GimBal_Init(&PID_Pitch_V_Struct);
//	
  PID_GimBal_Init(&PID_Yaw_P_Struct);
//	PID_Init(&PID_Roll_P_Struct);
  PID_GimBal_Init(&PID_Pitch_P_Struct);
	PID_Rammer_moto_Init(&PID_Rammer_moto);
	PID_Init(&Chassis_Position_Control);
	
}

/***************************************************************
PID计算-增量式
***************************************************************/
float PIDz_Calculate(PID_Struct* PID, float measured, float expect)
{
     float K_p = PID->Kp;
     float T_d = PID->Td;
     float T_i = PID->Ti;
     float increment;
    PID->Err_k=PID->Err_k_1;
    PID->Err_k_1=PID->Err_k_2;
    PID->Err_k_2=(expect-measured)*100;
    increment = (K_p+T_i+T_d)*PID->Err_k_2-(K_p+2*T_d)*PID->Err_k_1+T_d*PID->Err_k;
    PID->Ouput_deltaUk+=increment;
    if(PID->Ouput_deltaUk > PID->Ouput_deltaUk_Max)
    {
        PID->Ouput_deltaUk = PID->Ouput_deltaUk_Max;
    }
    if(PID->Ouput_deltaUk < PID->Ouput_deltaUk_Min)
    {
        PID->Ouput_deltaUk = PID->Ouput_deltaUk_Min;
    }
    
  return PID->Ouput_deltaUk;
}

/***************************************************************
PID计算-位置式
***************************************************************/
float PID_Calculate(PID_Struct* PID, float measured, float expect)
{
  float Value_Proportion;  
  float Value_Integral;		
  float Value_Derivative;	

  PID->expectation =  expect;
  PID->Err_k = (PID->expectation - measured)*10;
  PID->SumErr+= PID->Err_k;

  //P I D
  Value_Proportion    = PID->Kp * PID->Err_k;
  Value_Integral      =  PID->SumErr * PID->Ti;	
  Value_Derivative  = PID->Kp * PID->Td * (PID->Err_k - PID->Err_k_1);

  if(Value_Integral > PID->PID_Integral_Max)
  {
    PID->SumErr -= PID->Err_k;
	Value_Integral = PID->PID_Integral_Max;
  }
  if(Value_Integral < PID->PID_Integral_Min)
  {
  	PID->SumErr -= PID->Err_k;
    Value_Integral = PID->PID_Integral_Min;
  }
  
  PID->Ouput_deltaUk = Value_Proportion + Value_Integral + Value_Derivative;

  if(PID->Ouput_deltaUk > PID->Ouput_deltaUk_Max)
  {PID->Ouput_deltaUk = PID->Ouput_deltaUk_Max;}
  if(PID->Ouput_deltaUk < PID->Ouput_deltaUk_Min)
  {PID->Ouput_deltaUk = PID->Ouput_deltaUk_Min;}

  PID->Err_k_1 = PID->Err_k;	  //保存k-1次误差值
  
  return PID->Ouput_deltaUk;
}

 
