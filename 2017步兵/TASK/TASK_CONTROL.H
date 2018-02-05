#ifndef _TASK_CONTROL_H_
#define _TASK_CONTROL_H_
#include "stm32f4xx.h"
#include "math.h"
#include "MPU6050.h"
#include "mpu6050_driver.h"
#include "mpu6050_process.h"
#include "usart1.h"
#include "TASK_DBUG.h"
#include "usart2.h"
#include "task_can.h"
#include "pwm.h"  
#include "math.h"
#include "TASK_key_board.H"
#include "mpu6050_interrupt.h"
#define abs(x) ((x)>0? (x):(-(x)))

typedef struct {
  float expectation;          
  float Err_k;			  
  float Err_k_1;		  
  float Err_k_2;		 
  float SumErr;             
  float Kp;				 
  float Ti;				
  float Td;				 
  float Ouput_deltaUk;		
  float Ouput_deltaUk_Max;		
  float Ouput_deltaUk_Min;		
  float PID_Integral_Max;				
  float PID_Integral_Min;				
} PID_Struct;
typedef struct {
float current_dir[4];    
float power[4];
} Moto_Struct;
extern PID_Struct PID_Rammer_moto;
extern PID_Struct PID_Yaw_V_Struct;				    
extern PID_Struct PID_Pitch_V_Struct;
extern PID_Struct PID_Yaw_P_Struct;						   
extern PID_Struct PID_Pitch_P_Struct;
extern PID_Struct Chassis_Position_Control;
extern PID_Struct RM3510_Speed_Control_1;
extern PID_Struct RM3510_Speed_Control_2;
extern PID_Struct RM3510_Speed_Control_3;
extern PID_Struct RM3510_Speed_Control_4;//底盘pid
extern PID_Struct PID_Power_Struct_1;//底盘功率环
extern PID_Struct PID_Power_Struct_2;//底盘功率环
extern PID_Struct PID_Power_Struct_3;//底盘功率环
extern PID_Struct PID_Power_Struct_4;//底盘功率环
extern PID_Struct PID_Power_Struct;
extern s16 pwm_set_1,pwm_set_2,pwm_set_3,pwm_set_4;
extern s16 angle_pitch_OFFSET,angle_yaw_OFFSET;
///////////////////////////////////////////////////
void pid_init_struct(void);
float PIDz_Calculate(PID_Struct* PID, float measured, float expect);
float PID_Calculate(PID_Struct* PID, float measured, float expect);
float PID_Calculate_yaw_p(PID_Struct* PID, float measured, float expect);
float PID_Calculate_pitch(PID_Struct* PID, float measured, float expect );
void PID_Init_chassis(PID_Struct *PID);
void PID_init_RM3510(PID_Struct *PID);
void PID_GimBal_Init(PID_Struct *PID);
void PID_init_power(PID_Struct *PID);
void PID_Rammer_moto_Init(PID_Struct *PID);
///////////////////////////////
void Friction_wheel_control (s16 speed,u8 mode);
 void RM3510_control();
s16 RM3510_current_confine(s16 Pwm);
s16 transe_M_angle(s16 angle,u16 first_angle);
void angle_CALIBRATE(void);
 ///////////////////////////////
void Gimban_ctrl(void);
void Gimban_Camera_ctrl(void);
void control_task(void *pdata); 

#endif