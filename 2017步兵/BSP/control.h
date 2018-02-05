#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f4xx.h"
#define LED	P4.3
#define AilMiddle        1501        
#define EleMiddle        1501        
#define RudMiddle        1501        
#define PWM_MOTOR_MIN    2000	      
#define PWM_MOTOR_MAX    4000		 
extern float Time_dt;
extern float pid_roll;
extern float pid_pitch;
extern float pid_yaw;
extern float Motor_Ail;					  
extern float Motor_Ele;					   
extern float Motor_Thr;					 
extern float Motor_Rud;					  

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
extern PID_Struct PID_Rammer_moto;
extern PID_Struct PID_Yaw_V_Struct;	
extern PID_Struct PID_Roll_V_Struct;			    
extern PID_Struct PID_Pitch_V_Struct;
extern PID_Struct PID_Yaw_P_Struct;				
extern PID_Struct PID_Roll_P_Struct;			   
extern PID_Struct PID_Pitch_P_Struct;
extern PID_Struct Chassis_Position_Control;
void PID_Rammer_moto_Init(PID_Struct *PID);
float Limit_PWMOUT(float MOTOR);
void MOTOR_CAL(void);
void Change_InputPWM_To_Expect(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4);
void PID_Init(PID_Struct *PID);
void pid_init_struct(void);
float PIDz_Calculate(PID_Struct* PID, float measured, float expect);
float PID_Calculate(PID_Struct* PID, float measured, float expect);
#endif
