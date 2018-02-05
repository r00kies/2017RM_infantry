#include "task_control.h"				 
////////////////////////////////////////////////////////
//变量定义
Moto_Struct  Moto_Drive;                         //3510电机变量
extern Chassis RM3510_DATA;                       //3510电机反馈值
s16 pwm_set_1,pwm_set_2,pwm_set_3,pwm_set_4;      //3510速度环输出值 用于DBUG时方便观察波形
///////////////////////////////////////////////////////

PID_Struct PID_Rammer_moto;//拨弹电机PID
PID_Struct PID_Yaw_V_Struct;//yaw速度环			    
PID_Struct PID_Pitch_V_Struct;//pitch速度环
PID_Struct PID_Yaw_P_Struct;	//yaw位置环
PID_Struct PID_Yaw_P_Struct_chassis;	//yaw位置环跟随底盘
PID_Struct PID_Pitch_P_Struct;//pitch位置环
PID_Struct Chassis_Position_Control;//底盘pid
PID_Struct RM3510_Speed_Control_1;//底盘speed pid
PID_Struct RM3510_Speed_Control_2;//底盘speed pid
PID_Struct RM3510_Speed_Control_3;//底盘speed pid
PID_Struct RM3510_Speed_Control_4;//底盘speed pid
PID_Struct PID_Power_Struct_1;//底盘功率环
PID_Struct PID_Power_Struct_2;//底盘功率环
PID_Struct PID_Power_Struct_3;//底盘功率环
PID_Struct PID_Power_Struct_4;//底盘功率环
extern volatile unsigned char sbus_rx_buffer[25];
u8 Gear=0,Gear_flag;
int count,temp;//用于判断拨弹电机是否卡弹是否需要反转变量
s16 Rammer_Moto_Measured_Speed,Rammer_Moto_Speed,Set_Rammer_Moto_Speed;//拨弹电机速度
s16 Friction_wheel_set_speed;//摩擦轮速度
s16 Rammer_moto_position;//拨弹电机位置
s16 Rammer_Moto_Speed_out;//拨弹电机
//////////////////////////////////////////////////////////
extern RC_Ctl_t RC_Ctl;                          //遥控器变量
extern volatile unsigned char sbus_rx_buffer[25];
/////////////////////////////////////////////////////////

s16 limit_pitch_down,limit_pitch_up,limit_yaw_left,limit_yaw_right;//云台限位变量
extern float Pitch,Roll,Yaw;//陀螺仪角度值用于控制云台
u8 falg;//用于消除陀螺仪飘移对射击造成影响
//视觉坐标值
extern unsigned char X_point; 
extern unsigned char	Y_point;
s16 angle_pitch_OFFSET,angle_yaw_OFFSET;
////////////////////////////////////////
//宏定义
#define ch1_offset 1.610522   //通道1采集零点电压值
#define ch2_offset 1.662085   //通道2采集零点电压值
#define ch3_offset 1.625830   //通道3采集零点电压值
#define ch4_offset 1.671753   //通道4采集零点电压值

#define voltage  25          //电压值
#define voltage_to_current_conversion_ratio 25 //采集电压值转换电流比例 asc758 40MV/A
#define Velocity_amplification_factor       8  //3510速度放大系数
#define Gimban_Running_speed               20  //键鼠控制云台运转速度比例
#define Power_limiting_factor             500 //功率控制系数 和功率成反比    理论计算是1000为80W   可以做到绝对不超功率，可以适当减小释放更多功率
void control_task(void *pdata)
{
			while(1)
			{       
				       
				       Rammer_moto_run_speed();//拨弹电机速度控制
							if(RC_Ctl.rc.s2==2||jump_mouse_r==1)//开摩擦轮并打开红外
								{
									 LASER_on();
									 Friction_wheel_control(Friction_wheel_set_speed,1);//发射子弹
									 if(RC_Ctl.mouse.press_l==1||RC_Ctl.rc.s1==1)//射击
									 Rammer_Moto_Speed=Set_Rammer_Moto_Speed;
								   else 
									 Rammer_Moto_Speed=0;		 
								}  
							else if(RC_Ctl.rc.s2!=2||jump_mouse_r==0)
								{
										LASER_off();
									  Rammer_Moto_Speed=0;
										LED_GREEN_OFF();
									  Friction_wheel_control(Friction_wheel_set_speed,0);
								}
								if(temp==20)
								{
									 temp=0;
								 if(Rammer_Moto_Measured_Speed==0)
									 count++;
								}
								temp++;
								delay_ms(10);
		}
}

//功能:摩擦轮控制
//输入：速度，模式
//输出：无
void Friction_wheel_control (s16 speed,u8 mode)//发射子弹
{
	if(mode)
	{
		PWM1=speed;
		PWM2=speed;
	}
	else
	{
		PWM1=1000;
		PWM2=1000;
	}

}
	
//功能：获得实际电流值
//输入：ADC通道
//输出：电流值
float AD_Power(uint8_t ADC_Channel)
{
	float real_current[5];
	switch (ADC_Channel)
	{
		case 0:
			real_current[1]=abs(((float)current_out[1]/4096*3.3)-ch1_offset)*voltage_to_current_conversion_ratio;
		  if(real_current[1]<0.08)
			real_current[1]=0;
		return real_current[1];
		case 1:
			real_current[2]=abs(((float)current_out[2]/4096*3.3)-ch2_offset)*voltage_to_current_conversion_ratio;
			if(real_current[2]<0.08)
			real_current[2]=0;
		return real_current[2];
		case 2:
			real_current[3]=abs(((float)current_out[3]/4096*3.3)-ch3_offset)*voltage_to_current_conversion_ratio;
			if(real_current[3]<0.08)
			real_current[3]=0;
		return real_current[3];
		case 3:
			real_current[4]=abs(((float)current_out[4]/4096*3.3)-ch4_offset)*voltage_to_current_conversion_ratio;
			if(real_current[4]<0.08)
			real_current[4]=0;
		return real_current[4];		

	}
}
 ////////////////////////////////////////////////////////////////////////////////////////////
//功能:3510速度和功率控制保证不超功率
//输入：无
//输出：无
void RM3510_control()
{
	 static u8 k;
	 static s16 A,B;
	 static float out_pwm[4],AD_in[4],speed_in_new[4],speed_in_last[4],speed_in[4];
	float ch2;
	static float mouse_x;
   mouse_x=+RC_Ctl.mouse.x*20;
	 ch2=(RC_Ctl.rc.ch2-1024+mouse_x);
	 s16 a,d,b,set_1,set_2,set_3,set_4;
	 keyboard_scan(&A,&B);
	 b=RC_Ctl.rc.ch1-1024+B;//前后
   a=(RC_Ctl.rc.ch0-1024-A);//左右平移
   if(a>450)
    a=450;
   else if(a<-450)
		a=-450;
	 if(!key_jump_q)
//		 if(RC_Ctl.mouse.x==0&&RC_Ctl.rc.ch2==1024&&a==0&&b==0)
//		 {
//			falg=1;
//			d=0;
//		 }
//     else		 
	   d=PID_Calculate(&Chassis_Position_Control,Gimbal_DATA.centric_angle[4],4096);//底盘跟随
   else if(key_jump_q)
	 d=ch2 ;
//////////////////////////////////////////////////////////////////////////////////////////////
//用于限制加速时转弯的最大叠加量，便于快速转弯
	 if(a>0)
	 {
		 
		if(abs(d)<a) 
	   a=a-abs(d);
		else if(abs(d)>a)
		 a=abs(d)-a;
	 }
	 else if(a<0)
	 {
	
		if(abs(d)<abs(a))
	   a=a+abs(d); 
   else if(abs(d)>abs(a))
		  a=abs(d)+a; 
   }
	 	 if(b>0)
	 {
		if(abs(d)<b) 
	   b=b-abs(d);
		else if(abs(d)>b)
		 b=abs(d)-b;
	 }
	 else if(b<0)
	 {
		if(abs(d)<abs(b))
	   b=b+abs(d); 
   else if(abs(d)>abs(b))
		  b=abs(d)+b; 
   }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	 set_1=d+a+b;
	 set_2=d-a+b;
	 set_3=d-a-b;
	 set_4=d+a-b; 
  if(key_shift)//用于键鼠控制时刹车和狙击
	{
	 speed_in[0]=0;
	 speed_in[1]=0;
	 speed_in[2]=0;
	 speed_in[3]=0;
	}
 else	
 {
   speed_in[0]=set_1;
	 speed_in[1]=set_2;
	 speed_in[2]=set_3;
	 speed_in[3]=set_4;
 }

	pwm_set_1=PID_Calculate(&RM3510_Speed_Control_1, RM3510_DATA.speed[1],speed_in[0]*Velocity_amplification_factor);
	pwm_set_2=PID_Calculate(&RM3510_Speed_Control_2, RM3510_DATA.speed[2],speed_in[1]*Velocity_amplification_factor);
	pwm_set_3=PID_Calculate(&RM3510_Speed_Control_3, RM3510_DATA.speed[3],speed_in[2]*Velocity_amplification_factor);
	pwm_set_4=PID_Calculate(&RM3510_Speed_Control_4, RM3510_DATA.speed[4],speed_in[3]*Velocity_amplification_factor);


	
				  AD_in[0]=pwm_set_1;
				  AD_in[1]=pwm_set_2;
				  AD_in[2]=pwm_set_3;
				  AD_in[3]=pwm_set_4;
			
			
				if(out_pwm[0]>0)
					Moto_Drive.current_dir[0]=1;
				else
					Moto_Drive.current_dir[0]=-1;
				if(out_pwm[1]>0)
					Moto_Drive.current_dir[1]=1;
				else
					Moto_Drive.current_dir[1]=-1;
				if(out_pwm[2]>0)
					Moto_Drive.current_dir[2]=1;
				else
					Moto_Drive.current_dir[2]=-1;
				if(out_pwm[3]>0)
					Moto_Drive.current_dir[3]=1;
				else
					Moto_Drive.current_dir[3]=-1;
			  Moto_Drive.power[0]=AD_Power(0)*voltage*Power_limiting_factor*Moto_Drive.current_dir[0];
				Moto_Drive.power[1]=AD_Power(1)*voltage*Power_limiting_factor*Moto_Drive.current_dir[1];
				Moto_Drive.power[2]=AD_Power(2)*voltage*Power_limiting_factor*Moto_Drive.current_dir[2];
				Moto_Drive.power[3]=AD_Power(3)*voltage*Power_limiting_factor*Moto_Drive.current_dir[3];
   out_pwm[0]=PID_Calculate(&PID_Power_Struct_1,Moto_Drive.power[0],(float)AD_in[0]);
	 out_pwm[1]=PID_Calculate(&PID_Power_Struct_2,Moto_Drive.power[1],(float)AD_in[1]);
	 out_pwm[2]=PID_Calculate(&PID_Power_Struct_3,Moto_Drive.power[2],(float)AD_in[2]);
	 out_pwm[3]=PID_Calculate(&PID_Power_Struct_4,Moto_Drive.power[3],(float)AD_in[3]);

	
	
	RM3510_Cmd(CAN1,
	          RM3510_current_confine(out_pwm[0]),
						RM3510_current_confine(out_pwm[1]),
						RM3510_current_confine(out_pwm[2]),
						RM3510_current_confine(out_pwm[3]));
	
	LED_RED_OFF();
	
}
///////////////////////////////////////////////////////////////////////////////////////////
//功能:3510 限定current 控制 
//输入:功率环输出电流值
//输出:限制后输出
s16 RM3510_current_confine(s16 Pwm)
{
		if(Pwm>32767)
	{
		Pwm = 32767;
	}
	else if(Pwm < -32768)
	{
		Pwm = -32768;
	}	
	
 }
/////////////////////////////////////////////////////////////////////////////////////////////
 /****************************
功能：      机械角矫正
输入： 当前角度angle，初始化角度first_angle
输出：以4096为中心的数
*****************************/
 
s16 transe_M_angle(s16 angle,u16 first_angle)
{
	s16 i;
	i=4096-first_angle;
	angle=angle+i;
	if(angle<0)
	angle=angle+8192;
	if(angle>8192)
	angle=angle-8192;
	return angle;	
}
//机械角度校准
void angle_CALIBRATE(void)
{

	u16 i;
	float temp_angle_pitch=0,temp_angle_yaw=0;
	

	for(i=0; i<1000; i++)
	{
		temp_angle_pitch=temp_angle_pitch+Gimbal_DATA.centric_angle[5];
    temp_angle_yaw=temp_angle_pitch+Gimbal_DATA.centric_angle[4];
	}

	temp_angle_pitch=temp_angle_pitch/1000.00;
	temp_angle_yaw=temp_angle_yaw/1000.00;


	angle_pitch_OFFSET=temp_angle_pitch;
	angle_yaw_OFFSET=temp_angle_yaw;

}

 /****************************
功能：      云台运转
输入： 无
输出：无
*****************************/
 void Gimban_ctrl(void)
 {
	 float ch2,ch3;
	 static float expect_Yaw_angle,expect_Pitch_angle,PID_Pitch_P_out,PID_Pitch_V_out,PID_YAW_P_out,PID_YAW_V_out,mouse_x,mouse_y;
     mouse_x=+RC_Ctl.mouse.x*Gimban_Running_speed;
     mouse_y=-RC_Ctl.mouse.y*Gimban_Running_speed;
	  ch2=(RC_Ctl.rc.ch2-1024+mouse_x);
	  ch3=(RC_Ctl.rc.ch3-1024+mouse_y);
	 //机械角做反馈
	 if(Gimbal_DATA.centric_angle[4]>=limit_yaw_left||Gimbal_DATA.centric_angle[4]<=limit_yaw_right)
	 {
	  if(Gimbal_DATA.centric_angle[4]>=limit_yaw_left&&ch2>0)
		 expect_Yaw_angle+=ch2/1000;
	 else if(Gimbal_DATA.centric_angle[4]<=limit_yaw_right&&ch2<0)
		 expect_Yaw_angle+=ch2/1000;
    }
	 else
    expect_Yaw_angle+=ch2/1000;
	 if((RC_Ctl.rc.s1==2&&key_jump_e==1)||(RC_Ctl.rc.s1==3&&key_jump_e==0)) //补给站模式      
	 PID_Pitch_P_out=PID_Calculate(&PID_Pitch_P_Struct, MPU6050_Angle.Pitch,-15);//补给站模式pitch轴放水平往下15度
	 else    
	 {
		 if(Gimbal_DATA.centric_angle[5]>limit_pitch_up||Gimbal_DATA.centric_angle[5]<limit_pitch_down)
		 {
			 if(Gimbal_DATA.centric_angle[5]>=limit_pitch_up&&ch3<0)
				 expect_Pitch_angle+=ch3/1000;
			 else if(Gimbal_DATA.centric_angle[5]<=limit_pitch_down&&ch3>0)
				 expect_Pitch_angle+=ch3/1000;
		 }
			 else
				expect_Pitch_angle+=ch3/1000;
		  
		 PID_Pitch_P_out=PID_Calculate(&PID_Pitch_P_Struct, MPU6050_Angle.Pitch,expect_Pitch_angle);
	 }

	 if(!key_jump_q)
	 {
//	  if(falg)
//	   {
//			 falg=0;
//	     MPU6050_Angle.Yaw=0;
//			 expect_Yaw_angle=0;
//			 PID_YAW_P_out=-PID_Calculate(&PID_Yaw_P_Struct,MPU6050_Angle.Yaw,expect_Yaw_angle);
//			 //PID_YAW_P_out=PID_Calculate(&PID_Yaw_P_Struct_chassis,(float)Gimbal_DATA.centric_angle[4]*360/8192-180,0);
//    	}
//		 else
			PID_YAW_P_out=-PID_Calculate(&PID_Yaw_P_Struct,MPU6050_Angle.Yaw,expect_Yaw_angle);
		  
    }
	 else if(key_jump_q)
	 {
		MPU6050_Angle.Yaw=0;
		 expect_Yaw_angle=0;
	 PID_YAW_P_out=PID_Calculate(&PID_Yaw_P_Struct_chassis,(float)Gimbal_DATA.centric_angle[4]*360/8192-180,0);
	 }
	 PID_Pitch_V_out=PID_Calculate(&PID_Pitch_V_Struct,MPU6050_Real_Data.Gyro_Y,PID_Pitch_P_out);
	 PID_YAW_V_out=PID_Calculate(&PID_Yaw_V_Struct,MPU6050_Real_Data.Gyro_Z,PID_YAW_P_out);
   RM_Driver_Cmd(CAN1,PID_YAW_V_out,PID_Pitch_V_out);

 }

/*************************
      初始化PID参数
*************************/
void PID_Init_chassis(PID_Struct *PID)
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
  PID->Ouput_deltaUk_Max   = 5000;		    //限制输出量最大值
  PID->Ouput_deltaUk_Min   = -5000;		    //限制输出量最小值
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
  PID->PID_Integral_Max    = 1000.0;		     //限制积分项最大值
  PID->PID_Integral_Min    = -1000.0;			//限制积分项最小值
}
void PID_GimBal_Init_P(PID_Struct *PID)
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
  PID->PID_Integral_Max    = 10.0;		     //限制积分项最大值
  PID->PID_Integral_Min    = -10.0;			//限制积分项最小值
}
void PID_GimBal_Init_yaw_chaiss(PID_Struct *PID)
{
  PID->expectation       = 0.0;            //遥控给的期望值
  PID->Err_k			       = 0.0;            //当前误差值e(k)
  PID->Err_k_1		       = 0.0;           //k-1时刻误差值e(k-1)
  PID->Err_k_2		       = 0.0;           //k-2时刻误差值e(k-2)
  PID->SumErr              = 0.0;			//误差和
  PID->Kp				   = 4;           //比例系数，通过串口在线调PID参数再写入Flash
  PID->Ti				   = 0.0;           //积分系数，通过串口在线调PID参数再写入Flash
  PID->Td				   = 0.0;           //微分系数，通过串口在线调PID参数再写入Flash
  PID->Ouput_deltaUk       = 0.0;		    //PID计算后的输出量U(k) - U(k-1)
  PID->Ouput_deltaUk_Max   = 5000;		    //限制输出量最大值
  PID->Ouput_deltaUk_Min   = -5000;		    //限制输出量最小值
  PID->PID_Integral_Max    = 1000.0;		     //限制积分项最大值
  PID->PID_Integral_Min    = -1000.0;			//限制积分项最小值
}
void PID_init_RM3510(PID_Struct *PID)
{
	PID->expectation       = 0.0;            //遥控给的期望值
  PID->Err_k			       = 0.0;            //当前误差值e(k)
  PID->Err_k_1		       = 0.0;            //k-1时刻误差值e(k-1)
  PID->Err_k_2		       = 0.0;            //k-2时刻误差值e(k-2)
  PID->SumErr              = 0.0;		
  PID->Kp				   =0;           //P
  PID->Ti				   = 0;           //I
  PID->Td				   = 0;           //D
  PID->Ouput_deltaUk       = 0.0;		    //PID计算后的输出量U(k) - U(k-1)
  PID->Ouput_deltaUk_Max   = 20000;		    //限制输出量最大值
  PID->Ouput_deltaUk_Min   = -20000;		    //限制输出量最小值
  PID->PID_Integral_Max    = 10000.0;		     //限制积分项最大值
  PID->PID_Integral_Min    = -10000.0;			//限制积分项最小值
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
void PID_init_power(PID_Struct *PID)
{
	PID->expectation       = 0.0;            //遥控给的期望值
  PID->Err_k			       = 0.0;            //当前误差值e(k)
  PID->Err_k_1		       = 0.0;            //k-1时刻误差值e(k-1)
  PID->Err_k_2		       = 0.0;            //k-2时刻误差值e(k-2)
  PID->SumErr              = 0.0;		
  PID->Kp				   =0.2;           //P
  PID->Ti				   =0.01;           //I
  PID->Td				   = 0;           //D
  PID->Ouput_deltaUk       = 0.0;		    //PID计算后的输出量U(k) - U(k-1)
  PID->Ouput_deltaUk_Max   = 20000;		    //限制输出量最大值
  PID->Ouput_deltaUk_Min   = -20000;		    //限制输出量最小值
  PID->PID_Integral_Max    = 5000;		     //限制积分项最大值
  PID->PID_Integral_Min    = -5000;			//限制积分项最小值
}
void pid_init_struct(void)
{
	PID_GimBal_Init(&PID_Yaw_V_Struct);
  PID_GimBal_Init(&PID_Pitch_V_Struct);	
  PID_GimBal_Init_P(&PID_Yaw_P_Struct);
  PID_GimBal_Init_P(&PID_Pitch_P_Struct);
	PID_Rammer_moto_Init(&PID_Rammer_moto);
	PID_init_RM3510(&RM3510_Speed_Control_1);
	PID_init_RM3510(&RM3510_Speed_Control_2);
	PID_init_RM3510(&RM3510_Speed_Control_3);
	PID_init_RM3510(&RM3510_Speed_Control_4);
	PID_Init_chassis(&Chassis_Position_Control);
	PID_init_power(&PID_Power_Struct_1);
	PID_init_power(&PID_Power_Struct_2);
	PID_init_power(&PID_Power_Struct_3);
	PID_init_power(&PID_Power_Struct_4);
  PID_GimBal_Init_yaw_chaiss(&PID_Yaw_P_Struct_chassis);
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
  Value_Derivative  =  PID->Td * (PID->Err_k - PID->Err_k_1);

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
/***************************************************************
PID计算-位置式-比例变化
***************************************************************/
float PID_Calculate_pitch(PID_Struct* PID, float measured, float expect )
{
  float Value_Proportion;  
  float Value_Integral;		
  float Value_Derivative;	
	float kp;

  PID->expectation =  expect;
  PID->Err_k = (PID->expectation - measured)*10;
  PID->SumErr+= PID->Err_k;

  //P I D
	if(measured>0)
	kp=PID->Kp*0.6;
	else
	kp=PID->Kp;
  Value_Proportion    = kp* PID->Err_k;
  Value_Integral      =  PID->SumErr * PID->Ti;	
  Value_Derivative  =  PID->Td * measured;

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