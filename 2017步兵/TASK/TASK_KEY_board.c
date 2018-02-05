#include "TASK_key_board.H"
extern RC_Ctl_t RC_Ctl;
u8 key_a,key_b,key_d,key_s,key_w,key_z,key_x,key_c,key_v,key_g,key_f,key_r,key_e,key_q,key_ctrl,key_shift;
u8 key_jump_a,key_jump_b,key_jump_d,key_jump_s,key_jump_w,key_jump_z,key_jump_x,key_jump_c,key_jump_v,key_jump_g,
key_jump_f,key_jump_r,key_jump_e,key_jump_q,key_jump_ctrl,key_jump_shift;
u8 jump_mouse_r;
//����scan����
void key_board_task(void *pdata)
{
	while(1)
	{
		keyboard();
		delay_ms(5);
	}
}



/****************************
���ܣ�    ���̲ٿ�
���룺 A��ͨ��1  B��ͨ��2
�������
*****************************/
 void keyboard_scan(s16 *A,s16 *B)
 {
	 #define step 5
	 static u16 num;
	 	 if(RC_Ctl.key.v& KEY_D)  // key: d
	 {
		 *A=*A-step;
	 }
	 	else if(RC_Ctl.key.v& KEY_A) //key: a
	 {
		 *A=*A+step;
	 }

	 	else if(RC_Ctl.key.v & KEY_W)  // key: w
	 {
		 *B=*B+step;
	 }
	 	else if(RC_Ctl.key.v& KEY_S) //key: s
	 {
		 *B=*B-step;
	 }
	else
		{
            num++;
			if(num>2)
			{
				num=0;
				*A=0;
				*B=0;
			}
		}
     if(*A>moto_max)
		 {
			 *A=moto_max;
		 }
		 else if(*A<moto_min)
		 {
			 *A=moto_min;
		 }
		  if(*B>moto_max)
		 {
			 *B=moto_max;
		 }
		 else if(*B<moto_min)
		 {
			 *B=moto_min;
		 }
 }
 //����ɨ��
 void keyboard(void)
 {
	 key_a=keyboard_data(KEY_A);
	 key_d=keyboard_data(KEY_D);
	 key_w=keyboard_data(KEY_W);
	 key_s=keyboard_data(KEY_S);
	 key_z=keyboard_data(KEY_Z);
	 key_x=keyboard_data(KEY_X);
	 key_c=keyboard_data(KEY_C);
	 key_v=keyboard_data(KEY_V);
	 key_g=keyboard_data(KEY_G);
	 key_f=keyboard_data(KEY_F);
	 key_r=keyboard_data(KEY_R);
	 key_e=keyboard_data(KEY_E);
	 key_q=keyboard_data(KEY_Q);
	 key_b=keyboard_data(KEY_B);
	 key_ctrl=keyboard_data(KEY_CTRL);
	 key_shift=keyboard_data(KEY_SHIFT);
	 key_jump_q=keyboard_jump_1(key_q);
	 key_jump_e=keyboard_jump_2(key_e);
	 
	jump_mouse_r=keyboard_jump_mouse_r(RC_Ctl.mouse.press_r);
 }
//����ɨ���Ӻ���   �㷨��ʵ�ֹ��ܵ����д�����
//(�м���key��Ҫ�����������ĺ�����ʹ�������ӷ��,����ʵ�ֵ���һ������������м��)ԭ������static�����ܲٵ������ֲ���
//////////////////////////////////
 u8 keyboard_data(u16 key)
 {
	 if(RC_Ctl.key.v& key)
		 return 1;
	 else
		 return 0;
 }
 u8 keyboard_jump_1(u8 key)
 {
	 static u8 key_falg,key_value,last_key_value;
	 static u32 temp;

		last_key_value=key_value;
		key_value=key;
		if(last_key_value==0&&key_value==1) 
		{
			temp++;
		if(temp%2)
		 key_falg=1;
	 else
		 key_falg=0;
		}
	 return key_falg;
 }
 
u8 keyboard_jump_2(u8 key)
 {
	 static u8 key_falg,key_value,last_key_value;
	 static u32 temp;

		last_key_value=key_value;
		key_value=key;
		if(last_key_value==0&&key_value==1) 
		{
			temp++;
		if(temp%2)
		 key_falg=1;
	 else
		 key_falg=0;
		}
	 return key_falg;
 }
  u8 keyboard_jump_mouse_r(u8 key)
 {
	 static u8 key_falg,key_value,last_key_value;
	 static u32 temp;

		last_key_value=key_value;
		key_value=key;
		if(last_key_value==0&&key_value==1) 
		{
			temp++;
		if(temp%2)
		 key_falg=1;
	 else
		 key_falg=0;
		}
	 return key_falg;
 }
 /////////////////////////////////////////////////////////////////////////////