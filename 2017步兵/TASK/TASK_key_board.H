#ifndef __TASK_key_board_H__
#define __TASK_key_board_H__
#include <stm32f4xx.h>
#include "usart2.h"
#include "delay.h"
#define KEY_B   0x8000
#define KEY_V		0x4000
#define KEY_C		0x2000
#define KEY_X		0x1000
#define KEY_Z		0x0800
#define KEY_G		0x0400
#define KEY_F		0x0200
#define KEY_R		0x0100
#define KEY_E		0x0080
#define KEY_Q		0x0040
#define KEY_CTRL	0x0020
#define KEY_SHIFT	0x0010
#define KEY_D		0x0008
#define KEY_A		0x0004
#define KEY_S		0x0002
#define KEY_W		0x0001

#define moto_max  660         //¼üÊó¿ØÖÆÊä³ömax
#define moto_min -660         //¼üÊó¿ØÖÆÊä³ömin
extern u8 key_a,key_b,key_d,key_s,key_w,key_z,key_x,key_c,key_v,key_g,key_f,key_r,key_e,key_q,key_ctrl,key_shift;
extern u8 key_jump_a,key_jump_d,key_jump_s,key_jump_w,key_jump_z,key_jump_x,key_jump_c,key_jump_v,key_jump_g,
key_jump_f,key_jump_r,key_jump_e,key_jump_q,key_jump_ctrl,key_jump_shift,key_jump_b;
extern u8 jump_mouse_r;
u8 keyboard_data(u16 key);
void keyboard(void);
void keyboard_scan(s16 *A,s16 *B);
u8 keyboard_jump_1(u8 key);
u8 keyboard_jump_2(u8 key);
  u8 keyboard_jump_mouse_r(u8 key);
void key_board_task(void *pdata);
#endif