/*----------------------------------------------------------*/
/*	servo motor RS405CB driver								*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	servo_rs.c								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2004.8.29								*/
/*----------------------------------------------------------*/
#ifndef _SERVO_RS_H_
#define _SERVO_RS_H_

#ifdef _SERVO_RS_C_
#define	extern
#endif /* _SERVO_RS_C_ */
#include	"var.h"



typedef struct st_xp_servo_rs
{
	unsigned short	goal_time_slow[SERV_NUM];
	unsigned short  free_mode[SERV_NUM];
	unsigned short  normal_mode[SERV_NUM];
	unsigned short  control_mode[SERV_NUM];
	unsigned short  run_or_control[SERV_NUM];
	unsigned short  trajectory_normal[SERV_NUM];
	unsigned short  trajectory_even[SERV_NUM];
	unsigned short  gain_preset[SERV_NUM];
	unsigned short	deadband_width[SERV_NUM];
	unsigned short  control_kp0[SERV_NUM];
	unsigned short  control_kd0[SERV_NUM];
	unsigned short  control_ki0[SERV_NUM];
	unsigned short  control_static_friction0[SERV_NUM];
	unsigned short  control_dynamic_friction0[SERV_NUM];
	unsigned short  control_kp1[SERV_NUM];
	unsigned short  control_kd1[SERV_NUM];
	unsigned short  control_ki1[SERV_NUM];
	unsigned short  control_static_friction1[SERV_NUM];
	unsigned short  control_dynamic_friction1[SERV_NUM];
	unsigned short  control_kp2[SERV_NUM];
	unsigned short  control_kd2[SERV_NUM];
	unsigned short  control_ki2[SERV_NUM];
	unsigned short  control_static_friction2[SERV_NUM];
	unsigned short  control_dynamic_friction2[SERV_NUM];
}	tp_xp_servo_rs;

typedef struct st_xv_servo_rs
{
	unsigned short	present_temp[SERV_NUM];
	unsigned short	present_position[SERV_NUM];
	unsigned short	goal_position[SERV_NUM];
	unsigned short	present_current[SERV_NUM];
}	tp_xv_servo_rs;


/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
extern 	short			servo_rs_id_scif2[SERVO_RS_ID_NUM]		;
extern 	short			servo_rs_id_scif3[SERVO_RS_ID_NUM]		;


extern	unsigned char 	rs405_bpTxBuffer_scif2[2048];
extern	unsigned char 	rs405_bpTxBuffer_scif3[2048];
extern	unsigned char 	rs405_bpParameter[96];
extern	unsigned char 	rs405_bpRxBuffer_scif2[64];
extern	unsigned char 	rs405_bpRxBuffer_scif3[64];
extern	short			rs405_bCount;
extern	short			servo_rs_id;
//extern	short			servo_rs_address;
extern	short			servo_rs_len;
extern	short			servo_rs_sdata;
extern	short			servo_rs_rdata;
extern	short			flag_write_servo_rs_scif2;
extern	short			flag_write_servo_rs_scif3;
extern	short			flag_read_servo_rs_scif2;
extern	short			flag_read_servo_rs_scif3;
extern	short			flag_write_servo_rs_all;
extern	short			flag_read_servo_rs_all;
extern	short			flag_servo_rs_test;
extern 	short			flag_servo_on;
extern 	short			flag_servo_off;
extern 	short			flag_servo_output_wait;
extern	short			flag_read_servo_position;
extern 	float			servo_on_totaltime;
extern 	tp_xp_servo_rs	xp_servo_rs;
extern 	tp_xp_servo_rs	xp_servo_rs_soft;
extern	tp_xp_servo_rs	xp_servo_rs_motion;
extern	tp_xv_servo_rs	xv_servo_rs;
extern 	short			flag_ukemi;

#ifdef _SERVO_RS_C_
#undef	extern
#endif /* _SERVO_RS_C_ */

#endif /* _SERVO_RS_H_ */
