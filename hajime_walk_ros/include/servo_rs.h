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

#define	extern
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

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
extern	void	servo_rs_init( void );

extern	void 	write_servo_rs_all( unsigned char, unsigned short *, short );
extern	void 	read_servo_rs_all( unsigned char, unsigned short *, short );
extern	void	servo_rs_fun( void );
extern 	void 	servopos_to_jointdeg( void );
extern 	void 	servotemp_to_temp( void );
extern	void 	servocurrent_to_current( void );	//2010.5.8

#undef	extern

#endif /* _SERVO_RS_H_ */
