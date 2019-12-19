/*----------------------------------------------------------*/
/*	high speed walk sequence								*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	sq_walk.h								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#ifndef 	_SQ_WALK_H_
#define 	_SQ_WALK_H_

#include	"func.h"

#ifdef 		_SQ_WALK_C_
#define 	extern 
#endif 		/* _SQ_WALK_C_ */


/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/
enum {							//											�T�C�h�X�e�b�v�������Ƃ��̎���
	SQ_WALK_INIT	,			//  1)������								t: 0
	SQ_WALK_MV_L	,			//  2)���ɏd�S���ڂ�						t: 0 - period/4*k2 (0�Ƀ��Z�b�g)
	SQ_WALK_UP_R0	,			//  3)����ڂ̉E�����グ��܂ł̑҂��̎���	t: period/4*k2
	SQ_WALK_UP_R	,			//  4)�E�����グ��܂ł̓���				t: 0 - period/2*(1-duty)
	SQ_WALK_UP_R_1	,			//  5)�E�����グ�铮��						t: period/2*(1-duty)
	SQ_WALK_DW_R	,			//  7)�E���������铮��						t: period/2*(1-duty/2) - period/2
	SQ_WALK_MV_R2	,			//  8)�E�ɏd�S���ڂ�						t: 0

	SQ_WALK_MV_R	,			//    �E�ɏd�S���ڂ�
	SQ_WALK_UP_L0	,			//    ����ڂ̍������グ��܂ł̑҂��̎���
	SQ_WALK_UP_L	,			//  9)�������グ��܂ł̓���				t: 0 - period/2*(1-duty)
	SQ_WALK_UP_L_1	,			// 10)�������グ�铮��						t: period/2*(1-duty)
	SQ_WALK_DW_L	,			// 12)�����������铮��						t: period/2*(1-duty/2) - period/2
	SQ_WALK_MV_L2	,			// 13)���ɏd�S���ڂ� �� 4)�@�Ō�̈�� �� 14)

	SQ_WALK_UP_R2	,			// 14)�E�����グ�铮��i�Ō�̈���j
	SQ_WALK_UP_R2_1	,			// 15)�E�����グ�铮��i�Ō�̈���j
	SQ_WALK_UP_R2_2	,			// 16)�E�����グ�铮��i�Ō�̈���j
	SQ_WALK_DW_R2	,			// 17)�E���������铮��i�Ō�̈���j

	SQ_WALK_UP_L2	,			//    �������グ�铮��i�Ō�̈���j
	SQ_WALK_UP_L2_1	,			//    �������グ�铮��i�Ō�̈���j
	SQ_WALK_UP_L2_2	,			//    �������グ�铮��i�Ō�̈���j
	SQ_WALK_DW_L2	,			//    �����������铮��i�Ō�̈���j

	SQ_WALK_READY	,			//    ���f�B�̏�Ԃɖ߂�
	SQ_WALK_END		,			//    �I������
};


typedef struct st_xp_mv_walk
{
	long	num;				/* number of steps for test [-]			*/
	float	h_cog;				/* height of center of gravity [mm]		*/
	float	time;				/* step time [sec] 						*/
//	float	next_walk;			/* timing for next step [1] 			*/
	float	x_fwd_swg;			/* forward stride of swing leg [mm]		*/
	float	x_fwd_spt;			/* forward stride of support leg [mm]	*/
	float	x_bwd_swg;			/* backward stride of swing leg [mm]	*/
	float	x_bwd_spt;			/* backward stride of support leg [mm]	*/
	float	y_swg;				/* sidestep stride of swing leg [mm]	*/
	float	y_spt;				/* sidestep stride of support leg [mm]	*/
	float	theta;				/* hip yaw angle of each leg for turn [deg]	*/
	float	z;					/* hight of foot up [mm] 				*/
	float	y_balance;			/* right and left balance point [mm] 	*/	// �ҊԂ̋���/2
	float	hip_roll;			/* hip roll when leg up [deg] 			*/
	float	x_fwd_pitch;		/* pitch for forward [deg] 				*/
	float	x_bwd_pitch;		/* pitch for backward [deg] 			*/
	float	x_fwd_acc_pitch;	/* pitch for forward under acceleration [deg]	*/
	float	x_bwd_acc_pitch;	/* pitch for backward under acceleration [deg]	*/
	float	arm_sh_pitch;		/* shoulder pitch for walk [deg] 		*/
    float   arm_el_pitch;		/* elbow pitch for walk [dag]			*/
	float	start_time_k1;		/* first step time to balance [1] 		*/
	float	start_zmp_k1;		/* first step zmp y to balance [1] 		*/
	float	start_time_k2;		/* first step check time to balance [1]	*/
	float	foot_cntl_p;		/* pitch of foot control [deg] 			*/
	float	foot_cntl_r;		/* roll of foot control [deg] 			*/
	float	sidestep_time_k;	/* sidestep time offset [0] 			*/
	float	sidestep_roll;		/* sidestep roll [deg] 					*/
	float	y_wide;				/* y position of walk [mm] 				*/
	float	time_dutyfactor;	/* duty factor of walk [1] 				*/
    float   accurate_x_percent_dlim;
    float   accurate_y_percent_dlim;
    float   accurate_th_percent_dlim;
    float   accurate_step_z;
    float   accurate_step_time;
}	tp_xp_mv_walk;


typedef struct st_xv_mv_walk
{
	long	num;				/* ����							[-]		*/
	float	time;				/* ���s�̎���/2					[sec]	*/
	float	time_old;			/* ���s�������Ō�ɕύX��������	[sec]	*/
	float	x_swg;				/* �V�r�̑O��̕���				[mm]	*/
	float	x_spt;				/* �x���r�̑O��̕���			[mm]	*/
	float	y_swg;				/* �V�r�̍��E�̕���				[mm]	*/
	float	y_spt;				/* �x���r�̍��E�̕���			[mm]	*/
	float	theta;				/* �^�[�����̌҂̃��[���̊p�x	[deg]	*/
	float	z;					/* �����グ�鍂��				[mm]	*/
	float	pitch;				/* �O�ɂ����ފp�x				[deg]	*/
	float	arm_sh_pitch;		/* ���̃s�b�`�p�x				[deg]	*/
    float   arm_el_pitch;		/* �I�̃s�b�`�p�x				[deg]	*/
	float	zmp;				/* ZMP�K�͂ő����ӂ镝?			[mm]	*/
	float	x_percent;			/* x�����̕����̍ŏI�I�Ȋ���(0-1)[-]	*/
	float	y_percent;			/* y�����̕����̍ŏI�I�Ȋ���(0-1)[-]	*/
	float	theta_percent;		/* �҃��[���p�̍ŏI�I�Ȋ���(0-1)[-]		*/
	float	sidestep_time_k_r;	/* �E���ړ��̎��Ԃ̊���(0-1)	[bit]	*/
	float	sidestep_time_k_l;	/* �����ړ��̎��Ԃ̊���(0-1)	[bit]	*/
	float	sidestep_roll;		/* ���������̃��[������]�p�x	[deg]	*/
	float	sidestep_roll_z;	/* ���������̌Ҏ��̍����̕ω�	[mm]	*/
	float	x_percent_dlim;		/* x�����̕����̌��݂̊���(0-1)	[-] 	*/
	float	y_percent_dlim;		/* y�����̕����̌��݂̊���(0-1)	[-] 	*/
	float	theta_percent_dlim;	/* �҃��[���p�̌��݂̊���(0-1)	[-] 	*/
	float	pitch_percent_dlim;	/* �s�b�`���̌��݂̊���(0-1)	[-]		*/
	float	time_dutyfactor;	/* �V�r�ł��鎞�Ԃ̔䗦(0-1)	[-]		*/
    float   accurate_step_x;    /* ����őO�i���鋗��           [mm]    */
    float   accurate_step_y;
    float   accurate_step_z;
    float   accurate_step_th;
    float   accurate_step_time;
}	tp_xv_mv_walk;


typedef struct st_flag_walk
{
	short	upleg;				/*	up right leg or left leg		*/
	short	upleg_last;			/*	the last up lag					*/
	short	y;					/*	walk sidestep					*/
	short	turn;				/*	turn right or left				*/
	short	y_on;				/*	walk sidestep now				*/
	short	turn_on;			/*	turn right or left now			*/
}	tp_flag_walk;


/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
extern 	tp_xp_mv_walk	xp_mv_walk 			;	// ���s�Ɋւ���萔
extern 	tp_xv_mv_walk	xv_mv_walk 			;	// ���s�Ɋւ���ϐ�
extern 	tp_flag_walk	flag_walk 			;	// ���s�̃t���O
extern 	short			mode_sq_walk	 	;	// ���s�̃��[�h�̔ԍ�
extern	tp_xp_dlim_wait	xp_dlim_wait_x		;	// x�����̕����̊����̒萔
extern	tp_xv_dlim_wait	xv_dlim_wait_x		;	// x�����̕����̊����̕ϐ�
extern	tp_xp_dlim_wait	xp_dlim_wait_y		;	// y�����̕����̊����̒萔
extern	tp_xv_dlim_wait	xv_dlim_wait_y		;	// y�����̕����̊����̕ϐ�
extern	tp_xp_dlim_wait	xp_dlim_wait_theta	;	// �҃��[���p�̊����̒萔
extern	tp_xv_dlim_wait	xv_dlim_wait_theta	;	// �҃��[���p�̊����̕ϐ�
extern	tp_xp_dlim_wait	xp_dlim_wait_pitch	;	// �s�b�`�����̌X���̒萔
extern	tp_xv_dlim_wait	xv_dlim_wait_pitch	;	// �s�b�`�����̌X���̕ϐ�
extern  char            accurate_one_step_mode;

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
extern int sq_walk( void );				// ���s�i1:�I���C0:���s���j20ms���ɌĂяo��
extern void sq_walk_init( void );		// ���s�̏�����

#ifdef 		_SQ_WALK_C_
#undef 		extern
#endif 		/* _SQ_WALK_C_ */

#endif 		/* _SQ_WALK_H_ */
