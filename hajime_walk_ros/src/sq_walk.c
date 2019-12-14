/*----------------------------------------------------------*/
/*	high speed walk sequence								*/
/*															*/
/*															*/
/*	file name	:	sq_walk.c								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.6.3								*/
/*----------------------------------------------------------*/
#define		_SQ_WALK_C_

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include	<stdio.h>
#include    <math.h>
#include	"var.h"
#include	"func.h"
#include	"sq_walk.h"
#include	"calc_mv.h"
#include	"sq_ready.h"
#include 	"servo_rs.h"
#include	"b3m.h"
#include 	"gyro.h"
#include 	"kine.h"
#include 	"mvtbl.h"
#include 	"joy.h"
#include 	"serv.h"
#include	"motion.h"

extern void sq_walk_fun( void );
short flag_md_walk_end;

/*--------------------------------------*/
/*	sq_walk								*/
/*--------------------------------------*/
/*** parameter setting for walk		***/
int sq_walk( void )
{
	float abs_y_percent;							// y_percent_dlim�������̊����̐�Βl

	/*** set dafault walk parameters	***/
	if( mode_sq_walk == SQ_WALK_INIT )				/*	initialize mode	*/
	{
		xv_mv_walk.z = xp_mv_walk.z;				/*	hight of foot z	*/

		/*	dlimit initialize	*/
		dlim_wait_fun_init( &xp_dlim_wait_x    , &xv_dlim_wait_x     );	// �@�䗦��0�ɂ���
		dlim_wait_fun_init( &xp_dlim_wait_y    , &xv_dlim_wait_y     );	// �@�䗦��0�ɂ���
		dlim_wait_fun_init( &xp_dlim_wait_theta, &xv_dlim_wait_theta );	// �@�䗦��0�ɂ���
		dlim_wait_fun_init( &xp_dlim_wait_pitch, &xv_dlim_wait_pitch );	// �@�䗦��0�ɂ���
		xv_dlim_wait_pitch.out = xp_mv_ready.pitch;

		/*	gyro senser integrator clear	*/
//		xv_gyro.gyro_roll	=
//		xv_gyro.gyro_pitch	=
//		xv_gyro.gyro_yaw	=	0.f;
	}

	/***	set parameters from command receive	***/
	if (is_walk_change){
		copy_joy_parameter();											/*	command receive	*/
		is_walk_change = 0;
	}

	/***	calculation dlimit of x, y, theta percent	***/
	xv_dlim_wait_x.in				=	xv_mv_walk.x_percent;
	xv_dlim_wait_y.in				=	xv_mv_walk.y_percent;
	xv_dlim_wait_theta.in			=	xv_mv_walk.theta_percent;
    
    if(accurate_one_step_mode == 1){
        
        if(xv_mv_walk.accurate_step_x > -EPS_DATA && xv_mv_walk.accurate_step_x < EPS_DATA)
            xv_mv_walk.x_percent_dlim     = 0.f;
        else 
            xv_mv_walk.x_percent_dlim     = xv_mv_walk.accurate_step_x  > 0 ? 1.f : -1.f;
        
        if(xv_mv_walk.accurate_step_y > -EPS_DATA && xv_mv_walk.accurate_step_y < EPS_DATA)
            xv_mv_walk.y_percent_dlim     = 0.f;
        else
            xv_mv_walk.y_percent_dlim     = xv_mv_walk.accurate_step_y  > 0 ? 1.f : -1.f;
        
        if(xv_mv_walk.accurate_step_th > -EPS_DATA && xv_mv_walk.accurate_step_th < EPS_DATA)
            xv_mv_walk.theta_percent_dlim = 0.f;
        else
            xv_mv_walk.theta_percent_dlim = xv_mv_walk.accurate_step_th > 0 ? 1.f : -1.f;
    }else{
        xv_mv_walk.x_percent_dlim =	dlim_wait_fun( &xp_dlim_wait_x, &xv_dlim_wait_x );	//�����̊���(0-1) ���X�ɕ����𑝂₷���߂̎d�g��
	    xv_mv_walk.y_percent_dlim =	dlim_wait_fun( &xp_dlim_wait_y, &xv_dlim_wait_y );
        xv_mv_walk.theta_percent_dlim = xv_mv_walk.theta_percent;
    }

    /*
    if( !flag_gyro.yaw_cntl ){
		xv_mv_walk.theta_percent_dlim	=	dlim_wait_fun( &xp_dlim_wait_theta, &xv_dlim_wait_theta );
	}
    */

	/***	calculation walk forward and backward parameters	***/
	if( xv_mv_walk.x_percent_dlim > EPS_DATA ) {										/*	walk forward	*/
        if(accurate_one_step_mode == 1){
            xv_mv_walk.x_swg        =   (xv_mv_walk.accurate_step_x / 2.f) * xp_mv_walk.accurate_x_percent_dlim;
            xv_mv_walk.x_spt        =   (-xv_mv_walk.accurate_step_x / 2.f) * xp_mv_walk.accurate_x_percent_dlim;
        }else{
            xv_mv_walk.x_swg		=	xp_mv_walk.x_fwd_swg * xv_mv_walk.x_percent_dlim;	// �V�r�̕���
		    xv_mv_walk.x_spt		=	xp_mv_walk.x_fwd_spt * xv_mv_walk.x_percent_dlim;	// �x���r�̕���
        }

		xv_mv_walk.pitch		=	xp_mv_walk.x_fwd_pitch * xv_mv_walk.x_percent_dlim;	// �O�ɂ����ފp�x
		xv_mv_walk.arm_sh_pitch	=	xp_mv_walk.arm_sh_pitch;							// ���̊p�x
		xv_mv_walk.arm_el_pitch =   xp_mv_walk.arm_el_pitch;                            // �I�̊p�x
	} else if (xv_mv_walk.x_percent_dlim < -EPS_DATA) {									/*	walk backward	*/
		float w1, w2, w3;

		w1	=	xp_mv_walk.x_bwd_swg - xp_mv_walk.x_bwd_spt;							// �V�r�̑O��̃X�g���C�h�̍�
		w2	=	xp_mv_walk.x_fwd_swg - xp_mv_walk.x_fwd_spt;							// �x���r�̑O��̃X�g���C�h�̍�
		w3	=	(float)fabs( w1 / w2 );

        if(accurate_one_step_mode == 1){
            xv_mv_walk.x_swg        =   (xv_mv_walk.accurate_step_x / 2.f) * xp_mv_walk.accurate_x_percent_dlim;
            xv_mv_walk.x_spt        =   (-xv_mv_walk.accurate_step_x / 2.f) * xp_mv_walk.accurate_x_percent_dlim;
        }else{
		    xv_mv_walk.x_swg		=	-xp_mv_walk.x_bwd_swg * xv_mv_walk.x_percent_dlim * w3;
		    xv_mv_walk.x_spt		=	-xp_mv_walk.x_bwd_spt * xv_mv_walk.x_percent_dlim * w3;
        }

		xv_mv_walk.pitch		=	-xp_mv_walk.x_bwd_pitch * xv_mv_walk.x_percent_dlim;// ���ɂ����ފp�x
		xv_mv_walk.arm_sh_pitch	=	-xp_mv_walk.arm_sh_pitch;							// ���̊p�x
		xv_mv_walk.arm_el_pitch =   -xp_mv_walk.arm_el_pitch;                           // �I�̊p�x
	} else {
		xv_mv_walk.x_swg		=	0.f;
		xv_mv_walk.x_spt		=	0.f;

		xv_mv_walk.pitch		=	0.f;
		xv_mv_walk.arm_sh_pitch	=	0.f;
		xv_mv_walk.arm_el_pitch =   0.f;
	}

	// �s�b�`�̌v�Z
	if (xv_dlim_wait_x.dout > 0) {
		xv_mv_walk.pitch		+=	xp_mv_walk.x_fwd_acc_pitch * xv_dlim_wait_x.dout;		// �O�ɂ����ފp�x�̉������̕␳
	} else if (xv_dlim_wait_x.dout < 0) {
		xv_mv_walk.pitch		-=	xp_mv_walk.x_bwd_acc_pitch * xv_dlim_wait_x.dout;		// ��ɂ���p�x�̉������̕␳
	}
    
	xv_dlim_wait_pitch.in = xp_mv_ready.pitch + xv_mv_walk.pitch;
	xv_data_pitch.pos = dlim_wait_fun( &xp_dlim_wait_pitch, &xv_dlim_wait_pitch );
	xv_data_pitch.time = 0.1;

	/***	calculation walk sidestep parameters	***/
    if(accurate_one_step_mode == 1){
        xv_mv_walk.y_swg            =   (xv_mv_walk.accurate_step_y / 2.f) * xp_mv_walk.accurate_y_percent_dlim;
        xv_mv_walk.y_spt            =   (-xv_mv_walk.accurate_step_y / 2.f) * xp_mv_walk.accurate_y_percent_dlim;
    }else{
        xv_mv_walk.y_swg			=	xp_mv_walk.y_swg * xv_mv_walk.y_percent_dlim;				// �V�r�̈ړ���
	    xv_mv_walk.y_spt			=	xp_mv_walk.y_spt * xv_mv_walk.y_percent_dlim;				// �x���r�̈ړ���
    }
    abs_y_percent				=	(float)fabs( xv_mv_walk.y_percent_dlim );					// ���Ɉړ�����䗦

	/***	calculation walk turn parameters	***/
    if(accurate_one_step_mode == 1){
        xv_mv_walk.theta = xv_mv_walk.accurate_step_th * xp_mv_walk.accurate_th_percent_dlim;
    }else{
        xv_mv_walk.theta = xp_mv_walk.theta * xv_mv_walk.theta_percent_dlim;
    }

    if( xv_mv_walk.y_percent_dlim > EPS_DATA ){													// ���E�ɉ���������ꍇ
	    flag_walk.y		=	RIGHT;
	    flag_walk.turn	=	RIGHT;
        if(accurate_one_step_mode == 1){
            xv_mv_walk.sidestep_roll        =   0.f;
            xv_mv_walk.sidestep_time_k_r    =   1.f;
            xv_mv_walk.sidestep_time_k_l    =   1.f;
        }else{
	        xv_mv_walk.sidestep_roll		=	xp_mv_walk.sidestep_roll  * abs_y_percent;			// ���������Ƀ��[��������p�x
	        xv_mv_walk.sidestep_time_k_r	=	1.0f + xp_mv_walk.sidestep_time_k * abs_y_percent;	// �E���̈ړ��̔䗦
	        xv_mv_walk.sidestep_time_k_l	=	1.0f;												// �����̈ړ��̔䗦
        }
    } else if( xv_mv_walk.y_percent_dlim < -EPS_DATA ) {										// �����ɉ���������ꍇ
	    flag_walk.y		=	LEFT ;
	    flag_walk.turn	=	LEFT ;
        if(accurate_one_step_mode == 1){
            xv_mv_walk.sidestep_roll        =   0.f;
            xv_mv_walk.sidestep_time_k_r    =   1.f;
            xv_mv_walk.sidestep_time_k_l    =   1.f;
        }else{
            xv_mv_walk.sidestep_roll		=	-xp_mv_walk.sidestep_roll  * abs_y_percent;			// ���������Ƀ��[��������p�x
            xv_mv_walk.sidestep_time_k_r	=	1.0f;												// �E���̈ړ��̔䗦
            xv_mv_walk.sidestep_time_k_l    =   1.0f + xp_mv_walk.sidestep_time_k * abs_y_percent; // �����̈ړ��̔䗦
        }
    } else {																					// ���^�����������ꍇ
	    flag_walk.y		=	STRAIGHT;
	    if       ( xv_mv_walk.theta_percent_dlim >  EPS_DATA ){									// ���E�ɐ���
		    flag_walk.turn	=	RIGHT   ;
	    } else if( xv_mv_walk.theta_percent_dlim < -EPS_DATA ){									// �����ɐ���
		    flag_walk.turn	=	LEFT    ;
	    } else {
		    flag_walk.turn	=	STRAIGHT;														// ������Ȃ�
	    }

	    xv_mv_walk.sidestep_roll			=	0.f;											// ���������Ƀ��[��������p�x
	    xv_mv_walk.sidestep_time_k_r		=													// �E���̈ړ��̔䗦
	    xv_mv_walk.sidestep_time_k_l		=	1.0f;											// �����̈ړ��̔䗦
    }

	/***	gyro yaw feedback control	***/
 	gyro_yaw_cntr_fun();																		// �W���C���ɂ�郈�[�������̐���

    /***	walk sequence	***/
	sq_walk_fun();																				// ���s�̊֐�

	return flag_md_walk_end;
}

void side_step_modify(float t1, float t2, float t1a, float t1b, float t1c, float t2a,
					  float *t1_r, float *t2_r, float *t1a_r, float *t1b_r, float *t1c_r, float *t2a_r,
					  float *t1_l, float *t2_l, float *t1a_l, float *t1b_l, float *t1c_l, float *t2a_l,
					  float *_xv_mv_walk_y_swg, float *_xv_mv_walk_y_spt, float *_xv_posture_roll2){
	/***	calculation sidestep time and change sidestep direction	***/
	/*	right leg	*/
	*t1_r	=	t1  * xv_mv_walk.sidestep_time_k_r;					// ����/2
	*t2_r	=	t2  * xv_mv_walk.sidestep_time_k_r;					// ����/4
	*t1a_r	=	t1a * xv_mv_walk.sidestep_time_k_r;					// �V�r
	*t1b_r	=	t1b * xv_mv_walk.sidestep_time_k_r;					// ����/2�ł̎x���r
	*t1c_r	=	t1c * xv_mv_walk.sidestep_time_k_r;					// �����ł̎x���r
	*t2a_r	=	t2a * xv_mv_walk.sidestep_time_k_r;					// �V�r/2
	/*	left leg	*/
	*t1_l	=	t1  * xv_mv_walk.sidestep_time_k_l;					// ����/2
	*t2_l	=	t2  * xv_mv_walk.sidestep_time_k_l;					// ����/4
	*t1a_l	=	t1a * xv_mv_walk.sidestep_time_k_l;					// �V�r
	*t1b_l	=	t1b * xv_mv_walk.sidestep_time_k_l;					// ����/2�ł̎x���r
	*t1c_l	=	t1c * xv_mv_walk.sidestep_time_k_l;					// �����ł̎x���r
	*t2a_l	=	t2a * xv_mv_walk.sidestep_time_k_l;					// �V�r/2
	/*	leg change	*/
	*_xv_mv_walk_y_swg		=	xv_mv_walk.y_swg;					// �������̎��̗V�r�̈ړ���
	*_xv_mv_walk_y_spt		=	xv_mv_walk.y_spt;					// �������̎��̎x���r�̈ړ���
	*_xv_posture_roll2		=	xv_mv_walk.sidestep_roll;			// �������̎��̃��[���̉�]�p
}


/*--------------------------------------*/
/*	sq_walk_fun 						*/
/*--------------------------------------*/
void	sq_walk_fun( void )
{
	static 	float	t1, t2;
	static 	float	t1a, t1b, t1c, t2a;
	static 	float	t1_r, t2_r;
	static 	float	t1a_r, t1b_r, t1c_r, t2a_r;
	static 	float	t1_l, t2_l;
	static 	float	t1a_l, t1b_l, t1c_l, t2a_l;
	static 	float	mode_sq_time;
	static	float	_xv_mv_walk_y_swg;
	static	float	_xv_mv_walk_y_spt;
	static	float	_xv_posture_roll2;

	float	work;

    if(accurate_one_step_mode == 1){
        t1	=	xv_mv_walk.accurate_step_time;
        t2	=	xv_mv_walk.accurate_step_time/2.f;

        t1a	=	xv_mv_walk.accurate_step_time * xv_mv_walk.time_dutyfactor;
        t1b	=	xv_mv_walk.accurate_step_time - t1a;
        t1c	=	xv_mv_walk.accurate_step_time * 2.f - t1a;
	    t2a	=	t1a/2.f;

	    flag_gyro.vib			=	ON;
    }else{
	    t1	=	xv_mv_walk.time;												/*	walk period (one step)			*/
	    t2	=	xv_mv_walk.time/2.f;											/*	half of walk period				*/

	    t1a	=	xv_mv_walk.time * xv_mv_walk.time_dutyfactor;					/*	time during swing leg			*/
	    t1b	=	xv_mv_walk.time - t1a;											/*									*/
	    t1c	=	xv_mv_walk.time * 2.f - t1a;									/*	time during support leg			*/
	    t2a	=	t1a/2.f;														/*	time during swing leg up		*/

	    flag_gyro.vib			=	ON;											//	gain high
    }

	switch( mode_sq_walk )
	{
		case	SQ_WALK_INIT:												/*	init	*/
			/* action */
			flag_gyro.vib		=	ON;										//	gain normal
			
			xv_mv.count			=	0;										/*	walk counter clear		*/
			flag_md_walk_end 	= 	OFF;									/*	walk end flag OFF		*/

			set_sw_ref_d(FOOT_XYZ);											//	foot xyz position control

			/*	X trajectory table = lamp	*/
			xv_data_x_r.mv_tbl_select		=	MV_TBL_LAMP;				// �E���̃e�[�u���̓����v�֐�
			xv_data_x_l.mv_tbl_select		=	MV_TBL_LAMP;				// �����̃e�[�u���̓����v�֐� 
			chg_mvtbl( &xv_mvdata[0], &xv_data_x_r );						// �O���f�[�^xv_mvdata[0]�̃��Z�b�g
			chg_mvtbl( &xv_mvdata[3], &xv_data_x_l );						// �O���f�[�^xv_mvdata[3]�̃��Z�b�g 

			/*	THETA trajectory table = lamp	*/
			xv_data_d[LEG_YAW_R].mv_tbl_select	=	MV_TBL_LAMP;			// �E���[���̓����v�֐�
			xv_data_d[LEG_YAW_L].mv_tbl_select	=	MV_TBL_LAMP;			// �����[���̓����v�֐�
			chg_mvtbl( &xv_mvdata_d[LEG_YAW_R], &xv_data_d[LEG_YAW_R] );	// �O���f�[�^xv_mvdata_d[LEG_YAW_R]�̃��Z�b�g
			chg_mvtbl( &xv_mvdata_d[LEG_YAW_L], &xv_data_d[LEG_YAW_L] );	// �O���f�[�^xv_mvdata_d[LEG_YAW_L]�̃��Z�b�g

			/*	Y(zmp) trajectory table = zmp curve	*/
			xv_data_y_r.mv_tbl_select		=	mv_tbl_zmp_sel;				// MV_TBL_ZMP2��ݒ�
			xv_data_y_l.mv_tbl_select		=	mv_tbl_zmp_sel;				// MV_TBL_ZMP2��ݒ�
			chg_mvtbl( &xv_mvdata[1], &xv_data_y_r );						// �O���f�[�^xv_mvdata[1]�̃��Z�b�g
			chg_mvtbl( &xv_mvdata[4], &xv_data_y_l );						// �O���f�[�^xv_mvdata[4]�̃��Z�b�g

			/*	Y(side step) trajectory table = zmp curve	*/
			xv_data_y_r2.mv_tbl_select		=	mv_tbl_zmp_sel;				// MV_TBL_ZMP2��ݒ�
			xv_data_y_l2.mv_tbl_select		=	mv_tbl_zmp_sel;				// MV_TBL_ZMP2��ݒ�
			chg_mvtbl( &xv_mvdata[ 9], &xv_data_y_r2 );						// �O���f�[�^xv_mvdata[ 9]�̃��Z�b�g
			chg_mvtbl( &xv_mvdata[10], &xv_data_y_l2 );						// �O���f�[�^xv_mvdata[10]�̃��Z�b�g

			flag_walk.y_on		=	flag_walk.y;							/* store actual walk y direction	*/
            flag_walk.turn_on	=	flag_walk.turn;							/* store actual turn direction		*/
            
            side_step_modify(t1, t2, t1a, t1b, t1c, t2a,					// �������̂��߂̃p�����[�^�C��
			    &t1_r, &t2_r, &t1a_r, &t1b_r, &t1c_r, &t2a_r,
			    &t1_l, &t2_l, &t1a_l, &t1b_l, &t1c_l, &t2a_l,
			    &_xv_mv_walk_y_swg, &_xv_mv_walk_y_spt, &_xv_posture_roll2);  
            
            mode_sq_time	=	0.f;										/*	clear state time		*/

			/* status */
			// �����n�߂�Ƃ��C���ɐi�ޏꍇ�͂����瑤�̑�����グ��D
			// ���Ƀ^�[������ꍇ���C�i�ޕ����̑�����グ��D
			// �^�������i�ޏꍇ�͍Ō�ɉ��낵��������グ��D

			if( flag_walk.y == RIGHT )										/*	sidestep right			*/
			{
				mode_sq_walk	=	SQ_WALK_MV_R;							/*	move cog to left		*/
			}					
			else if( flag_walk.y == LEFT )									/*	sidestep left			*/
			{
				mode_sq_walk	=	SQ_WALK_MV_L;							/*	move cog to right		*/
			}
			else
			{
				if( flag_walk.turn == RIGHT )								/*	turn right				*/
					mode_sq_walk	=	SQ_WALK_MV_R;						/*	move cog to left		*/
				else if( flag_walk.turn == LEFT )							/*	turn left				*/
					mode_sq_walk	=	SQ_WALK_MV_L;						/*	move cog to right		*/
				else
				{															/*	start opposite leg		*/
					if( flag_walk.upleg_last == RIGHT )						/*	last step is right leg	*/
						mode_sq_walk	=	SQ_WALK_MV_L;					/*	move cog to right		*/
					else													/*	last step is left leg	*/
						mode_sq_walk	=	SQ_WALK_MV_R;					/*	move cog to left		*/
				}
			}

			break;

		case	SQ_WALK_MV_L:		/*	move cog to left	*/
			/*	first step should be adjusted because not based on zmp	*/
			xv_data_y_r.time		=
			xv_data_y_l.time		=	t2 * xp_mv_walk.start_time_k1;
			work					=	xv_mv_walk.zmp * xp_mv_walk.start_zmp_k1 + xp_mv_walk.y_wide;
			//xv_data_y_r.pos			=	limit( work, xp_mv_walk.y_balance, -xp_mv_walk.y_balance );
            xv_data_y_r.pos = work;
            work					=	xv_mv_walk.zmp * xp_mv_walk.start_zmp_k1 - xp_mv_walk.y_wide;
			//xv_data_y_l.pos			=	limit( work, xp_mv_walk.y_balance, -xp_mv_walk.y_balance );
            xv_data_y_l.pos = work;
			mode_sq_time	=	0.f;

			if( ++xv_mv.count >= xv_mv_walk.num )		mode_sq_walk	=	SQ_WALK_UP_R2;		/*	go to last step	*/
			else										mode_sq_walk	=	SQ_WALK_UP_R0;		/*	continue		*/

			break;

		case	SQ_WALK_MV_R:		/*	move cog to right	*/
			xv_data_y_r.time		=
			xv_data_y_l.time		=	t2 * xp_mv_walk.start_time_k1;
			work					=	-xv_mv_walk.zmp * xp_mv_walk.start_zmp_k1 + xp_mv_walk.y_wide;
			xv_data_y_r.pos			=	limit( work, xp_mv_walk.y_balance, -xp_mv_walk.y_balance );
			work					=	-xv_mv_walk.zmp * xp_mv_walk.start_zmp_k1 - xp_mv_walk.y_wide;
			xv_data_y_l.pos			=	limit( work, xp_mv_walk.y_balance, -xp_mv_walk.y_balance );

			mode_sq_time	=	0.f;

			if( ++xv_mv.count >= xv_mv_walk.num )		mode_sq_walk	=	SQ_WALK_UP_L2;
			else										mode_sq_walk	=	SQ_WALK_UP_L0;

			break;

		case	SQ_WALK_UP_R0:		/*	up right leg for first step	*/
			if (mode_sq_time < (t2 * xp_mv_walk.start_time_k1 - EPS_TIME)) break;	/*	wait until previos state finish	*/
			
			mode_sq_walk	=	SQ_WALK_UP_R;
//			break;

		case	SQ_WALK_UP_R:		/*	up right leg	*/
			flag_walk.upleg 			= 	RIGHT;									// �グ�鑫
			if( flag_walk.y_on == RIGHT )											// �E�ɓ����ꍇ
			{
				xv_data_x_l.time		=	t1c_r;									// �����i�x���r�j��O��ɓ���������
				xv_data_x_l.pos			=	xv_mv_walk.x_spt;						// �����i�x���r�j��O��ɓ���������
				xv_data_x_l.mv_tbl_select = MV_TBL_LAMP;

				xv_data_y_l2.time		=	t1c_r;									// �����i�x���r�j�����E�ɓ���������
				xv_data_y_l2.pos		=	_xv_mv_walk_y_spt;						// �����i�x���r�j�����E�ɓ���������
				xv_data_roll2.time		=	t2_r;									// ���[�����𓮂�������
				xv_data_roll2.pos		=	_xv_posture_roll2;						// ���[�����𓮂����p�x
			}
			else if( flag_walk.y_on == LEFT )
			{
				xv_data_x_l.time		=	t1c_r;									// �����i�x���r�j��O��ɓ���������
				xv_data_x_l.pos			=	xv_mv_walk.x_spt;						// �����i�x���r�j��O��ɓ���������
				xv_data_x_l.mv_tbl_select = MV_TBL_LAMP;

				xv_data_y_l2.time		=	t1c_r;									// �����i�x���r�j�����E�ɓ���������
				xv_data_y_l2.pos		=	0.f;									// �����i�x���r�j�����E�ɓ���������
				xv_data_roll2.time		=	t2_r;									// ���[�����𓮂�������
				xv_data_roll2.pos		=	_xv_posture_roll2;						// ���[�����𓮂����p�x
			}
			else
			{
				xv_data_x_l.time		=	t1c_r;									// �����i�x���r�j��O��ɓ���������
				xv_data_x_l.pos			=	xv_mv_walk.x_spt;						// �����i�x���r�j��O��ɓ���������
				xv_data_x_l.mv_tbl_select = MV_TBL_LAMP;

				xv_data_y_l2.time		=	t1c_r;									// �����i�x���r�j�����E�ɓ���������
				xv_data_y_l2.pos		=	0.f;									// �����i�x���r�j�����E�ɓ���������
				xv_data_roll2.time		=	t2_r;									// ���[�����𓮂�������
				xv_data_roll2.pos		=	0.f;									// ���[�����𓮂����p�x
			}

			mode_sq_walk	=	SQ_WALK_UP_R_1;

			break;

		case	SQ_WALK_UP_R_1:		/*	up right leg	*/
			if (mode_sq_time < (t1b_r - EPS_TIME)) break;							// �����x�����Ԃ��I������܂ŁC���ɐi�܂Ȃ�

//			xv_data_x_r.pos			=	xv_mv_walk.x_swg;							// �E���i�V�r�j��O��ɓ���������
//			xv_data_x_r.time		=	t1a_r;										// �E���i�V�r�j��O��ɓ���������
			xv_data_x_r.pos			=	0.0;										// �E���i�V�r�j��O��ɓ���������
			xv_data_x_r.time		=	t2a_r;										// �E���i�V�r�j��O��ɓ���������
			xv_data_x_r.mv_tbl_select = MV_TBL_X_UP;

			if (flag_walk.y_on == RIGHT){
				xv_data_y_r2.pos	=	_xv_mv_walk_y_swg;							// �E���i�V�r�j�����E�ɓ���������
			} else {
				xv_data_y_r2.pos	=	0.f;
			}
			xv_data_y_r2.time		=	t1a_r;										// �E���i�V�r�j�����E�ɓ���������
            
            work = (accurate_one_step_mode == 1 ? -xv_mv_walk.accurate_step_z : -xv_mv_walk.z) + xp_mv_ready.z3;     // ���f�B�̍������瑫�グ�ʂ��������l
			xv_data_z_r.pos			=	limit_l( work, Z3_LIMIT_L );				// �E���i�V�r�j���㉺�ɓ���������
			xv_data_z_r.mv_tbl_select	=	MV_TBL_Z_UP;							// �E���i�V�r�j���グ��O���̃e�[�u��
			xv_data_z_r.time		=	t2a_r;										// �E���i�V�r�j���㉺�ɓ���������

			// �r�U��
			xv_data_d[ARM_PITCH_R].time		=
			xv_data_d[ARM_PITCH_L].time		=
			xv_data_d[ELBOW_PITCH_R].time   =
			xv_data_d[ELBOW_PITCH_L].time   =   t1_r; 
			xv_data_d[ARM_PITCH_R].pos		=	xp_mv_ready.arm_sh_pitch - xv_mv_walk.arm_sh_pitch;
			xv_data_d[ARM_PITCH_L].pos		=	xp_mv_ready.arm_sh_pitch + xv_mv_walk.arm_sh_pitch;
			xv_data_d[ELBOW_PITCH_R].pos		=	xp_mv_ready.arm_el_pitch - xv_mv_walk.arm_el_pitch;
         	xv_data_d[ELBOW_PITCH_L].pos		=	xp_mv_ready.arm_el_pitch + xv_mv_walk.arm_el_pitch;

			// ���[���̐���
			xv_data_d[LEG_YAW_R].time		=										// ���[���𓮂�������
			xv_data_d[LEG_YAW_L].time		=	t1a_r;

			if( flag_walk.turn_on == RIGHT ){										// �E�ɐ��񂷂�ꍇ
				xv_data_d[LEG_YAW_R].pos		=	xv_mv_walk.theta;				// �E�����グ��Ƃ��ɐ���
				xv_data_d[LEG_YAW_L].pos		=	-xv_mv_walk.theta;
			} else {
				xv_data_d[LEG_YAW_R].pos		=	0.f;
				xv_data_d[LEG_YAW_L].pos		=	0.f;
			}

			mode_sq_walk	=	SQ_WALK_DW_R;
			mode_sq_time	=	0.f;
			break;

		case	SQ_WALK_DW_R:		/*	down right leg	*/
			if (mode_sq_time < (t2a_r - RTC_TIME_SEC - EPS_TIME)) break;			// �����グ��܂Ő�ɐi�܂Ȃ�

			xv_data_y_l.time		=	t1_r;										// �E���i�V�r�j�����E�ɓ���������(1/2����)
			xv_data_y_l.pos			=	-xv_mv_walk.zmp - xp_mv_walk.y_wide;		// �E���i�V�r�j�����E�ɓ������ʒu

			xv_data_y_r.time		=	t1_r;										// �����i�x���r�j�����E�ɓ���������(1/2����)
			xv_data_y_r.pos			=	-xv_mv_walk.zmp + xp_mv_walk.y_wide;		// �����i�x���r�j�����E�ɓ������ʒu

			xv_data_x_r.pos			=	xv_mv_walk.x_swg;						// �E���i�V�r�j��O��ɓ���������
			xv_data_x_r.time		=	t2a_r;										// �E���i�V�r�j��O��ɓ���������
			xv_data_x_r.mv_tbl_select = MV_TBL_X_DW;

			xv_data_z_r.pos			=	limit_l( xp_mv_ready.z3, Z3_LIMIT_L );		// �E���i�V�r�j���㉺�ɓ������ʒu
			xv_data_z_r.mv_tbl_select	=	MV_TBL_Z_DW;							// �E���i�V�r�j���㉺�ɓ������O���̃e�[�u��
			xv_data_z_r.time		=	t2a_r;										// �E���i�V�r�j���㉺�ɓ���������

			mode_sq_walk	=	SQ_WALK_MV_R2;
			mode_sq_time	=	0.f;
			break;

		case	SQ_WALK_MV_R2:		/*	move cog to right	*/
			if( mode_sq_time < (t2a_r - EPS_TIME)) break;							// ����������܂Ő�ɐi�܂Ȃ�

			flag_walk.upleg 	= 	OFF;

			if( flag_walk.y_on != RIGHT ){
                
			    side_step_modify(t1, t2, t1a, t1b, t1c, t2a,						// �������̂��߂̃p�����[�^�C��
				    &t1_r, &t2_r, &t1a_r, &t1b_r, &t1c_r, &t2a_r,
				    &t1_l, &t2_l, &t1a_l, &t1b_l, &t1c_l, &t2a_l,
				    &_xv_mv_walk_y_swg, &_xv_mv_walk_y_spt, &_xv_posture_roll2);
                
                flag_walk.y_on	=	flag_walk.y;									/*	store actual walk y direction	*/
			}

			if( flag_walk.turn_on != RIGHT ){
				flag_walk.turn_on	=	flag_walk.turn;								/*	store actual turn direction		*/
			}

			if( ++xv_mv.count >= xv_mv_walk.num ){
				mode_sq_walk	=	SQ_WALK_UP_L2;									/*	go to last step  	*/
			} else {
//				mode_sq_walk	=	SQ_WALK_UP_L;									/*	continue	*/
				flag_walk.upleg 		= 	LEFT;										// �グ�鑫
				if( flag_walk.y_on == RIGHT )											// �E�ɓ����ꍇ
				{
					xv_data_x_r.time		=	t1c_l;									// �E���i�x���r�j��O��ɓ���������
					xv_data_x_r.pos			=	xv_mv_walk.x_spt;						// �E���i�x���r�j��O��ɓ���������
					xv_data_x_r.mv_tbl_select = MV_TBL_LAMP;

					xv_data_y_r2.time		=	t1c_l;									// �E���i�x���r�j�����E�ɓ���������
					xv_data_y_r2.pos		=	0.f;									// �E���i�x���r�j�����E�ɓ���������
					xv_data_roll2.time		=	t2_l;									// ���[�����𓮂�������
					xv_data_roll2.pos		=	_xv_posture_roll2;						// ���[�����𓮂����p�x
				}
				else if( flag_walk.y_on == LEFT )
				{
					xv_data_x_r.time		=	t1c_l;									// �E���i�x���r�j��O��ɓ���������
					xv_data_x_r.pos			=	xv_mv_walk.x_spt;						// �E���i�x���r�j��O��ɓ���������
					xv_data_x_r.mv_tbl_select = MV_TBL_LAMP;
					
					xv_data_y_r2.time		=	t1c_l;									// �E���i�x���r�j�����E�ɓ���������
					xv_data_y_r2.pos		=	_xv_mv_walk_y_spt;						// �E���i�x���r�j�����E�ɓ���������
					xv_data_roll2.time		=	t2_l;									// ���[�����𓮂�������
					xv_data_roll2.pos		=	_xv_posture_roll2;						// ���[�����𓮂����p�x
				}
				else
				{
					xv_data_x_r.time		=	t1c_l;									// �E���i�x���r�j��O��ɓ���������
					xv_data_x_r.pos			=	xv_mv_walk.x_spt;						// �E���i�x���r�j��O��ɓ���������
					xv_data_x_r.mv_tbl_select = MV_TBL_LAMP;
					
					xv_data_y_r2.time		=	t1c_l;									// �E���i�x���r�j�����E�ɓ���������
					xv_data_y_r2.pos		=	0.f;									// �E���i�x���r�j�����E�ɓ���������
					xv_data_roll2.time		=	t2_l;									// ���[�����𓮂�������
					xv_data_roll2.pos		=	0.f;									// ���[�����𓮂����p�x
				}

				mode_sq_walk	=	SQ_WALK_UP_L_1;
			}

			mode_sq_time	=	0.f;
			break;

		case	SQ_WALK_UP_L0:		/*	up left leg for first step	*/
			if (mode_sq_time < (t2 * xp_mv_walk.start_time_k1 - EPS_TIME)) break;	/*	wait until previos state finish	*/

			mode_sq_walk	=	SQ_WALK_UP_L;
//			break;

		case	SQ_WALK_UP_L:		/*	up left leg		*/
			flag_walk.upleg 		= 	LEFT;										// �グ�鑫
			if( flag_walk.y_on == RIGHT )											// �E�ɓ����ꍇ
			{
				xv_data_x_r.time		=	t1c_l;									// �E���i�x���r�j��O��ɓ���������
				xv_data_x_r.pos			=	xv_mv_walk.x_spt;						// �E���i�x���r�j��O��ɓ���������
				xv_data_x_r.mv_tbl_select = MV_TBL_LAMP;
				
				xv_data_y_r2.time		=	t1c_l;									// �E���i�x���r�j�����E�ɓ���������
				xv_data_y_r2.pos		=	0.f;									// �E���i�x���r�j�����E�ɓ���������
				xv_data_roll2.time		=	t2_l;									// ���[�����𓮂�������
				xv_data_roll2.pos		=	_xv_posture_roll2;						// ���[�����𓮂����p�x
			}
			else if( flag_walk.y_on == LEFT )
			{
				xv_data_x_r.time		=	t1c_l;									// �E���i�x���r�j��O��ɓ���������
				xv_data_x_r.pos			=	xv_mv_walk.x_spt;						// �E���i�x���r�j��O��ɓ���������
				xv_data_x_r.mv_tbl_select = MV_TBL_LAMP;
				
				xv_data_y_r2.time		=	t1c_l;									// �E���i�x���r�j�����E�ɓ���������
				xv_data_y_r2.pos		=	_xv_mv_walk_y_spt;						// �E���i�x���r�j�����E�ɓ���������
				xv_data_roll2.time		=	t2_l;									// ���[�����𓮂�������
				xv_data_roll2.pos		=	_xv_posture_roll2;						// ���[�����𓮂����p�x
			}
			else
			{
				xv_data_x_r.time		=	t1c_r;									// �E���i�x���r�j��O��ɓ���������
				xv_data_x_r.pos			=	xv_mv_walk.x_spt;						// �E���i�x���r�j��O��ɓ���������
				xv_data_x_r.mv_tbl_select = MV_TBL_LAMP;
				
				xv_data_y_r2.time		=	t1c_r;									// �E���i�x���r�j�����E�ɓ���������
				xv_data_y_r2.pos		=	0.f;									// �E���i�x���r�j�����E�ɓ���������
				xv_data_roll2.time		=	t2_r;									// ���[�����𓮂�������
				xv_data_roll2.pos		=	0.f;									// ���[�����𓮂����p�x
			}

			mode_sq_walk	=	SQ_WALK_UP_L_1;

			break;

		case	SQ_WALK_UP_L_1:		/*	up left leg		*/
			if (mode_sq_time < (t1b_l - EPS_TIME)) break;							// �����x�����Ԃ��I������܂ŁC���ɐi�܂Ȃ�

//			xv_data_x_l.pos			=	xv_mv_walk.x_swg;							// �E���i�V�r�j��O��ɓ���������
//			xv_data_x_l.time		=	t1a_l;										// �E���i�V�r�j��O��ɓ���������
			xv_data_x_l.pos			=	0.0;										// �E���i�V�r�j��O��ɓ���������
			xv_data_x_l.time		=	t2a_l;										// �E���i�V�r�j��O��ɓ���������
			xv_data_x_l.mv_tbl_select = MV_TBL_X_UP;

			if (flag_walk.y_on == LEFT){
				xv_data_y_l2.pos	=	_xv_mv_walk_y_swg;							// �����i�V�r�j�����E�ɓ���������
			} else {
				xv_data_y_l2.pos	=	0.f;
			}
			xv_data_y_l2.time	=	t1a_l;											// �����i�V�r�j�����E�ɓ���������

            work = (accurate_one_step_mode == 1 ? -xv_mv_walk.accurate_step_z : -xv_mv_walk.z) + xp_mv_ready.z3; // ���f�B�̍������瑫�グ�ʂ��������l
			xv_data_z_l.pos			=	limit_l( work, Z3_LIMIT_L );				// �����i�V�r�j���㉺�ɓ���������
			xv_data_z_l.mv_tbl_select	=	MV_TBL_Z_UP;							// �����i�V�r�j���グ��O���̃e�[�u��
			xv_data_z_l.time		=	t2a_l;										// �����i�V�r�j���㉺�ɓ���������

			// �r�U��
			xv_data_d[ARM_PITCH_R].time		=
			xv_data_d[ARM_PITCH_L].time		=
			xv_data_d[ELBOW_PITCH_R].time		=
			xv_data_d[ELBOW_PITCH_L].time		=	t1_l;
			xv_data_d[ARM_PITCH_R].pos		=	xp_mv_ready.arm_sh_pitch + xv_mv_walk.arm_sh_pitch;
			xv_data_d[ARM_PITCH_L].pos		=	xp_mv_ready.arm_sh_pitch - xv_mv_walk.arm_sh_pitch;
			xv_data_d[ELBOW_PITCH_R].pos		=	xp_mv_ready.arm_el_pitch + xv_mv_walk.arm_el_pitch;
        	xv_data_d[ELBOW_PITCH_L].pos		=	xp_mv_ready.arm_el_pitch - xv_mv_walk.arm_el_pitch;

			// ���[���̐���
			xv_data_d[LEG_YAW_R].time		=										// ���[���𓮂�������
			xv_data_d[LEG_YAW_L].time		=	t1a_l;

			if (flag_walk.turn_on == LEFT){											// ���ɐ��񂷂�ꍇ
				xv_data_d[LEG_YAW_R].pos		=	-xv_mv_walk.theta;				// �������グ��Ƃ��ɐ���
				xv_data_d[LEG_YAW_L].pos		=	xv_mv_walk.theta;
			} else {
				xv_data_d[LEG_YAW_R].pos		=	0.f;
				xv_data_d[LEG_YAW_L].pos		=	0.f;
			}

			mode_sq_walk	=	SQ_WALK_DW_L;
			mode_sq_time	=	0.f;
			break;

		case	SQ_WALK_DW_L:		/*	down left leg		*/
			if (mode_sq_time < (t2a_l - RTC_TIME_SEC - EPS_TIME)) break;			// �����グ��܂Ő�ɐi�܂Ȃ�

			xv_data_y_l.time		=	t1_l;										// �����i�V�r�j�����E�ɓ���������(1/2����)
			xv_data_y_l.pos			=	xv_mv_walk.zmp - xp_mv_walk.y_wide;			// �����i�V�r�j�����E�ɓ������ʒu

			xv_data_y_r.time		=	t1_l;										// �E���i�x���r�j�����E�ɓ���������(1/2����)
			xv_data_y_r.pos			=	xv_mv_walk.zmp + xp_mv_walk.y_wide;			// �E���i�x���r�j�����E�ɓ������ʒu

			xv_data_x_l.pos			=	xv_mv_walk.x_swg;							// �E���i�V�r�j��O��ɓ���������
			xv_data_x_l.time		=	t2a_l;										// �E���i�V�r�j��O��ɓ���������
			xv_data_x_l.mv_tbl_select = MV_TBL_X_DW;

			xv_data_z_l.pos			=	limit_l( xp_mv_ready.z3, Z3_LIMIT_L );		// �����i�V�r�j���㉺�ɓ������ʒu
			xv_data_z_l.mv_tbl_select	=	MV_TBL_Z_DW;							// �����i�V�r�j���㉺�ɓ������O���̃e�[�u��
			xv_data_z_l.time		=	t2a_l;										// �����i�V�r�j���㉺�ɓ���������

			mode_sq_walk	=	SQ_WALK_MV_L2;
			mode_sq_time	=	0.f;

			break;

		case	SQ_WALK_MV_L2:		/*	move cog to left	*/
			if (mode_sq_time < (t2a_l - EPS_TIME)) break;							// ����������܂Ő�ɐi�܂Ȃ�

			flag_walk.upleg 	= 	OFF;

			if( flag_walk.y_on != LEFT ){
                
			    side_step_modify(t1, t2, t1a, t1b, t1c, t2a,						// �������̂��߂̃p�����[�^�C��
				    &t1_r, &t2_r, &t1a_r, &t1b_r, &t1c_r, &t2a_r,
				    &t1_l, &t2_l, &t1a_l, &t1b_l, &t1c_l, &t2a_l,
				    &_xv_mv_walk_y_swg, &_xv_mv_walk_y_spt, &_xv_posture_roll2);
                
                flag_walk.y_on	=	flag_walk.y;									/*	store actual walk y direction	*/
			}

			if( flag_walk.turn_on != LEFT ){
				flag_walk.turn_on	=	flag_walk.turn;								/*	store actual turn direction		*/
			}

			if( ++xv_mv.count >= xv_mv_walk.num ){
				mode_sq_walk	=	SQ_WALK_UP_R2;									/*	go to last step 	*/
			} else {
//				mode_sq_walk	=	SQ_WALK_UP_R;									/*	continue	*/
				flag_walk.upleg 			= 	RIGHT;									// �グ�鑫
				if( flag_walk.y_on == RIGHT )											// �E�ɓ����ꍇ
				{
					xv_data_x_l.time		=	t1c_r;									// �����i�x���r�j��O��ɓ���������
					xv_data_x_l.pos			=	xv_mv_walk.x_spt;						// �����i�x���r�j��O��ɓ���������
					xv_data_x_l.mv_tbl_select = MV_TBL_LAMP;
					
					xv_data_y_l2.time		=	t1c_r;									// �����i�x���r�j�����E�ɓ���������
					xv_data_y_l2.pos		=	_xv_mv_walk_y_spt;						// �����i�x���r�j�����E�ɓ���������
					xv_data_roll2.time		=	t2_r;									// ���[�����𓮂�������
					xv_data_roll2.pos		=	_xv_posture_roll2;						// ���[�����𓮂����p�x
				}
				else if( flag_walk.y_on == LEFT )
				{
					xv_data_x_l.time		=	t1c_r;									// �����i�x���r�j��O��ɓ���������
					xv_data_x_l.pos			=	xv_mv_walk.x_spt;						// �����i�x���r�j��O��ɓ���������
					xv_data_x_l.mv_tbl_select = MV_TBL_LAMP;
					
					xv_data_y_l2.time		=	t1c_r;									// �����i�x���r�j�����E�ɓ���������
					xv_data_y_l2.pos		=	0.f;									// �����i�x���r�j�����E�ɓ���������
					xv_data_roll2.time		=	t2_r;									// ���[�����𓮂�������
					xv_data_roll2.pos		=	_xv_posture_roll2;						// ���[�����𓮂����p�x
				}
				else
				{
					xv_data_x_l.time		=	t1c_r;									// �����i�x���r�j��O��ɓ���������
					xv_data_x_l.pos			=	xv_mv_walk.x_spt;						// �����i�x���r�j��O��ɓ���������
					xv_data_x_l.mv_tbl_select = MV_TBL_LAMP;
					
					xv_data_y_l2.time		=	t1c_r;									// �����i�x���r�j�����E�ɓ���������
					xv_data_y_l2.pos		=	0.f;									// �����i�x���r�j�����E�ɓ���������
					xv_data_roll2.time		=	t2_r;									// ���[�����𓮂�������
					xv_data_roll2.pos		=	0.f;									// ���[�����𓮂����p�x
				}

				mode_sq_walk	=	SQ_WALK_UP_R_1;			
			}

			mode_sq_time	=	0.f;
			break;

		case	SQ_WALK_UP_R2:		/*	up right leg for last step	*/
			flag_walk.upleg 	= 	RIGHT;
			xv_data_x_l.time	=	t1c_r;
			xv_data_x_l.pos		=	0.f;
			xv_data_x_l.mv_tbl_select = MV_TBL_LAMP;

			xv_data_y_l2.time	=	t1c_r;
			xv_data_y_l2.pos	=	0.f;

			mode_sq_walk	=	SQ_WALK_UP_R2_1;

			break;

		case	SQ_WALK_UP_R2_1:	/*	up right leg for last step	*/
			if (mode_sq_time < (t1b_r - EPS_TIME)) break;

			xv_data_z_r.mv_tbl_select	=	MV_TBL_Z_UP;

			xv_data_x_r.time		=	t1a_r;
			xv_data_z_r.time		=	t2a_r;

			xv_data_x_r.pos			=	0.f;
			xv_data_x_r.mv_tbl_select = MV_TBL_LAMP;

			xv_data_y_r2.time	=	t1a_r;
			xv_data_y_r2.pos	=	0.f;
            
            work = (accurate_one_step_mode == 1 ? -xv_mv_walk.accurate_step_z : -xv_mv_walk.z) + xp_mv_ready.z3; 
			xv_data_z_r.pos			=	limit_l( work, Z3_LIMIT_L );

			xv_data_z_l.time		=	t2a_r;
			xv_data_z_l.pos			=	xp_mv_ready.z3;

			xv_data_d[ARM_PITCH_R].time		=
			xv_data_d[ARM_PITCH_L].time		=	t1_r;
			xv_data_d[ARM_PITCH_R].pos		=	xp_mv_ready.arm_sh_pitch - xv_mv_walk.arm_sh_pitch;
			xv_data_d[ARM_PITCH_L].pos		=	xp_mv_ready.arm_sh_pitch + xv_mv_walk.arm_sh_pitch;

			mode_sq_walk	=	SQ_WALK_UP_R2_2;
			mode_sq_time	=	0.f;

//			break;

		case	SQ_WALK_UP_R2_2:	/*	up right leg for last step	*/
			xv_data_d[LEG_YAW_R].time		=
			xv_data_d[LEG_YAW_L].time		=	t1a_r;

			xv_data_d[LEG_YAW_R].pos		=	0.f;
			xv_data_d[LEG_YAW_L].pos		=	0.f;

			mode_sq_walk	=	SQ_WALK_DW_R2;

			break;

		case	SQ_WALK_DW_R2:		/*	down right leg for last step	*/
			if (mode_sq_time < (t2a_r - EPS_TIME)) break;

			xv_data_z_r.mv_tbl_select	=	MV_TBL_Z_DW;

			xv_data_z_r.time		=	t2a_r;

			xv_data_z_r.pos			=	limit_l( xp_mv_ready.z3, Z3_LIMIT_L );

			xv_data_z_l.time		=	t2a_r;
			xv_data_z_l.pos			=	xp_mv_ready.z3;

			xv_data_y_r.time		=
			xv_data_y_l.time		=	t1_r;
			xv_data_y_r.pos			=	-xv_mv_walk.zmp + xp_mv_walk.y_wide;
			xv_data_y_l.pos			=	-xv_mv_walk.zmp - xp_mv_walk.y_wide;
			xv_data_roll2.time		=	t2_r;
			xv_data_roll2.pos		=	0.f;

			mode_sq_walk	=	SQ_WALK_READY;
			mode_sq_time	=	0.f;

			break;

		case	SQ_WALK_UP_L2:		/*	up left leg for last step	*/
			flag_walk.upleg 	= 	LEFT;
			xv_data_x_r.time		=	t1c_l;
			xv_data_x_r.pos			=	0.f;
			xv_data_x_r.mv_tbl_select = MV_TBL_LAMP;

			xv_data_y_r2.time	=	t1c_l;
			xv_data_y_r2.pos	=	0.f;

			mode_sq_walk	=	SQ_WALK_UP_L2_1;

			break;

		case	SQ_WALK_UP_L2_1:	/*	up left leg for last step	*/
			if(mode_sq_time < (t1b_l - EPS_TIME)) break;

			xv_data_z_l.mv_tbl_select	=	MV_TBL_Z_UP;

			xv_data_x_l.time		=	t1a_l;
			xv_data_z_l.time		=	t2a_l;

			xv_data_x_l.pos			=	0.f;
			xv_data_x_l.mv_tbl_select = MV_TBL_LAMP;
			
			xv_data_y_l2.time	=	t1a_l;
			xv_data_y_l2.pos	=	0.f;
            
            work = (accurate_one_step_mode == 1 ? -xv_mv_walk.accurate_step_z : -xv_mv_walk.z) + xp_mv_ready.z3;
			xv_data_z_l.pos			=	limit_l( work, Z3_LIMIT_L );

			xv_data_z_r.time		=	t2a_l;
			xv_data_z_r.pos			=	xp_mv_ready.z3;

			xv_data_d[ARM_PITCH_R].time		=
			xv_data_d[ARM_PITCH_L].time		=	t1_l;
			xv_data_d[ARM_PITCH_R].pos		=	xp_mv_ready.arm_sh_pitch + xv_mv_walk.arm_sh_pitch;
			xv_data_d[ARM_PITCH_L].pos		=	xp_mv_ready.arm_sh_pitch - xv_mv_walk.arm_sh_pitch;

			mode_sq_walk	=	SQ_WALK_UP_L2_2;
			mode_sq_time	=	0.f;

//			break;

		case	SQ_WALK_UP_L2_2:	/*	up left leg for last step	*/
			xv_data_d[LEG_YAW_R].time		=
			xv_data_d[LEG_YAW_L].time		=	t1a_l;

			xv_data_d[LEG_YAW_R].pos		=	0.f;
			xv_data_d[LEG_YAW_L].pos		=	0.f;

			mode_sq_walk	=	SQ_WALK_DW_L2;

			break;

		case	SQ_WALK_DW_L2:		/*	down left leg for last step	*/
			if (mode_sq_time < (t2a_l - EPS_TIME)) break;
			xv_data_z_l.mv_tbl_select	=	MV_TBL_Z_DW;

			xv_data_z_l.time		=	t2a_l;

			work					=	xp_mv_ready.z3;
			xv_data_z_l.pos			=	limit_l( work, Z3_LIMIT_L );

			xv_data_z_r.time		=	t2a_l;
			xv_data_z_r.pos			=	xp_mv_ready.z3;

			xv_data_y_r.time		=
			xv_data_y_l.time		=	t1_l;
			xv_data_y_r.pos			=	xv_mv_walk.zmp + xp_mv_walk.y_wide;
			xv_data_y_l.pos			=	xv_mv_walk.zmp - xp_mv_walk.y_wide;

			xv_data_roll2.time		=	t2_l;
			xv_data_roll2.pos		=	0.f;

			mode_sq_walk	=	SQ_WALK_READY;
			mode_sq_time	=	0.f;

			break;

		case	SQ_WALK_READY:		/*	ready position	*/
			flag_gyro.vib			=	ON;		//	normal gain

			if( mode_sq_time < (t2a - RTC_TIME_SEC*2 - EPS_TIME) ) break;

			flag_walk.upleg 		= 	OFF;
			flag_walk.y_on 			= 	STRAIGHT;
			flag_walk.turn_on 		= 	STRAIGHT;

			xv_data_z_r.mv_tbl_select	=	MV_TBL_SIN;
			xv_data_z_l.mv_tbl_select	=	MV_TBL_SIN;

			xv_data_y_r.time		=	t1;
			xv_data_z_r.time		=	t1;
			xv_data_y_l.time		=	t1;
			xv_data_z_l.time		=	t1;
			xv_data_roll2.time		=	t2;

			xv_data_y_r.pos			=	xp_mv_walk.y_wide;
			xv_data_z_r.pos			=	xp_mv_ready.z3;
			xv_data_y_l.pos			=	-xp_mv_walk.y_wide;
			xv_data_z_l.pos			=	xp_mv_ready.z3;
			xv_data_pitch.pos		=	xp_mv_ready.pitch;
			xv_data_roll2.pos		=	0.f;

			xv_data_d[ARM_PITCH_L].time		=	
			xv_data_d[ARM_PITCH_R].time		=	
			xv_data_d[ARM_ROLL_L].time		=	
			xv_data_d[ARM_ROLL_R].time		= 
			xv_data_d[ELBOW_PITCH_L].time   =
			xv_data_d[ELBOW_PITCH_R].time   =   t1;

			xv_data_d[ARM_PITCH_L].pos		=	
			xv_data_d[ARM_PITCH_R].pos		=	xp_mv_ready.arm_sh_pitch;
			xv_data_d[ARM_ROLL_L].pos		=	-xp_mv_ready.arm_sh_roll;
			xv_data_d[ARM_ROLL_R].pos		=	xp_mv_ready.arm_sh_roll;
			xv_data_d[ELBOW_PITCH_L].pos    =   xp_mv_ready.arm_el_pitch;
			xv_data_d[ELBOW_PITCH_R].pos    =   xp_mv_ready.arm_el_pitch;

			mode_sq_walk	=	SQ_WALK_END;
			mode_sq_time	=	0.f;

			break;

		case	SQ_WALK_END:		/*	end	*/
			flag_gyro.vib		=	ON;		//	normal gain

			sq_flag.walk		=	OFF;
			flag_md_walk_end 	= 	ON;
			mode_sq_walk		=	SQ_WALK_INIT;
			mode_sq_time		=	0.f;

			xv_mv.count			=	0;
			xv_mv_walk.num		=	xp_mv_walk.num;
			xv_data_pitch.pos	=	xp_mv_ready.pitch;
            
            accurate_one_step_mode = 0;
			break;

		default:
			break;
	}

	mode_sq_time	+=	RTC_TIME_SEC;							/*	count up state time		*/

	/***	store last leg for next start	***/
	if( flag_walk.upleg == RIGHT || flag_walk.upleg == LEFT )
		flag_walk.upleg_last	=	flag_walk.upleg;
}

