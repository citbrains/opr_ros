/*----------------------------------------------------------*/
/*	motion sequence											*/
/*															*/
/*															*/
/*	file name	:	motion.c								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#define		_MOTION_C_

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include    <stdio.h>
#include	<string.h>
#include	"var.h"
#include	"motion.h"
#include	"sq_start.h"
#include	"sq_straight.h"
#include	"sq_ready.h"
#include	"sq_walk.h"
#include	"sq_motion.h"
#include    "servo_rs.h"
#include    "b3m.h"

/*--------------------------------------*/
/*	motion								*/
/*	���[�V�����̏�Ԃɂ�镪��			*/
/*--------------------------------------*/
void motion()
{
	static int relax_count_down = RTC_TICK;
	static int mode_motion_from = MOTION_NONE;	
	int is_first = (mode_motion != mode_motion_from);
    
	// �t���O���烂�[�V������I������D�D�揇�ʂ͈ȉ��̒ʂ�D
	if (mode_motion == MOTION_NONE){
		if      (sq_flag.start   ) mode_motion = MOTION_START   ;
		else if (sq_flag.straight) mode_motion = MOTION_STRAIGHT;
		else if (sq_flag.ready   ) mode_motion = MOTION_READY   ;
		else if (sq_flag.walk    ){
			if ((mode_motion_from != MOTION_WALK)&&(mode_motion_from != MOTION_READY)){
				mode_motion = MOTION_READY;
			} else {
				mode_motion = MOTION_WALK ;
			}
		} else if (sq_flag.motion  ){
			if ((mode_motion_from != MOTION_WALK)&&(mode_motion_from != MOTION_READY)){
				mode_motion = MOTION_READY ;
			} else {
				mode_motion = MOTION_MOTION;
			}
		}
	}
   
	switch( mode_motion )
	{
		case	MOTION_START:		/* servo on */
			mode_motion_from	=	mode_motion;
			flag_moving			=	STATE_MOVING;
			flag_servo_output_wait	= 1;				// �T�[�{�̏o�͂��K������

			if (is_first){								// ���[�h���؂�ւ�������̏���
				sq_start_init();
			} else if(sq_start()){						// �T�[�{ON�C�N�����̏���
				flag_servo_on		=	ON;				// �����I����̏���
				mode_motion			=	MOTION_STRAIGHT;
			}
			break;

		case	MOTION_STRAIGHT:	/* straight position */
			mode_motion_from	=	mode_motion;
			flag_moving			=	STATE_MOVING;

			if (is_first){
				sq_straight_init();
			} else if (sq_straight()){
				flag_moving		=	STATE_STOP;
				mode_motion		=	MOTION_NONE;
			}
			break;

		case	MOTION_READY:		/* ready position */
			if (is_first){
				int slow_mode = (mode_motion_from == MOTION_STRAIGHT);
				sq_ready_init(slow_mode);		// ������Ԃ��烌�f�B�ɂȂ�Ƃ��͂������
			} else if (sq_ready()){
				flag_moving		=	STATE_STOP;
				mode_motion		=	MOTION_NONE;
			}

			mode_motion_from	=	MOTION_READY;
			flag_moving			=	STATE_MOVING;
			break;

		case	MOTION_WALK:		/* walk */
			mode_motion_from	=	mode_motion;
			flag_moving			=	STATE_WALKING;
			if (is_first){
//				sq_walk_init();
			} if (sq_walk() && (sq_flag.walk == OFF)){
				mode_motion		=	MOTION_NONE;
			}
			break;

		case	MOTION_MOTION:	/*	motion (special action)	*/
			mode_motion_from	=	mode_motion;
			flag_moving			=	STATE_MOTION;

#if 0
			if(motion_first){
				motion_first = false;
				motion_gain_change(1);
			}
#endif
			if (sq_motion()){
				//motion_gain_change(0);
				//motion_first = true;
				flag_moving		=	STATE_STOP;
				mode_motion		=	MOTION_READY;
			}
			break;

		default:
			flag_moving			=	STATE_STOP;
			break;
	}
}

// ��Ԃ̃t���O�����Z�b�g����֐�
void reset_flag(){
	memset(&sq_flag, 0, sizeof(sq_flag));
}
