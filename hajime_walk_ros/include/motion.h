/*----------------------------------------------------------*/
/*	motion sequence											*/
/*															*/
/*		header program										*/
/*															*/
/*	file name	:	motion.h								*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#ifndef 	_MOTION_H_
#define 	_MOTION_H_

#ifdef 		_MOTION_C_
#define 	extern 
#endif 		/* _MOTION_C_ */

/*--------------------------------------*/
/*	typedef								*/
/*--------------------------------------*/
// ���
enum {
	MOTION_NONE,		// �������Ȃ����
	MOTION_START,		// �T�[�{�I��
	MOTION_STRAIGHT,	// �����p��
	MOTION_READY,		// ���f�B�p��
	MOTION_WALK,		// ���s
	MOTION_MOTION,		// ���[�V�����Đ�
};

enum{
	STATE_STOP,
	STATE_WALKING,
	STATE_MOTION,	
	STATE_MOVING		// stand, ready
};

/*--------------------------------------*/
/*	variables							*/
/*--------------------------------------*/
extern short			mode_motion;
extern short			flag_moving;		// 0:STOP,1:WALKING,2:MOTION,3:MOVING?

// �V�[�P���X�̃t���O�i�ォ�珇�ԂɎ��s�����j
extern struct sq_flag_T{
	int start	;	// �T�[�{ON�̃V�[�P���X
	int straight;	// �����̃V�[�P���X
	int ready	;	// ���f�B��Ԃւ̃V�[�P���X
	int walk	;	// ���s�̃V�[�P���X
	int motion	;	// ���[�V�����̃V�[�P���X
} sq_flag;

/*--------------------------------------*/
/*	function prototypes					*/
/*--------------------------------------*/
void motion();
void reset_flag();

#ifdef 		_MOTION_C_
#undef 		extern
#endif 		/* _MOTION_C_ */

#endif 		/* _MOTION_H_ */
