/*----------------------------------------------------------*/
/*	control													*/
/*															*/
/*															*/
/*	file name	:	cntr.c									*/
/*	target		:											*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2003.1.8								*/
/*----------------------------------------------------------*/
#define		_CNTR_C_

/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/
#include	"var.h"
#include	"func.h"
#include	"acc.h"
#include	"gyro.h"
#include	"joy.h"
#include	"motion.h"
#include	"calc_mv.h"
#include	"kine.h"
#include	"calc_deg.h"
#include	"serv.h"
#include 	"servo_rs.h"
#include	"b3m.h"
#include	"sq_walk.h"
#include	"mvtbl.h"
#include	"sq_straight.h"
#include	"sq_motion.h"


/*--------------------------------------*/
/*	control								*/
/*--------------------------------------*/
void	cntr( void )
{
	joy();					/*	command receive						*/
	motion();				/*	motion sequense						*/
	calc_mv();				/*	calculate trajectory tables			*/
	calc_z();				/*	calculate leg length by hip roll 	*/
	kine();					/*	calculate inv. kinetics				*/
	acc_fun();				/*	acceleration sensor					*/
	gyro_fun();				/*	gyro sensor							*/
	calc_deg();				/*	calculate joint angle by inv. kinetics data	*/
	serv();					/*	servo motor control					*/

	//servo_rs_fun();			/*	output to servo motors				*/
}
