/*----------------------------------------------------------*/
/*  HAJIME WALK CONTROL ROS                                 */
/*  main program                                            */
/*                                                          */
/*  file name   :   main.cpp                                */
/*  compiler    :   gcc                                     */
/*  authors     :   Hajime Sakamoto, Joshua Supratman       */
/*  date        :   2007.7.15                               */
/*  note        :   editor tab = 4                          */
/*                                                          */
/*  memo        :   南方，土橋がWindowsからの歩行パターンの */
/*                  生成のために変更中                      */
/*  date        :   2011.12.29                              */
/*                  2012.01.14  一定周期で制御するために追加*/
/*                  2019.12.16 modified to ROS              */
/*----------------------------------------------------------*/


/*--------------------------------------*/
/*  include                             */
/*--------------------------------------*/

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include "hajime_walk_ros/WalkCommand.h"
#include "hajime_walk_ros/MotionCommand.h"

#include    <stdio.h>
#include    <assert.h>
#include    <stdio.h>
#include    <math.h>
#include    <boost/thread.hpp>
#include    <string>

#include "pc_motion.h"

extern "C" {
#include    "var.h"
#include    "func.h"
#include    "servo_rs.h"
#include    "sq_walk.h"
#include    "cntr.h"
#include    "kine.h"
#include    "serv.h"
#include    "serv_init.h"
#include    "calc_mv.h"
#include    "mvtbl.h"
#include    "sq_motion.h"
#include    "sq_start.h"
#include    "sq_straight.h"
#include    "sq_ready.h"
#include    "sq_walk.h"
#include    "motion.h"
#include    "joy.h"

#include    "acc.h"
#include    "gyro.h"
#include    "b3m.h"
}

#define PARAM_TABLE_OFFSET 26

using namespace std;
using namespace boost;

static mutex lock_obj;
static string cmd;
static bool response_ready = false;
static string res;
unsigned char walk_cmd  = 'C';
unsigned char num_step  = '0';
unsigned char period    = '0';
unsigned char stride_x  = '0';
unsigned char stride_y  = '0';
unsigned char stride_th = '0';
bool motion_flag = false;

char ParamTable[53] =
{
	'z','y','x','w','v','u','t','s','r','q','p','o','n','m','l','k','j','i','h','g','f','e','d','c','b','a', // -26 - -1
	'0',                                                                                                     // 0
	'1','2','3','4','5','6','7','8','9','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'  // 1 - 26
};

extern "C"

int scif1_tx_fun()
{
    mutex::scoped_lock look(lock_obj);

    int len = 0;
    for(int i = 0; i < SFMT_SIZE-1 && sfmt[i] != EOF_CODE && sfmt[i] != EOF_CODE3; i++ )
    {
        len++;
    }
    res = string(sfmt, len);
    memset(sfmt, EOF_CODE, SFMT_SIZE);
    response_ready = true;

    return 1;
}


extern "C"
int servo_offset[SERV_NUM]; // オフセット保存用

void load_eeprom()
{
    ros::NodeHandle private_nh("~");
    if(private_nh.getParam("odometry_correct_para_x", odometry_correct_para_x)){
        ROS_INFO("odometry_correct_para_x: %f", odometry_correct_para_x);
    } else{
        ROS_ERROR("failed to get odometry_correct_para_x");
    }
    if(private_nh.getParam("odometry_correct_para_y", odometry_correct_para_y)){
        ROS_INFO("odometry_correct_para_y: %f", odometry_correct_para_y);
    } else{
        ROS_ERROR("failed to get odometry_correct_para_y");
    }

    ros::NodeHandle private_nh_xp_acc(std::string("~")+"xp_acc");
    if(private_nh_xp_acc.getParam("acc_k1", xp_acc.acc_k1)){
        ROS_INFO("xp_acc.acc_k1: %f", xp_acc.acc_k1);
    } else{
        ROS_ERROR("failed to get xp_acc.acc_k1");
    }
    if(private_nh_xp_acc.getParam("acc_k2", xp_acc.acc_k2)){
        ROS_INFO("xp_acc.acc_k2: %f", xp_acc.acc_k2);
    } else{
        ROS_ERROR("failed to get xp_acc.acc_k2");
    }
    if(private_nh_xp_acc.getParam("acc_k3", xp_acc.acc_k3)){
        ROS_INFO("xp_acc.acc_k3: %f", xp_acc.acc_k3);
    } else{
        ROS_ERROR("failed to get xp_acc.acc_k3");
    }
    if(private_nh_xp_acc.getParam("ad_volt_offset1", xp_acc.ad_volt_offset1)){
        ROS_INFO("xp_acc.ad_volt_offset1: %f", xp_acc.ad_volt_offset1);
    } else{
        ROS_ERROR("failed to get xp_acc.ad_volt_offset1");
    }
    if(private_nh_xp_acc.getParam("ad_volt_offset2", xp_acc.ad_volt_offset2)){
        ROS_INFO("xp_acc.ad_volt_offset2: %f", xp_acc.ad_volt_offset2);
    } else{
        ROS_ERROR("failed to get xp_acc.ad_volt_offset2");
    }
    if(private_nh_xp_acc.getParam("ad_volt_offset3", xp_acc.ad_volt_offset3)){
        ROS_INFO("xp_acc.ad_volt_offset3: %f", xp_acc.ad_volt_offset3);
    } else{
        ROS_ERROR("failed to get xp_acc.ad_volt_offset3");
    }
    if(private_nh_xp_acc.getParam("t1", xp_acc.t1)){
        ROS_INFO("xp_acc.t1: %f", xp_acc.t1);
    } else{
        ROS_ERROR("failed to get xp_acc.t1");
    }
    if(private_nh_xp_acc.getParam("t2", xp_acc.t2)){
        ROS_INFO("xp_acc.t2: %f", xp_acc.t2);
    } else{
        ROS_ERROR("failed to get xp_acc.t2");
    }
    if(private_nh_xp_acc.getParam("fall_fwd", xp_acc.fall_fwd)){
        ROS_INFO("xp_acc.fall_fwd: %f", xp_acc.fall_fwd);
    } else{
        ROS_ERROR("failed to get xp_acc.fall_fwd");
    }
    if(private_nh_xp_acc.getParam("fall_bwd", xp_acc.fall_bwd)){
        ROS_INFO("xp_acc.fall_bwd: %f", xp_acc.fall_bwd);
    } else{
        ROS_ERROR("failed to get xp_acc.fall_bwd");
    }
    if(private_nh_xp_acc.getParam("fall_right", xp_acc.fall_right)){
        ROS_INFO("xp_acc.fall_right: %f", xp_acc.fall_right);
    } else{
        ROS_ERROR("failed to get xp_acc.fall_right");
    }
    if(private_nh_xp_acc.getParam("fall_left", xp_acc.fall_left)){
        ROS_INFO("xp_acc.fall_left: %f", xp_acc.fall_left);
    } else{
        ROS_ERROR("failed to get xp_acc.fall_left");
    }
    if(private_nh_xp_acc.getParam("fall_check_time", xp_acc.fall_check_time)){
        ROS_INFO("xp_acc.fall_check_time: %f", xp_acc.fall_check_time);
    } else{
        ROS_ERROR("failed to get xp_acc.fall_check_time");
    }
    if(private_nh_xp_acc.getParam("fall_pitch", xp_acc.fall_pitch)){
        ROS_INFO("xp_acc.fall_pitch: %f", xp_acc.fall_pitch);
    } else{
        ROS_ERROR("failed to get xp_acc.fall_pitch");
    }
    if(private_nh_xp_acc.getParam("fall_roll", xp_acc.fall_roll)){
        ROS_INFO("xp_acc.fall_roll: %f", xp_acc.fall_roll);
    } else{
        ROS_ERROR("failed to get xp_acc.fall_roll");
    }
    if(private_nh_xp_acc.getParam("fall_pitch_oblique", xp_acc.fall_pitch_oblique)){
        ROS_INFO("xp_acc.fall_pitch_oblique: %f", xp_acc.fall_pitch_oblique);
    } else{
        ROS_ERROR("failed to get xp_acc.fall_pitch_oblique");
    }
    if(private_nh_xp_acc.getParam("fall_roll_oblique", xp_acc.fall_roll_oblique)){
        ROS_INFO("xp_acc.fall_roll_oblique: %f", xp_acc.fall_roll_oblique);
    } else{
        ROS_ERROR("failed to get xp_acc.fall_roll_oblique");
    }

    ros::NodeHandle private_nh_xp_gyro(std::string("~")+"xp_gyro");
    if(private_nh_xp_gyro.getParam("kp1_foot", xp_gyro.kp1_foot)){
        ROS_INFO("xp_gyro.kp1_foot: %f", xp_gyro.kp1_foot);
    } else{
        ROS_ERROR("failed to get xp_gyro.kp1_foot");
    }
    if(private_nh_xp_gyro.getParam("kp2_foot", xp_gyro.kp2_foot)){
        ROS_INFO("xp_gyro.kp2_foot: %f", xp_gyro.kp2_foot);
    } else{
        ROS_ERROR("failed to get xp_gyro.kp2_foot");
    }
    if(private_nh_xp_gyro.getParam("kp1_hip", xp_gyro.kp1_hip)){
        ROS_INFO("xp_gyro.kp1_hip: %f", xp_gyro.kp1_hip);
    } else{
        ROS_ERROR("failed to get xp_gyro.kp1_hip");
    }
    if(private_nh_xp_gyro.getParam("kp2_hip", xp_gyro.kp2_hip)){
        ROS_INFO("xp_gyro.kp2_hip: %f", xp_gyro.kp2_hip);
    } else{
        ROS_ERROR("failed to get xp_gyro.kp2_hip");
    }
    if(private_nh_xp_gyro.getParam("kp1_arm", xp_gyro.kp1_arm)){
        ROS_INFO("xp_gyro.kp1_arm: %f", xp_gyro.kp1_arm);
    } else{
        ROS_ERROR("failed to get xp_gyro.kp1_arm");
    }
    if(private_nh_xp_gyro.getParam("kp2_arm", xp_gyro.kp2_arm)){
        ROS_INFO("xp_gyro.kp2_arm: %f", xp_gyro.kp2_arm);
    } else{
        ROS_ERROR("failed to get xp_gyro.kp2_arm");
    }
    if(private_nh_xp_gyro.getParam("kp2_waist", xp_gyro.kp2_waist)){
        ROS_INFO("xp_gyro.kp2_waist: %f", xp_gyro.kp2_waist);
    } else{
        ROS_ERROR("failed to get xp_gyro.kp2_waist");
    }
    if(private_nh_xp_gyro.getParam("kp3_waist", xp_gyro.kp3_waist)){
        ROS_INFO("xp_gyro.kp3_waist: %f", xp_gyro.kp3_waist);
    } else{
        ROS_ERROR("failed to get xp_gyro.kp3_waist");
    }
    if(private_nh_xp_gyro.getParam("gyro_k1", xp_gyro.gyro_k1)){
        ROS_INFO("xp_gyro.gyro_k1: %f", xp_gyro.gyro_k1);
    } else{
        ROS_ERROR("failed to get xp_gyro.gyro_k1");
    }
    if(private_nh_xp_gyro.getParam("gyro_k2", xp_gyro.gyro_k2)){
        ROS_INFO("xp_gyro.gyro_k2: %f", xp_gyro.gyro_k2);
    } else{
        ROS_ERROR("failed to get xp_gyro.gyro_k2");
    }
    if(private_nh_xp_gyro.getParam("gyro_k3", xp_gyro.gyro_k3)){
        ROS_INFO("xp_gyro.gyro_k3: %f", xp_gyro.gyro_k3);
    } else{
        ROS_ERROR("failed to get xp_gyro.gyro_k3");
    }
    if(private_nh_xp_gyro.getParam("ad_volt_offset1", xp_gyro.ad_volt_offset1)){
        ROS_INFO("xp_gyro.ad_volt_offset1: %f", xp_gyro.ad_volt_offset1);
    } else{
        ROS_ERROR("failed to get xp_gyro.ad_volt_offset1");
    }
    if(private_nh_xp_gyro.getParam("ad_volt_offset2", xp_gyro.ad_volt_offset2)){
        ROS_INFO("xp_gyro.ad_volt_offset2: %f", xp_gyro.ad_volt_offset2);
    } else{
        ROS_ERROR("failed to get xp_gyro.ad_volt_offset2");
    }
    if(private_nh_xp_gyro.getParam("ad_volt_offset3", xp_gyro.ad_volt_offset3)){
        ROS_INFO("xp_gyro.ad_volt_offset3: %f", xp_gyro.ad_volt_offset3);
    } else{
        ROS_ERROR("failed to get xp_gyro.ad_volt_offset3");
    }
    if(private_nh_xp_gyro.getParam("t1", xp_gyro.t1)){
        ROS_INFO("xp_gyro.t1: %f", xp_gyro.t1);
    } else{
        ROS_ERROR("failed to get xp_gyro.t1");
    }
    if(private_nh_xp_gyro.getParam("t2", xp_gyro.t2)){
        ROS_INFO("xp_gyro.t2: %f", xp_gyro.t2);
    } else{
        ROS_ERROR("failed to get xp_gyro.t2");
    }
    if(private_nh_xp_gyro.getParam("gyro_data3_flt2_t1", xp_gyro.gyro_data3_flt2_t1)){
        ROS_INFO("xp_gyro.gyro_data3_flt2_t1: %f", xp_gyro.gyro_data3_flt2_t1);
    } else{
        ROS_ERROR("failed to get xp_gyro.gyro_data3_flt2_t1");
    }
    if(private_nh_xp_gyro.getParam("yaw_cntl_gain", xp_gyro.yaw_cntl_gain)){
        ROS_INFO("xp_gyro.yaw_cntl_gain: %f", xp_gyro.yaw_cntl_gain);
    } else{
        ROS_ERROR("failed to get xp_gyro.yaw_cntl_gain");
    }
    if(private_nh_xp_gyro.getParam("yaw_cntl_dead", xp_gyro.yaw_cntl_dead)){
        ROS_INFO("xp_gyro.yaw_cntl_dead: %f", xp_gyro.yaw_cntl_dead);
    } else{
        ROS_ERROR("failed to get xp_gyro.yaw_cntl_dead");
    }
    if(private_nh_xp_gyro.getParam("yaw_cntl_theta", xp_gyro.yaw_cntl_theta)){
        ROS_INFO("xp_gyro.yaw_cntl_theta: %f", xp_gyro.yaw_cntl_theta);
    } else{
        ROS_ERROR("failed to get xp_gyro.yaw_cntl_theta");
    }
    if(private_nh_xp_gyro.getParam("gyro_omega", xp_gyro.gyro_omega)){
        ROS_INFO("xp_gyro.gyro_omega: %f", xp_gyro.gyro_omega);
    } else{
        ROS_ERROR("failed to get xp_gyro.gyro_omega");
    }
    if(private_nh_xp_gyro.getParam("fall_roll_deg1", xp_gyro.fall_roll_deg1)){
        ROS_INFO("xp_gyro.fall_roll_deg1: %f", xp_gyro.fall_roll_deg1);
    } else{
        ROS_ERROR("failed to get xp_gyro.fall_roll_deg1");
    }
    if(private_nh_xp_gyro.getParam("fall_pitch_deg1", xp_gyro.fall_pitch_deg1)){
        ROS_INFO("xp_gyro.fall_pitch_deg1: %f", xp_gyro.fall_pitch_deg1);
    } else{
        ROS_ERROR("failed to get xp_gyro.fall_pitch_deg1");
    }

    ros::NodeHandle private_nh_flag_gyro(std::string("~")+"flag_gyro");
    float fall_cntl;
    if(private_nh_flag_gyro.getParam("fall_cntl", fall_cntl)){
        flag_gyro.fall_cntl = (short)fall_cntl;
        ROS_INFO("flag_gyro.fall_cntl: %d", flag_gyro.fall_cntl);
    } else{
        ROS_ERROR("failed to get flag_gyro.fall_cntl");
    }
    float zero;
    if(private_nh_flag_gyro.getParam("zero", zero)){
        flag_gyro.zero = (short)zero;
        ROS_INFO("flag_gyro.zero: %d", flag_gyro.zero);
    } else{
        ROS_ERROR("failed to get flag_gyro.zero");
    }

    ros::NodeHandle private_nh_xp_mv_straight(std::string("~")+"xp_mv_straight");
    if(private_nh_xp_mv_straight.getParam("time", xp_mv_straight.time)){
        ROS_INFO("xp_mv_straight.time: %f", xp_mv_straight.time);
    } else{
        ROS_ERROR("failed to get xp_mv_straight.time");
    }
    if(private_nh_xp_mv_straight.getParam("z3", xp_mv_straight.z3)){
        ROS_INFO("xp_mv_straight.z3: %f", xp_mv_straight.z3);
    } else{
        ROS_ERROR("failed to get xp_mv_straight.z3");
    }
    if(private_nh_xp_mv_straight.getParam("arm_sh_pitch", xp_mv_straight.arm_sh_pitch)){
        ROS_INFO("xp_mv_straight.arm_sh_pitch: %f", xp_mv_straight.arm_sh_pitch);
    } else{
        ROS_ERROR("failed to get xp_mv_straight.arm_sh_pitch");
    }
    if(private_nh_xp_mv_straight.getParam("arm_sh_roll", xp_mv_straight.arm_sh_roll)){
        ROS_INFO("xp_mv_straight.arm_sh_roll: %f", xp_mv_straight.arm_sh_roll);
    } else{
        ROS_ERROR("failed to get xp_mv_straight.arm_sh_roll");
    }
    if(private_nh_xp_mv_straight.getParam("arm_el_yaw", xp_mv_straight.arm_el_yaw)){
        ROS_INFO("xp_mv_straight.arm_el_yaw: %f", xp_mv_straight.arm_el_yaw);
    } else{
        ROS_ERROR("failed to get xp_mv_straight.arm_el_yaw");
    }
    if(private_nh_xp_mv_straight.getParam("arm_el_pitch", xp_mv_straight.arm_el_pitch)){
        ROS_INFO("xp_mv_straight.arm_el_pitch: %f", xp_mv_straight.arm_el_pitch);
    } else{
        ROS_ERROR("failed to get xp_mv_straight.arm_el_pitch");
    }
            
    ros::NodeHandle private_nh_xp_mv_ready(std::string("~")+"xp_mv_ready");
    if(private_nh_xp_mv_ready.getParam("time", xp_mv_ready.time)){
        ROS_INFO("xp_mv_ready.time: %f", xp_mv_ready.time);
    } else{
        ROS_ERROR("failed to get xp_mv_ready.time");
    }
    if(private_nh_xp_mv_ready.getParam("z3", xp_mv_ready.z3)){
        ROS_INFO("xp_mv_ready.z3: %f", xp_mv_ready.z3);
    } else{
        ROS_ERROR("failed to get xp_mv_ready.z3");
    }
    if(private_nh_xp_mv_ready.getParam("arm_sh_pitch", xp_mv_ready.arm_sh_pitch)){
        ROS_INFO("xp_mv_ready.arm_sh_pitch: %f", xp_mv_ready.arm_sh_pitch);
    } else{
        ROS_ERROR("failed to get xp_mv_ready.arm_sh_pitch");
    }
    if(private_nh_xp_mv_ready.getParam("arm_sh_roll", xp_mv_ready.arm_sh_roll)){
        ROS_INFO("xp_mv_ready.arm_sh_roll: %f", xp_mv_ready.arm_sh_roll);
    } else{
        ROS_ERROR("failed to get xp_mv_ready.arm_sh_roll");
    }
    if(private_nh_xp_mv_ready.getParam("arm_el_yaw", xp_mv_ready.arm_el_yaw)){
        ROS_INFO("xp_mv_ready.arm_el_yaw: %f", xp_mv_ready.arm_el_yaw);
    } else{
        ROS_ERROR("failed to get xp_mv_ready.arm_el_yaw");
    }
    if(private_nh_xp_mv_ready.getParam("arm_el_pitch", xp_mv_ready.arm_el_pitch)){
        ROS_INFO("xp_mv_ready.arm_el_pitch: %f", xp_mv_ready.arm_el_pitch);
    } else{
        ROS_ERROR("failed to get xp_mv_ready.arm_el_pitch");
    }
    if(private_nh_xp_mv_ready.getParam("pitch", xp_mv_ready.pitch)){
        ROS_INFO("xp_mv_ready.pitch: %f", xp_mv_ready.pitch);
    } else{
        ROS_ERROR("failed to get xp_mv_ready.pitch");
    }

    ros::NodeHandle private_nh_xp_mv_walk(std::string("~")+"xp_mv_walk");
    float num;
    if(private_nh_xp_mv_walk.getParam("num", num)){
        xp_mv_walk.num = (long)num;
        ROS_INFO("xp_mv_walk.num: %ld", xp_mv_walk.num);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.num");
    }
    if(private_nh_xp_mv_walk.getParam("h_cog", xp_mv_walk.h_cog)){
        ROS_INFO("xp_mv_walk.h_cog: %f", xp_mv_walk.h_cog);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.h_cog");
    }
    if(private_nh_xp_mv_walk.getParam("time", xp_mv_walk.time)){
        ROS_INFO("xp_mv_walk.time: %f", xp_mv_walk.time);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.time");
    }
    if(private_nh_xp_mv_walk.getParam("x_fwd_swg", xp_mv_walk.x_fwd_swg)){
        ROS_INFO("xp_mv_walk.x_fwd_swg: %f", xp_mv_walk.x_fwd_swg);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.x_fwd_swg");
    }
    if(private_nh_xp_mv_walk.getParam("x_fwd_spt", xp_mv_walk.x_fwd_spt)){
        ROS_INFO("xp_mv_walk.x_fwd_spt: %f", xp_mv_walk.x_fwd_spt);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.x_fwd_spt");
    }
    if(private_nh_xp_mv_walk.getParam("x_bwd_swg", xp_mv_walk.x_bwd_swg)){
        ROS_INFO("xp_mv_walk.x_bwd_swg: %f", xp_mv_walk.x_bwd_swg);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.x_bwd_swg");
    }
    if(private_nh_xp_mv_walk.getParam("x_bwd_spt", xp_mv_walk.x_bwd_spt)){
        ROS_INFO("xp_mv_walk.x_bwd_spt: %f", xp_mv_walk.x_bwd_spt);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.x_bwd_spt");
    }
    if(private_nh_xp_mv_walk.getParam("y_swg", xp_mv_walk.y_swg)){
        ROS_INFO("xp_mv_walk.y_swg: %f", xp_mv_walk.y_swg);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.y_swg");
    }
    if(private_nh_xp_mv_walk.getParam("y_spt", xp_mv_walk.y_spt)){
        ROS_INFO("xp_mv_walk.y_spt: %f", xp_mv_walk.y_spt);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.y_spt");
    }
    if(private_nh_xp_mv_walk.getParam("theta", xp_mv_walk.theta)){
        ROS_INFO("xp_mv_walk.theta: %f", xp_mv_walk.theta);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.theta");
    }
    if(private_nh_xp_mv_walk.getParam("z", xp_mv_walk.z)){
        ROS_INFO("xp_mv_walk.z: %f", xp_mv_walk.z);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.z");
    }
    if(private_nh_xp_mv_walk.getParam("y_balance", xp_mv_walk.y_balance)){
        ROS_INFO("xp_mv_walk.y_balance: %f", xp_mv_walk.y_balance);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.y_balance");
    }
    if(private_nh_xp_mv_walk.getParam("hip_roll", xp_mv_walk.hip_roll)){
        ROS_INFO("xp_mv_walk.hip_roll: %f", xp_mv_walk.hip_roll);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.hip_roll");
    }
    if(private_nh_xp_mv_walk.getParam("x_fwd_pitch", xp_mv_walk.x_fwd_pitch)){
        ROS_INFO("xp_mv_walk.x_fwd_pitch: %f", xp_mv_walk.x_fwd_pitch);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.x_fwd_pitch");
    }
    if(private_nh_xp_mv_walk.getParam("x_bwd_pitch", xp_mv_walk.x_bwd_pitch)){
        ROS_INFO("xp_mv_walk.x_bwd_pitch: %f", xp_mv_walk.x_bwd_pitch);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.x_bwd_pitch");
    }
    if(private_nh_xp_mv_walk.getParam("arm_sh_pitch", xp_mv_walk.arm_sh_pitch)){
        ROS_INFO("xp_mv_walk.arm_sh_pitch: %f", xp_mv_walk.arm_sh_pitch);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.arm_sh_pitch");
    }
    if(private_nh_xp_mv_walk.getParam("start_time_k1", xp_mv_walk.start_time_k1)){
        ROS_INFO("xp_mv_walk.start_time_k1: %f", xp_mv_walk.start_time_k1);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.start_time_k1");
    }
    if(private_nh_xp_mv_walk.getParam("start_zmp_k1", xp_mv_walk.start_zmp_k1)){
        ROS_INFO("xp_mv_walk.start_zmp_k1: %f", xp_mv_walk.start_zmp_k1);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.start_zmp_k1");
    }
    //if(private_nh_xp_mv_walk.getParam("start_time_k2", xp_mv_walk.start_time_k2)){
    //    ROS_INFO("xp_mv_walk.start_time_k2: %f", xp_mv_walk.start_time_k2);
    //} else{
    //    ROS_ERROR("failed to get xp_mv_walk.start_time_k2");
    //}
    if(private_nh_xp_mv_walk.getParam("foot_cntl_p", xp_mv_walk.foot_cntl_p)){
        ROS_INFO("xp_mv_walk.foot_cntl_p: %f", xp_mv_walk.foot_cntl_p);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.foot_cntl_p");
    }
    if(private_nh_xp_mv_walk.getParam("foot_cntl_r", xp_mv_walk.foot_cntl_r)){
        ROS_INFO("xp_mv_walk.foot_cntl_r: %f", xp_mv_walk.foot_cntl_r);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.foot_cntl_r");
    }
    if(private_nh_xp_mv_walk.getParam("sidestep_time_k", xp_mv_walk.sidestep_time_k)){
        ROS_INFO("xp_mv_walk.sidestep_time_k: %f", xp_mv_walk.sidestep_time_k);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.sidestep_time_k");
    }
    if(private_nh_xp_mv_walk.getParam("sidestep_roll", xp_mv_walk.sidestep_roll)){
        ROS_INFO("xp_mv_walk.sidestep_roll: %f", xp_mv_walk.sidestep_roll);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.sidestep_roll");
    }
    if(private_nh_xp_mv_walk.getParam("y_wide", xp_mv_walk.y_wide)){
        ROS_INFO("xp_mv_walk.y_wide: %f", xp_mv_walk.y_wide);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.y_wide");
    }
    if(private_nh_xp_mv_walk.getParam("time_dutyfactor", xp_mv_walk.time_dutyfactor)){
        ROS_INFO("xp_mv_walk.time_dutyfactor: %f", xp_mv_walk.time_dutyfactor);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.time_dutyfactor");
    }
    if(private_nh_xp_mv_walk.getParam("x_fwd_acc_pitch", xp_mv_walk.x_fwd_acc_pitch)){
        ROS_INFO("xp_mv_walk.x_fwd_acc_pitch: %f", xp_mv_walk.x_fwd_acc_pitch);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.x_fwd_acc_pitch");
    }
    if(private_nh_xp_mv_walk.getParam("x_bwd_acc_pitch", xp_mv_walk.x_bwd_acc_pitch)){
        ROS_INFO("xp_mv_walk.x_bwd_acc_pitch: %f", xp_mv_walk.x_bwd_acc_pitch);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.x_bwd_acc_pitch");
    }
    if(private_nh_xp_mv_walk.getParam("accurate_x_percent_dlim", xp_mv_walk.accurate_x_percent_dlim)){
        ROS_INFO("xp_mv_walk.accurate_x_percent_dlim: %f", xp_mv_walk.accurate_x_percent_dlim);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.accurate_x_percent_dlim");
    }
    if(private_nh_xp_mv_walk.getParam("accurate_y_percent_dlim", xp_mv_walk.accurate_y_percent_dlim)){
        ROS_INFO("xp_mv_walk.accurate_y_percent_dlim: %f", xp_mv_walk.accurate_y_percent_dlim);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.accurate_y_percent_dlim");
    }
    if(private_nh_xp_mv_walk.getParam("accurate_th_percent_dlim", xp_mv_walk.accurate_th_percent_dlim)){
        ROS_INFO("xp_mv_walk.accurate_th_percent_dlim: %f", xp_mv_walk.accurate_th_percent_dlim);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.accurate_th_percent_dlim");
    }
    if(private_nh_xp_mv_walk.getParam("arm_el_pitch", xp_mv_walk.arm_el_pitch)){
        ROS_INFO("xp_mv_walk.arm_el_pitch: %f", xp_mv_walk.arm_el_pitch);
    } else{
        ROS_ERROR("failed to get xp_mv_walk.arm_el_pitch");
    }
   
    ros::NodeHandle private_nh_xp_dlim_wait_x(std::string("~")+"xp_dlim_wait_x");
    if(private_nh_xp_dlim_wait_x.getParam("dlim", xp_dlim_wait_x.dlim)){
        ROS_INFO("xp_dlim_wait_x.dlim: %f", xp_dlim_wait_x.dlim);
    } else{
        ROS_ERROR("failed to get xp_dlim_wait_x.dlim");
    }
    if(private_nh_xp_dlim_wait_x.getParam("wait_time", xp_dlim_wait_x.wait_time)){
        ROS_INFO("xp_dlim_wait_x.wait_time: %f", xp_dlim_wait_x.wait_time);
    } else{
        ROS_ERROR("failed to get xp_dlim_wait_x.wait_time");
    }

    ros::NodeHandle private_nh_xp_dlim_wait_y(std::string("~")+"xp_dlim_wait_y");
    if(private_nh_xp_dlim_wait_y.getParam("dlim", xp_dlim_wait_y.dlim)){
        ROS_INFO("xp_dlim_wait_y.dlim: %f", xp_dlim_wait_y.dlim);
    } else{
        ROS_ERROR("failed to get xp_dlim_wait_y.dlim");
    }
    if(private_nh_xp_dlim_wait_y.getParam("wait_time", xp_dlim_wait_y.wait_time)){
        ROS_INFO("xp_dlim_wait_y.wait_time: %f", xp_dlim_wait_y.wait_time);
    } else{
        ROS_ERROR("failed to get xp_dlim_wait_y.wait_time");
    }

    ros::NodeHandle private_nh_xp_dlim_wait_theta(std::string("~")+"xp_dlim_wait_theta");
    if(private_nh_xp_dlim_wait_theta.getParam("dlim", xp_dlim_wait_theta.dlim)){
        ROS_INFO("xp_dlim_wait_theta.dlim: %f", xp_dlim_wait_theta.dlim);
    } else{
        ROS_ERROR("failed to get xp_dlim_wait_theta.dlim");
    }
    if(private_nh_xp_dlim_wait_theta.getParam("wait_time", xp_dlim_wait_theta.wait_time)){
        ROS_INFO("xp_dlim_wait_theta.wait_time: %f", xp_dlim_wait_theta.wait_time);
    } else{
        ROS_ERROR("failed to get xp_dlim_wait_theta.wait_time");
    }

    // ピッチを変更する比率
    ros::NodeHandle private_nh_xp_dlim_wait_pitch(std::string("~")+"xp_dlim_wait_pitch");
    if(private_nh_xp_dlim_wait_pitch.getParam("dlim", xp_dlim_wait_pitch.dlim)){
        ROS_INFO("xp_dlim_wait_pitch.dlim: %f", xp_dlim_wait_pitch.dlim);
    } else{
        ROS_ERROR("failed to get xp_dlim_wait_pitch.dlim");
    }

}

void motionCallback(const hajime_walk_ros::MotionCommand::ConstPtr &msg)
{
    motion_flag = true;
    std::string s = std::to_string(msg->motion_id);
    if(msg->motion_id < 100){
        s = "0" + s;
    }
    if(msg->motion_id < 10){
        s = "0" + s;
    }
    char const *pchar = s.c_str();

    walk_cmd = 'M';
    char para1  = pchar[0];
    char para2  = pchar[1];
    char para3  = pchar[2];
    char para4  = ParamTable[(int)(msg->num_repeat + PARAM_TABLE_OFFSET)];
    char para5  = 0;

    set_xv_comm(&xv_comm, walk_cmd, para1, para2, para3, para4, para5);
    convert_bin(&xv_comm_bin, &xv_comm);
}

void cancelCallback(const std_msgs::Bool::ConstPtr &msg)
{
    motion_flag = false;
    walk_cmd = 'C';
    num_step  = 0;
    period    = 0;
    stride_x  = 0;
    stride_y  = 0;
    stride_th = 0;
}

void walkCallback(const hajime_walk_ros::WalkCommand::ConstPtr &msg)
{
    motion_flag = false;
    walk_cmd = 'A';
    num_step  = ParamTable[(int)(msg->num_step + PARAM_TABLE_OFFSET)];
    period    = ParamTable[(int)(msg->period + PARAM_TABLE_OFFSET)];
    stride_x  = ParamTable[(int)(msg->stride_x + PARAM_TABLE_OFFSET)];
    stride_y  = ParamTable[(int)(msg->stride_y + PARAM_TABLE_OFFSET)];
    stride_th = ParamTable[(int)(msg->stride_th + PARAM_TABLE_OFFSET)];
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    xv_acc.acc_data1 = msg->linear_acceleration.x; //-imu->getIMUData().accel.y();
    xv_acc.acc_data2 = msg->linear_acceleration.y; //imu->getIMUData().accel.x();
    xv_acc.acc_data3 = msg->linear_acceleration.z; //imu->getIMUData().accel.z();

    const float radian_to_degree = 180.0 / M_PI;
    xv_gyro.gyro_data1 = msg->angular_velocity.x * radian_to_degree; //-imu->getIMUData().gyro.y() * radian_to_degree;
    xv_gyro.gyro_data2 = msg->angular_velocity.y * radian_to_degree; //imu->getIMUData().gyro.x() * radian_to_degree;
    xv_gyro.gyro_data3 = msg->angular_velocity.z * radian_to_degree; //imu->getIMUData().gyro.z() * radian_to_degree;

    xv_gyro.quaternion[0] = msg->orientation.x; //-imuData.fusionQPose.x();
    xv_gyro.quaternion[1] = msg->orientation.y; //imuData.fusionQPose.z();
    xv_gyro.quaternion[2] = msg->orientation.z; //imuData.fusionQPose.scalar();
    xv_gyro.quaternion[3] = msg->orientation.w; //imuData.fusionQPose.y();

    //double pitch_abs = 180.0 - abs(imuData.fusionPose.x()*RTMATH_RAD_TO_DEGREE);
    //if(imuData.fusionPose.x() < 0)
    //    pitch_abs = -pitch_abs;
    xv_gyro.gyro_roll = xv_gyro.gyro_roll2 = roll * radian_to_degree;    //imuData.fusionPose.y()*RTMATH_RAD_TO_DEGREE;
    xv_gyro.gyro_pitch = xv_gyro.gyro_pitch2 = pitch * radian_to_degree; //pitch_abs;
    xv_gyro.gyro_yaw = xv_gyro.gyro_yaw2 = yaw * radian_to_degree;       //-(imuData.fusionPose.z()*RTMATH_RAD_TO_DEGREE);
}

int main( int argc, char *argv[] )
{
    ros::init(argc, argv, "hajime_walk");
    ros::NodeHandle nh;

    ros::Subscriber sub_walk = nh.subscribe("/hajime_walk/walk", 10, walkCallback);
    ros::Subscriber sub_cancel = nh.subscribe("/hajime_walk/cancel", 10, cancelCallback);
    ros::Subscriber sub_motion = nh.subscribe("/hajime_walk/motion", 10, motionCallback);
    ros::Subscriber sub_imu = nh.subscribe("/imu/data", 1000, imuCallback);
    ros::Publisher left_ankle_pitch_pub = nh.advertise<std_msgs::Float64>("/left_ankle_pitch_controller/command", 1);
    ros::Publisher left_ankle_roll_pub = nh.advertise<std_msgs::Float64>("/left_ankle_roll_controller/command", 1);
    ros::Publisher left_elbow_pitch_pub = nh.advertise<std_msgs::Float64>("/left_elbow_pitch_controller/command", 1);
    ros::Publisher left_knee_pitch_pub = nh.advertise<std_msgs::Float64>("/left_knee_pitch_controller/command", 1);
    ros::Publisher left_shoulder_pitch_pub = nh.advertise<std_msgs::Float64>("/left_shoulder_pitch_controller/command", 1);
    ros::Publisher left_shoulder_roll_pub = nh.advertise<std_msgs::Float64>("/left_shoulder_roll_controller/command", 1);
    ros::Publisher left_waist_pitch_pub = nh.advertise<std_msgs::Float64>("/left_waist_pitch_controller/command", 1);
    ros::Publisher left_waist_roll_pub = nh.advertise<std_msgs::Float64>("/left_waist_roll_controller/command", 1);
    ros::Publisher left_waist_yaw_pub = nh.advertise<std_msgs::Float64>("/left_waist_yaw_controller/command", 1);

    ros::Publisher right_ankle_pitch_pub = nh.advertise<std_msgs::Float64>("/right_ankle_pitch_controller/command", 1);
    ros::Publisher right_ankle_roll_pub = nh.advertise<std_msgs::Float64>("/right_ankle_roll_controller/command", 1);
    ros::Publisher right_elbow_pitch_pub = nh.advertise<std_msgs::Float64>("/right_elbow_pitch_controller/command", 1);
    ros::Publisher right_knee_pitch_pub = nh.advertise<std_msgs::Float64>("/right_knee_pitch_controller/command", 1);
    ros::Publisher right_shoulder_pitch_pub = nh.advertise<std_msgs::Float64>("/right_shoulder_pitch_controller/command", 1);
    ros::Publisher right_shoulder_roll_pub = nh.advertise<std_msgs::Float64>("/right_shoulder_roll_controller/command", 1);
    ros::Publisher right_waist_pitch_pub = nh.advertise<std_msgs::Float64>("/right_waist_pitch_controller/command", 1);
    ros::Publisher right_waist_roll_pub = nh.advertise<std_msgs::Float64>("/right_waist_roll_controller/command", 1);
    ros::Publisher right_waist_yaw_pub = nh.advertise<std_msgs::Float64>("/right_waist_yaw_controller/command", 1);

    std_msgs::Float64 rad;

    var_init();                 // 変数の初期化
    serv_init();              // サーボモータの初期化
    calc_mv_init();             // 動きの計算の初期化

    ros::NodeHandle private_nh("~");
    std::string motion_path_;
    private_nh.param("motion_path", motion_path_, std::string("motions"));
    load_pc_motion(motion_path_.c_str());

    load_eeprom();

    ros::Rate rate(100); //10 ms
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    
    while(ros::ok()){
        if(!motion_flag){
            set_xv_comm(&xv_comm, walk_cmd, num_step, stride_th, stride_x, period, stride_y);
            convert_bin(&xv_comm_bin, &xv_comm);
        }
        cntr();
        
        rad.data = -xv_ref.d[LEG_PITCH_L] * (M_PI/180);
        left_ankle_pitch_pub.publish(rad); 
        rad.data = -xv_ref.d[FOOT_ROLL_L] * (M_PI/180);
        left_ankle_roll_pub.publish(rad); 
        rad.data = -xv_ref.d[ELBOW_PITCH_L] * (M_PI/180);
        left_elbow_pitch_pub.publish(rad); 
        rad.data = -xv_ref.d[KNEE_L1] * (M_PI/180);
        left_knee_pitch_pub.publish(rad); 
        rad.data = -xv_ref.d[ARM_PITCH_L] * (M_PI/180);
        left_shoulder_pitch_pub.publish(rad); 
        rad.data = -xv_ref.d[ARM_ROLL_L] * (M_PI/180);
        left_shoulder_roll_pub.publish(rad); 
        rad.data = -xv_ref.d[KNEE_L2] * (M_PI/180);
        left_waist_pitch_pub.publish(rad); 
        rad.data = -xv_ref.d[LEG_ROLL_L] * (M_PI/180);
        left_waist_roll_pub.publish(rad); 
        rad.data = -xv_ref.d[LEG_YAW_L] * (M_PI/180);
        left_waist_yaw_pub.publish(rad); 

        rad.data = -xv_ref.d[LEG_PITCH_R] * (M_PI/180);
        right_ankle_pitch_pub.publish(rad); 
        rad.data = -xv_ref.d[FOOT_ROLL_R] * (M_PI/180);
        right_ankle_roll_pub.publish(rad); 
        rad.data = -xv_ref.d[ELBOW_PITCH_R] * (M_PI/180);
        right_elbow_pitch_pub.publish(rad); 
        rad.data = -xv_ref.d[KNEE_R1] * (M_PI/180);
        right_knee_pitch_pub.publish(rad); 
        rad.data = -xv_ref.d[ARM_PITCH_R] * (M_PI/180);
        right_shoulder_pitch_pub.publish(rad); 
        rad.data = -xv_ref.d[ARM_ROLL_R] * (M_PI/180);
        right_shoulder_roll_pub.publish(rad); 
        rad.data = -xv_ref.d[KNEE_R2] * (M_PI/180);
        right_waist_pitch_pub.publish(rad); 
        rad.data = -xv_ref.d[LEG_ROLL_R] * (M_PI/180);
        right_waist_roll_pub.publish(rad); 
        rad.data = -xv_ref.d[LEG_YAW_R] * (M_PI/180);
        right_waist_yaw_pub.publish(rad); 
        
        ros::spinOnce();
        rate.sleep();
    }
    
    //spinner.stop();
    return 0;

    /*
    int id = 0;
    short j;
    int shutdown_flag = 0;
    boost::posix_time::ptime ptime = boost::posix_time::microsec_clock::local_time(); 
    const char *servo_port = "/dev/kondoservo";
    if (argc > 1)
        servo_port = argv[1];
    int m_sampleCount = 0;
    int m_sampleRate = 0;
    uint64_t m_rateTimer;
    uint64_t m_displayTimer;
    uint64_t m_now;
    */
    //RTIMUSettings *m_settings = new RTIMUSettings("RTIMULib");
    //RTIMU *imu = RTIMU::createIMU(m_settings);

    /*
    if((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)){
        printf("No IMU found\n");
        exit(1);
    }
    imu->IMUInit();

    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(false);

    m_rateTimer = m_displayTimer = RTMath::currentUSecsSinceEpoch();


    RSOpen(servo_port);
    B3MTorqueALLDown();
    */

    // loop start
    //for( count_time_l = 0; ; count_time_l ++ )
    //{
            /*
        //bool cmd_accept = false;
        {
            // accept command
            mutex::scoped_lock look(lock_obj);
            if (cmd.size() > 0) {
                memcpy(rfmt, &cmd[0], cmd.size());
                joy_read();
                cmd_accept = true;
                cmd = "";
            }
        }
            */
        /*
        {
            ///// MPU9250 reading sensor data, calc quaternion and settings
            static int continueous_error_count = 0;
            
            if(imu->IMURead())
            {
                continueous_error_count = 0;
                RTIMU_DATA imuData = imu->getIMUData();
                m_sampleCount++;
                m_now = RTMath::currentUSecsSinceEpoch();

                if((m_now - m_displayTimer) > 1000000)
                {
                    m_displayTimer = m_now;
                }

                if((m_now - m_rateTimer) > 1000000)
                {
                    m_sampleRate =m_sampleCount;
                    m_sampleCount = 0;
                    m_rateTimer = m_now;
                }

                xv_acc.acc_data1 =  -imu->getIMUData().accel.y();
                xv_acc.acc_data2 =  imu->getIMUData().accel.x();
                xv_acc.acc_data3 =  imu->getIMUData().accel.z();

                const float radian_to_degree = 180.0 / M_PI;
                xv_gyro.gyro_data1 = -imu->getIMUData().gyro.y() * radian_to_degree;
                xv_gyro.gyro_data2 = imu->getIMUData().gyro.x() * radian_to_degree;
                xv_gyro.gyro_data3 = imu->getIMUData().gyro.z() * radian_to_degree;

                xv_gyro.quaternion[0] = -imuData.fusionQPose.x();
                xv_gyro.quaternion[1] = imuData.fusionQPose.z();
                xv_gyro.quaternion[2] = imuData.fusionQPose.scalar();
                xv_gyro.quaternion[3] = imuData.fusionQPose.y();

                double pitch_abs = 180.0 - abs(imuData.fusionPose.x()*RTMATH_RAD_TO_DEGREE);
                if(imuData.fusionPose.x() < 0)
                    pitch_abs = -pitch_abs;
                xv_gyro.gyro_roll = xv_gyro.gyro_roll2 = imuData.fusionPose.y()*RTMATH_RAD_TO_DEGREE;
                xv_gyro.gyro_pitch = xv_gyro.gyro_pitch2 = pitch_abs;
                xv_gyro.gyro_yaw = xv_gyro.gyro_yaw2 = -(imuData.fusionPose.z()*RTMATH_RAD_TO_DEGREE);
            } else {
                FILE *fp = fopen("/var/tmp/error.txt","a");
                if (fp != NULL){
                    fprintf(fp, "IMU Read Error\r\n");
                    fclose(fp);
                }
                continueous_error_count ++;
                if (continueous_error_count > 10) shutdown_flag = 1;
            }
        }
        */

/*
        static unsigned long last_pan_update = 0;
        if ((fabs(xv_gyro.gyro_roll) > 30)||(fabs(xv_gyro.gyro_pitch) > 30)){
            if (abs((long)(count_time_l - last_pan_update)) > 10){
                set_xv_comm(&xv_comm, 'H', '0', '0', '0', '1', '0');
                convert_bin(&xv_comm_bin, &xv_comm);
                last_pan_update = count_time_l;
            }
        }
        */

        /*
        for( j=0; j<SERV_NUM; j++ )
        {
            if( xv_sv[j].deg_sv > xp_sv[j].deg_lim_h*100 || xv_sv[j].deg_sv < xp_sv[j].deg_lim_l*100 )
                    printf("*******ERROR**** xv_sv[%d].deg_sv=%f\n", j, xv_sv[j].deg_sv/100.0f);
        }

        //rtm_main();//モーションローダを動かすのに必要だが、適切なタイミングを見つける必要あり
        // 一定周期のためのwait
        boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time(); 
        boost::posix_time::time_duration diff = now - ptime;
        if (diff.total_milliseconds() > (FRAME_RATE))
        {
            FILE *fp = fopen("/var/tmp/error.txt","a");
            if (fp != NULL){
                time_t now = time(NULL);
                struct tm *pnow = localtime(&now);
                fprintf(fp, "%d:%d:%d WALKING CONTROL OVERTIME: %dms\r\n", 
                pnow->tm_hour, pnow->tm_min, pnow->tm_sec,
                (int)diff.total_milliseconds());
                fclose(fp);
            }
        }

        while(diff.total_milliseconds() < FRAME_RATE) {
//          boost::this_thread::sleep(boost::posix_time::microseconds(10));
            boost::this_thread::sleep(boost::posix_time::microseconds(FRAME_RATE*1000 - diff.total_microseconds() - 100));
            now = boost::posix_time::microsec_clock::local_time(); 
            diff = now - ptime;
        }
//      printf("%dus\n", diff.total_microseconds());
        ptime = now;
//      printf("cnt:%05d mode:%d%d%d%d%d\n", count_time_l, sq_flag.start, sq_flag.straight, sq_flag.ready, sq_flag.walk, sq_flag.motion);
*/
//    }
}
