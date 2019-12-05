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

#include    <stdio.h>
#include    <assert.h>
#include    <stdio.h>
#include    <math.h>
//#include  <KSerialPort.h>
#include    <boost/thread.hpp>
#include    <string>

#include "pc_motion.h"
//#include  "ADIS16375.h"
//#include  "OrientationEstimator.h"

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

//========================
// オフセット入力（実機からのデータ）
//========================
int offset_load(char *filename, int servo_offset[SERV_NUM]){
    char off_tmp[100];
    int i, ser_ID, off_angle;
    FILE *fp;

    for(i = 0; i < SERV_NUM; i ++){
        servo_offset[i] = 0;
    }

    if(NULL == (fp = fopen(filename, "r"))){
        fprintf(stderr, "cannot open (offset_load): %s\n", filename);
        return -1;
    }
    while(fscanf(fp, "%d %d %s",&ser_ID, &off_angle, off_tmp) != EOF){
        servo_offset[ser_ID] = off_angle;
    }
    fclose(fp);

    return 0;
}

//========================
// eeprom load
//========================
int eeprom_load(char *filename)
{
    char buff_tmp[100];
    int buff_num;
    float buff_val, eeprom_buff[111];
    FILE *fp;

    if(NULL == (fp = fopen(filename,"r"))){
        fprintf(stderr, "cannot open (eeprom_load): %s\n", filename);
        return -1;
    }
    while(fscanf(fp, "%d %f %s",&buff_num, &buff_val, buff_tmp) != EOF){
        eeprom_buff[buff_num] = buff_val;
    }
    fclose(fp);

    //flag_eeprom_para_ok           =   eeprom_buff[ 0];
    //sw_soft1                      =   eeprom_buff[ 1];
    //sw_soft2                      =   eeprom_buff[ 2];
    //sw_soft3                      =   eeprom_buff[ 3];
    //sw_soft4                      =   eeprom_buff[ 4];
    xp_acc.acc_k1                   =   eeprom_buff[ 5];
    xp_acc.acc_k2                   =   eeprom_buff[ 6];
    xp_acc.acc_k3                   =   eeprom_buff[ 7];
    xp_acc.ad_volt_offset1          =   eeprom_buff[ 8];
    xp_acc.ad_volt_offset2          =   eeprom_buff[ 9];
    xp_acc.ad_volt_offset3          =   eeprom_buff[10];
    xp_acc.t1                       =   eeprom_buff[11];
    xp_acc.t2                       =   eeprom_buff[12];
    xp_acc.fall_fwd                 =   eeprom_buff[13];
    xp_acc.fall_bwd                 =   eeprom_buff[14];
    xp_acc.fall_right               =   eeprom_buff[15];
    xp_acc.fall_left                =   eeprom_buff[16];
    xp_acc.fall_check_time          =   eeprom_buff[17];
    xp_acc.fall_pitch               =   eeprom_buff[18];
    xp_acc.fall_roll                =   eeprom_buff[19];
    xp_acc.fall_pitch_oblique       =   eeprom_buff[20];
    xp_acc.fall_roll_oblique        =   eeprom_buff[21];
            
    xp_gyro.kp1_foot                =   eeprom_buff[25];
    xp_gyro.kp2_foot                =   eeprom_buff[26];
    xp_gyro.kp1_hip                 =   eeprom_buff[27];
    xp_gyro.kp2_hip                 =   eeprom_buff[28];
    xp_gyro.kp1_arm                 =   eeprom_buff[29];
    xp_gyro.kp2_arm                 =   eeprom_buff[30];
    xp_gyro.kp2_waist               =   eeprom_buff[31];
    xp_gyro.kp3_waist               =   eeprom_buff[32];
    xp_gyro.gyro_k1                 =   eeprom_buff[33];
    xp_gyro.gyro_k2                 =   eeprom_buff[34];
    xp_gyro.gyro_k3                 =   eeprom_buff[35];
    xp_gyro.ad_volt_offset1         =   eeprom_buff[36];
    xp_gyro.ad_volt_offset2         =   eeprom_buff[37];
    xp_gyro.ad_volt_offset3         =   eeprom_buff[38];
    xp_gyro.t1                      =   eeprom_buff[39];
    xp_gyro.t2                      =   eeprom_buff[40];
    xp_gyro.gyro_data3_flt2_t1      =   eeprom_buff[41];
    xp_gyro.yaw_cntl_gain           =   eeprom_buff[42];
    xp_gyro.yaw_cntl_dead           =   eeprom_buff[43];
    xp_gyro.yaw_cntl_theta          =   eeprom_buff[44];
    xp_gyro.gyro_omega              =   eeprom_buff[45];
    xp_gyro.fall_roll_deg1          =   eeprom_buff[46];
    xp_gyro.fall_pitch_deg1         =   eeprom_buff[47];
    flag_gyro.fall_cntl             =   (short)eeprom_buff[48];
            
    xp_mv_straight.time             =   eeprom_buff[50];
    xp_mv_straight.z3               =   eeprom_buff[51];
    xp_mv_straight.arm_sh_pitch     =   eeprom_buff[52];
    xp_mv_straight.arm_sh_roll      =   eeprom_buff[53];
    xp_mv_straight.arm_el_yaw       =   eeprom_buff[54];
    xp_mv_straight.arm_el_pitch     =   eeprom_buff[55];
            
    xp_mv_ready.time                =   eeprom_buff[56];
    xp_mv_ready.z3                  =   eeprom_buff[57];
    xp_mv_ready.arm_sh_pitch        =   eeprom_buff[58];
    xp_mv_ready.arm_sh_roll         =   eeprom_buff[59];
    xp_mv_ready.arm_el_yaw          =   eeprom_buff[60];
    xp_mv_ready.arm_el_pitch        =   eeprom_buff[61];
    xp_mv_ready.pitch               =   eeprom_buff[62];

    xp_mv_walk.num                  =   (long)eeprom_buff[65];
    xp_mv_walk.h_cog                =   eeprom_buff[66];
    xp_mv_walk.time                 =   eeprom_buff[67];
    xp_mv_walk.x_fwd_swg            =   eeprom_buff[69];
    xp_mv_walk.x_fwd_spt            =   eeprom_buff[70];
    xp_mv_walk.x_bwd_swg            =   eeprom_buff[71];
    xp_mv_walk.x_bwd_spt            =   eeprom_buff[72];
    xp_mv_walk.y_swg                =   eeprom_buff[73];
    xp_mv_walk.y_spt                =   eeprom_buff[74];
    xp_mv_walk.theta                =   eeprom_buff[75];
    xp_mv_walk.z                    =   eeprom_buff[76];
    xp_mv_walk.y_balance            =   eeprom_buff[77];
    xp_mv_walk.hip_roll             =   eeprom_buff[78];
    xp_mv_walk.x_fwd_pitch          =   eeprom_buff[79];
    xp_mv_walk.x_bwd_pitch          =   eeprom_buff[80];
    xp_mv_walk.arm_sh_pitch         =   eeprom_buff[81];
    xp_mv_walk.start_time_k1        =   eeprom_buff[82];
    xp_mv_walk.start_zmp_k1         =   eeprom_buff[83];
//  xp_mv_walk.start_time_k2        =   eeprom_buff[84];
    xp_mv_walk.foot_cntl_p          =   eeprom_buff[85];
    xp_mv_walk.foot_cntl_r          =   eeprom_buff[86];
    xp_mv_walk.sidestep_time_k      =   eeprom_buff[87];
    xp_mv_walk.sidestep_roll        =   eeprom_buff[88];
    xp_mv_walk.y_wide               =   eeprom_buff[90];
    xp_mv_walk.time_dutyfactor      =   eeprom_buff[91];
            
    xp_dlim_wait_x.dlim             =   eeprom_buff[92];
    xp_dlim_wait_x.wait_time        =   eeprom_buff[93];
    xp_dlim_wait_y.dlim             =   eeprom_buff[94];
    xp_dlim_wait_y.wait_time        =   eeprom_buff[95];
    xp_dlim_wait_theta.dlim         =   eeprom_buff[96];
    xp_dlim_wait_theta.wait_time    =   eeprom_buff[97];
            
    odometry_correct_para_x         =   eeprom_buff[98];
    odometry_correct_para_y         =   eeprom_buff[99];

    xp_mv_walk.x_fwd_acc_pitch      =   eeprom_buff[101];
    xp_mv_walk.x_bwd_acc_pitch      =   eeprom_buff[102];
    xp_dlim_wait_pitch.dlim         =   eeprom_buff[103];       // ピッチを変更する比率
    xp_mv_walk.accurate_x_percent_dlim = eeprom_buff[104];
    xp_mv_walk.accurate_y_percent_dlim = eeprom_buff[105];
    xp_mv_walk.accurate_th_percent_dlim = eeprom_buff[106];
    xp_mv_walk.arm_el_pitch         =   eeprom_buff[107];

    return 0;
}

void motionCallback(const std_msgs::Int32::ConstPtr &msg)
{
    //walk_cmd = 'M';
}

void cancelCallback(const std_msgs::Bool::ConstPtr &msg)
{
    walk_cmd = 'C';
    num_step  = 0;
    period    = 0;
    stride_x  = 0;
    stride_y  = 0;
    stride_th = 0;
}

void walkCallback(const hajime_walk_ros::WalkCommand::ConstPtr &msg)
{
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
    //xv_gyro.gyro_data1 = -imu->getIMUData().gyro.y() * radian_to_degree;
    //xv_gyro.gyro_data2 = imu->getIMUData().gyro.x() * radian_to_degree;
    //xv_gyro.gyro_data3 = imu->getIMUData().gyro.z() * radian_to_degree;

    xv_gyro.quaternion[0] = msg->orientation.x;//-imuData.fusionQPose.x();
    xv_gyro.quaternion[1] = msg->orientation.y;//imuData.fusionQPose.z();
    xv_gyro.quaternion[2] = msg->orientation.z;//imuData.fusionQPose.scalar();
    xv_gyro.quaternion[3] = msg->orientation.w;//imuData.fusionQPose.y();

    //double pitch_abs = 180.0 - abs(imuData.fusionPose.x()*RTMATH_RAD_TO_DEGREE);
    //if(imuData.fusionPose.x() < 0)
    //    pitch_abs = -pitch_abs;
    xv_gyro.gyro_roll = xv_gyro.gyro_roll2 = roll * radian_to_degree;//imuData.fusionPose.y()*RTMATH_RAD_TO_DEGREE;
    xv_gyro.gyro_pitch = xv_gyro.gyro_pitch2 = pitch * radian_to_degree;//pitch_abs;
    xv_gyro.gyro_yaw = xv_gyro.gyro_yaw2 = yaw * radian_to_degree;//-(imuData.fusionPose.z()*RTMATH_RAD_TO_DEGREE);
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
    ros::Publisher left_waist_yaw_pub = nh.advertise<std_msgs::Float64>("/left_waist_yaw_contyawer/command", 1);

    ros::Publisher right_ankle_pitch_pub = nh.advertise<std_msgs::Float64>("/right_ankle_pitch_controller/command", 1);
    ros::Publisher right_ankle_roll_pub = nh.advertise<std_msgs::Float64>("/right_ankle_roll_controller/command", 1);
    ros::Publisher right_elbow_pitch_pub = nh.advertise<std_msgs::Float64>("/right_elbow_pitch_controller/command", 1);
    ros::Publisher right_knee_pitch_pub = nh.advertise<std_msgs::Float64>("/right_knee_pitch_controller/command", 1);
    ros::Publisher right_shoulder_pitch_pub = nh.advertise<std_msgs::Float64>("/right_shoulder_pitch_controller/command", 1);
    ros::Publisher right_shoulder_roll_pub = nh.advertise<std_msgs::Float64>("/right_shoulder_roll_controller/command", 1);
    ros::Publisher right_waist_pitch_pub = nh.advertise<std_msgs::Float64>("/right_waist_pitch_controller/command", 1);
    ros::Publisher right_waist_roll_pub = nh.advertise<std_msgs::Float64>("/right_waist_roll_controller/command", 1);
    ros::Publisher right_waist_yaw_pub = nh.advertise<std_msgs::Float64>("/right_waist_yaw_contyawer/command", 1);

    std_msgs::Float64 rad;
    int i;

    var_init();                 // 変数の初期化
    serv_init();              // サーボモータの初期化
    calc_mv_init();             // 動きの計算の初期化
    //load_pc_motion("motions");
    //offset_load((char *)"/home/rdc-lab/catkin_ws/src/opr_ros/hajime_walk_ros/src/offset_angle.txt", servo_offset);

    std::string eeprom_path = ros::package::getPath("hajime_walk_ros") + "/config/eeprom_list.txt";
    nh.getParam("eeprom_path", eeprom_path);
    eeprom_load((char *) eeprom_path.c_str());

    flag_gyro.zero = ON;

    ros::Rate rate(100); //10 ms
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    
    count_time_l = 0;
    while(ros::ok()){
        set_xv_comm(&xv_comm, walk_cmd, num_step, stride_th, stride_x, period, stride_y);
        convert_bin(&xv_comm_bin, &xv_comm);
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
