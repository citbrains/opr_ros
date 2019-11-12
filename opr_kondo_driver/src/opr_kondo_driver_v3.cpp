#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
extern "C" {
#include "opr_kondo_driver/b3m.h"
}

const double RAD_TO_DEG = 180.0/M_PI*100;

double deg100_to_radian(double deg100)
{
    return deg100/RAD_TO_DEG;
}

int radian_to_deg100(double radian)
{
    return radian*RAD_TO_DEG;
}

class OPRKondoDriver : public hardware_interface::RobotHW
{
    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        std::vector<std::string> joint_names;
        std::vector<double> cmd;
        std::vector<double> pos;
        std::vector<double> vel;
        std::vector<double> eff;
        B3MData *b3m;

    public:
        OPRKondoDriver(int num, char **actuators){
            ros::NodeHandle nh("~/kondo");
            //TODO: open b3m port
            std::string serial_port;
            nh.param<std::string>("b3m_serial_port", serial_port, "/dev/ttyUSB0");
            ROS_INFO("B3M serial port: %s", serial_port.c_str());
            if(b3m_init(b3m, serial_port.c_str()) < 0){
                ROS_ERROR("could not init B3M: %s\n", b3m->error);
                exit(0);
            }
            
            //TODO: rosparam to b3m settings; velocity, temp, max/min angle, current, etc
            if(b3m_servo_mode(b3m, 1, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 1);}
            if(b3m_servo_mode(b3m, 2, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 2);}
            if(b3m_servo_mode(b3m, 3, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 3);}
            if(b3m_servo_mode(b3m, 4, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 4);}
            if(b3m_servo_mode(b3m, 5, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 5);}
            if(b3m_servo_mode(b3m, 6, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 6);}
            if(b3m_servo_mode(b3m, 7, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 7);}
            if(b3m_servo_mode(b3m, 8, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 8);}
            if(b3m_servo_mode(b3m, 9, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 9);}
            if(b3m_servo_mode(b3m, 10, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 10);}
            if(b3m_servo_mode(b3m, 11, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 11);}
            if(b3m_servo_mode(b3m, 12, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 12);}
            if(b3m_servo_mode(b3m, 13, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 13);}
            if(b3m_servo_mode(b3m, 14, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 14);}
            if(b3m_servo_mode(b3m, 15, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 15);}
            if(b3m_servo_mode(b3m, 16, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 16);}
            if(b3m_servo_mode(b3m, 17, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 17);}
            if(b3m_servo_mode(b3m, 18, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 18);}
            if(b3m_servo_mode(b3m, 19, B3M_OPTIONS_RUN_NORMAL)) {ROS_WARN("cannot connect to servo ID: %d", 19);}
            if(b3m_set_trajectory_mode(b3m, 1, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 1);}
            if(b3m_set_trajectory_mode(b3m, 2, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 2);}
            if(b3m_set_trajectory_mode(b3m, 3, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 3);}
            if(b3m_set_trajectory_mode(b3m, 4, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 4);}
            if(b3m_set_trajectory_mode(b3m, 5, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 5);}
            if(b3m_set_trajectory_mode(b3m, 6, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 6);}
            if(b3m_set_trajectory_mode(b3m, 7, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 7);}
            if(b3m_set_trajectory_mode(b3m, 8, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 8);}
            if(b3m_set_trajectory_mode(b3m, 9, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 9);}
            if(b3m_set_trajectory_mode(b3m, 10, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 10);}
            if(b3m_set_trajectory_mode(b3m, 11, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 11);}
            if(b3m_set_trajectory_mode(b3m, 12, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 12);}
            if(b3m_set_trajectory_mode(b3m, 13, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 13);}
            if(b3m_set_trajectory_mode(b3m, 14, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 14);}
            if(b3m_set_trajectory_mode(b3m, 15, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 15);}
            if(b3m_set_trajectory_mode(b3m, 16, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 16);}
            if(b3m_set_trajectory_mode(b3m, 17, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 17);}
            if(b3m_set_trajectory_mode(b3m, 18, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 18);}
            if(b3m_set_trajectory_mode(b3m, 19, B3M_OPTIONS_TRAJECTORY_4)) {ROS_WARN("cannot set trajectory mode to servo ID: %d", 19);}

            //TODO: rosparam to joint_names
            joint_names.push_back("head_yaw_joint");
            joint_names.push_back("left_shoulder_pitch_joint");
            joint_names.push_back("left_shoulder_roll_joint");
            joint_names.push_back("left_elbow_pitch_joint");
            joint_names.push_back("left_waist_yaw_joint");
            joint_names.push_back("left_waist_roll_joint");
            joint_names.push_back("left_waist_pitch_joint");
            joint_names.push_back("left_knee_pitch_joint");
            joint_names.push_back("left_ankle_pitch_joint");
            joint_names.push_back("left_ankle_roll_joint");
            joint_names.push_back("right_shoulder_pitch_joint");
            joint_names.push_back("right_shoulder_roll_joint");
            joint_names.push_back("right_elbow_pitch_joint");
            joint_names.push_back("right_waist_yaw_joint");
            joint_names.push_back("right_waist_roll_joint");
            joint_names.push_back("right_waist_pitch_joint");
            joint_names.push_back("right_knee_pitch_joint");
            joint_names.push_back("right_ankle_pitch_joint");
            joint_names.push_back("right_ankle_roll_joint");

            //TODO: rosparam to offset angles
            cmd.assign(joint_names.size(), 0);
            pos.assign(joint_names.size(), 0);
            vel.assign(joint_names.size(), 0);
            eff.assign(joint_names.size(), 0);

            for(int i=0; i<joint_names.size(); i++){
                hardware_interface::JointStateHandle state_handle(joint_names[i].c_str(), &pos[i], &vel[i], &eff[i]);
                jnt_state_interface.registerHandle(state_handle);

                hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_names[i].c_str()), &cmd[i]);
                jnt_pos_interface.registerHandle(pos_handle);
            }

            registerInterface(&jnt_state_interface);
            registerInterface(&jnt_pos_interface);
        }

        ~OPRKondoDriver(){
            b3m_close(b3m);
        }

        void read(){
            int deg100 = 0;

            b3m_get_angle(b3m, 1, &deg100);
            pos[0] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 2, &deg100);
            pos[1] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 3, &deg100);
            pos[2] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 4, &deg100);
            pos[3] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 5, &deg100);
            pos[4] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 6, &deg100);
            pos[5] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 7, &deg100);
            pos[6] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 8, &deg100);
            pos[7] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 9, &deg100);
            pos[8] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 10, &deg100);
            pos[9] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 11, &deg100);
            pos[10] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 12, &deg100);
            pos[11] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 13, &deg100);
            pos[12] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 14, &deg100);
            pos[13] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 15, &deg100);
            pos[14] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 16, &deg100);
            pos[15] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 17, &deg100);
            pos[16] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 18, &deg100);
            pos[17] = deg100_to_radian(deg100);
            b3m_get_angle(b3m, 19, &deg100);
            pos[18] = deg100_to_radian(deg100);

            //TODO: get velocity and effort
        }

        void write(){
            //TODO: limit cmd, add cmd offset
            
            int deg100 = 0;

            deg100 = radian_to_deg100(cmd[0]);
            b3m_set_angle(b3m, 1, deg100);
            deg100 = radian_to_deg100(cmd[1]);
            b3m_set_angle(b3m, 2, deg100);
            deg100 = radian_to_deg100(cmd[2]);
            b3m_set_angle(b3m, 3, deg100);
            deg100 = radian_to_deg100(cmd[3]);
            b3m_set_angle(b3m, 4, deg100);
            deg100 = radian_to_deg100(cmd[4]);
            b3m_set_angle(b3m, 5, deg100);
            deg100 = radian_to_deg100(cmd[5]);
            b3m_set_angle(b3m, 6, deg100);
            deg100 = radian_to_deg100(cmd[6]);
            b3m_set_angle(b3m, 7, deg100);
            deg100 = radian_to_deg100(cmd[7]);
            b3m_set_angle(b3m, 8, deg100);
            deg100 = radian_to_deg100(cmd[8]);
            b3m_set_angle(b3m, 9, deg100);
            deg100 = radian_to_deg100(cmd[9]);
            b3m_set_angle(b3m, 10, deg100);
            deg100 = radian_to_deg100(cmd[10]);
            b3m_set_angle(b3m, 11, deg100);
            deg100 = radian_to_deg100(cmd[11]);
            b3m_set_angle(b3m, 12, deg100);
            deg100 = radian_to_deg100(cmd[12]);
            b3m_set_angle(b3m, 13, deg100);
            deg100 = radian_to_deg100(cmd[13]);
            b3m_set_angle(b3m, 14, deg100);
            deg100 = radian_to_deg100(cmd[14]);
            b3m_set_angle(b3m, 15, deg100);
            deg100 = radian_to_deg100(cmd[15]);
            b3m_set_angle(b3m, 16, deg100);
            deg100 = radian_to_deg100(cmd[16]);
            b3m_set_angle(b3m, 17, deg100);
            deg100 = radian_to_deg100(cmd[17]);
            b3m_set_angle(b3m, 18, deg100);
            deg100 = radian_to_deg100(cmd[18]);
            b3m_set_angle(b3m, 19, deg100);
        }

        ros::Time getTime() const {return ros::Time::now();}
        ros::Duration getPeriod() const {return ros::Duration(0.05);}
};

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "opr_kondo_driver");
    ros::NodeHandle nh;

    OPRKondoDriver robot(argc-1, &argv[1]);

    controller_manager::ControllerManager cm(&robot, nh);

    ros::Rate rate(1.0 / robot.getPeriod().toSec());
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok()){
        robot.read();
        cm.update(robot.getTime(), robot.getPeriod());
        robot.write();
        rate.sleep();
    }
    
    spinner.stop();
    
    return 0;
}
