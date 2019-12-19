/*
 * B3M Hardware interface for OPR_V3
 */
#include <boost/shared_ptr.hpp>
#include <math.h>
#include "ros/ros.h"
#include "controller_manager/controller_manager.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/actuator_command_interface.h"
#include "hardware_interface/actuator_state_interface.h"
#include "hardware_interface/robot_hw.h"
#include "opr_kondo_driver/b3m.hpp"

#define SERV_NUM 19
#define REDUCTION_RATIO (2.0f)

const double RAD_TO_DEG = 180.0/M_PI*100;

double deg100_to_radian (double deg100)
{
    return deg100/RAD_TO_DEG;
}

int radian_to_deg100 (double radian)
{
    return radian*RAD_TO_DEG;
}

class KondoMotor
{
    public:
        int dir;
        double offset;
        double cmd, pos, vel, eff;
        std::string joint_name;
        int id, min_angle, max_angle, control_gain_preset;
        int control_kp0, control_kp1, control_kp2;
        int control_kd0, control_kd1, control_kd2;
        int control_ki0, control_ki1, control_ki2;
        int control_static_friction0, control_static_friction1, control_static_friction2;
        int control_dynamic_friction0, control_dynamic_friction1, control_dynamic_friction2;

        KondoMotor (std::string actuator_name, hardware_interface::JointStateInterface& state_interface, hardware_interface::PositionJointInterface& pos_interface) : cmd(0.0), pos(0.0), vel(0.0), eff(0.0) {

            ros::NodeHandle nh(std::string("~")+actuator_name);
            if (nh.getParam("id", id)) {
                ROS_INFO("id: %d", id);
            }
            if (nh.getParam("joint_name", joint_name)) {
                ROS_INFO("joint_name: %s", joint_name.c_str());
            }
            if (nh.getParam("min_angle", min_angle)) {
                ROS_INFO("min_angle: %d", min_angle);
            }
            if (nh.getParam("max_angle", max_angle)) {
                ROS_INFO("max_angle: %d", max_angle);
            }
            if (nh.getParam("offset", offset)) {
                ROS_INFO("offset: %f", offset);
            }
            if (nh.getParam("direction", dir)) {
                ROS_INFO("direction: %d", dir);
            }
            if (nh.getParam("control_kp0", control_kp0)) {
                ROS_INFO("control_kp0: %d", control_kp0);
            }
            if (nh.getParam("control_kd0", control_kd0)) {
                ROS_INFO("control_kd0: %d", control_kd0);
            }
            if (nh.getParam("control_ki0", control_ki0)) {
                ROS_INFO("control_ki0: %d", control_ki0);
            }
            if (nh.getParam("control_static_friction0", control_static_friction0)) {
                ROS_INFO("control_static_friction0: %d", control_static_friction0);
            }
            if (nh.getParam("control_dynamic_friction0", control_dynamic_friction0)) {
                ROS_INFO("control_dynamic_friction0: %d", control_dynamic_friction0);
            }
            if (nh.getParam("control_kp1", control_kp1)) {
                ROS_INFO("control_kp1: %d", control_kp1);
            }
            if (nh.getParam("control_kd1", control_kd1)) {
                ROS_INFO("control_kd1: %d", control_kd1);
            }
            if (nh.getParam("control_ki1", control_ki1)) {
                ROS_INFO("control_ki1: %d", control_ki1);
            }
            if (nh.getParam("control_static_friction1", control_static_friction1)) {
                ROS_INFO("control_static_friction1: %d", control_static_friction1);
            }
            if (nh.getParam("control_dynamic_friction1", control_dynamic_friction1)) {
                ROS_INFO("control_dynamic_friction1: %d", control_dynamic_friction1);
            }
            if (nh.getParam("control_kp2", control_kp2)) {
                ROS_INFO("control_kp2: %d", control_kp2);
            }
            if (nh.getParam("control_kd2", control_kd2)) {
                ROS_INFO("control_kd2: %d", control_kd2);
            }
            if (nh.getParam("control_ki2", control_ki2)) {
                ROS_INFO("control_ki2: %d", control_ki2);
            }
            if (nh.getParam("control_static_friction2", control_static_friction2)) {
                ROS_INFO("control_static_friction2: %d", control_static_friction2);
            }
            if (nh.getParam("control_dynamic_friction2", control_dynamic_friction2)) {
                ROS_INFO("control_dynamic_friction2: %d", control_dynamic_friction2);
            }
            if (nh.getParam("control_gain_preset", control_gain_preset)) {
                ROS_INFO("control_gain_preset: %d", control_gain_preset);
            }
            
            hardware_interface::JointStateHandle state_handle(joint_name, &pos, &vel, &eff);
            state_interface.registerHandle(state_handle);
            hardware_interface::JointHandle pos_handle(state_interface.getHandle(joint_name), &cmd);
            pos_interface.registerHandle(pos_handle);
        }

        void update(int *deg100)
        {
            double radian = cmd;
            if (radian < min_angle * 3.14 / 180) {
                radian = min_angle * 3.14 / 180;
            }
            if (radian > max_angle * 3.14 / 180) {
                radian = max_angle * 3.14 / 180;
            }
            
            if (joint_name == "left_waist_roll_joint" || joint_name == "right_waist_roll_joint"){
                radian *= REDUCTION_RATIO;
            }

            *deg100 = dir * radian_to_deg100(radian + offset);

            //TODO: get pos, vel, eff(torque) from b3m
            //pos = deg100_to_radian(dir * deg100) - offset;
            pos = cmd;
        }
};

class KondoDriver : public hardware_interface::RobotHW
{
    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        std::vector<boost::shared_ptr<KondoMotor> > actuator_vector;
        unsigned short goal_position[SERV_NUM] = {0};

    public:
        KondoDriver (int num, char** actuators)
        {
            ros::NodeHandle nh("~");

            std::string serial_port;
            nh.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
            ROS_INFO("serial_port: %s", serial_port.c_str());
          
            if(RSOpen(serial_port.c_str()) < 0){
                ROS_ERROR ("Could not open B3M port");
                exit(0);
            }

            for (int i=0; i<num; i++) {
                boost::shared_ptr<KondoMotor> actuator(new KondoMotor(std::string(actuators[i]), jnt_state_interface, jnt_pos_interface));
                actuator_vector.push_back(actuator);
            }
            registerInterface(&jnt_state_interface);
            registerInterface(&jnt_pos_interface);

            ROS_INFO("initialize b3m");
            B3MInitializeVariable();
            for (int i=0; i<actuator_vector.size(); i++) {
                int index = actuator_vector[i]->id - 1;
                b3m_control_kp0[index] = actuator_vector[i]->control_kp0;
                b3m_control_kd0[index] = actuator_vector[i]->control_kd0;
                b3m_control_ki0[index] = actuator_vector[i]->control_ki0;
                b3m_control_static_friction0[index] = actuator_vector[i]->control_static_friction0;
                b3m_control_dynamic_friction0[index] = actuator_vector[i]->control_dynamic_friction0;
                b3m_control_kp1[index] = actuator_vector[i]->control_kp1;
                b3m_control_kd1[index] = actuator_vector[i]->control_kd1;
                b3m_control_ki1[index] = actuator_vector[i]->control_ki1;
                b3m_control_static_friction1[index] = actuator_vector[i]->control_static_friction1;
                b3m_control_dynamic_friction1[index] = actuator_vector[i]->control_dynamic_friction1;
                b3m_control_kp2[index] = actuator_vector[i]->control_kp2;
                b3m_control_kd2[index] = actuator_vector[i]->control_kd2;
                b3m_control_ki2[index] = actuator_vector[i]->control_ki2;
                b3m_control_static_friction2[index] = actuator_vector[i]->control_static_friction2;
                b3m_control_dynamic_friction2[index] = actuator_vector[i]->control_dynamic_friction2;
                b3m_gain_preset[index] = actuator_vector[i]->control_gain_preset;
                b3m_servo_offset[index] = actuator_vector[i]->dir * radian_to_deg100(actuator_vector[i]->offset);
            }

            B3MInitializeSequence();
            ROS_INFO("initialize complete");
        }

        void update()
        {   
            int deg100 = 0;
            int index = 0;
            for (int i=0; i<actuator_vector.size(); i++) {
                actuator_vector[i]->update(&deg100);
                index = actuator_vector[i]->id - 1;
                goal_position[index] = deg100;
            }

            //TODO: adjust gain depending on situation
            Write_Servo_B3M_All_2Kport(B3M_SERVO_DESIRED_POSITION, &goal_position[0], 2);
        }

        ~KondoDriver () {
            //Write_Servo_B3M_All_2Kport(B3M_SERVO_SERVO_MODE, &b3m_free_mode[0], 1);
            RSClose();
        }

        ros::Time getTime() const {return ros::Time::now();}
        ros::Duration getPeriod() const {return ros::Duration(0.05);}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kondo_driver");
    ros::NodeHandle nh;

    KondoDriver robot(argc-1, &argv[1]);
    controller_manager::ControllerManager cm(&robot, nh);

    ros::Rate rate(1.0 / robot.getPeriod().toSec());
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok()){
        cm.update(robot.getTime(), robot.getPeriod());
        robot.update();
        rate.sleep();
    }
    spinner.stop();

    return 0;
}


