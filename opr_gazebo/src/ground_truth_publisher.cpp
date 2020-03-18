#include <cmath>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include "opr_msgs/BallPos.h"

class GroundTruthPublisher
{
    private:
        bool get_selfpos;
        bool get_ballpos;
        std::string robot_name;

    protected:
        ros::NodeHandle nh;
        ros::Publisher pub_selfpos = nh.advertise<geometry_msgs::Pose2D>("/brain/selfpos", 1);
        ros::Publisher pub_ballpos = nh.advertise<opr_msgs::BallPos>("/brain/ballpos", 1);
        ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 2, &GroundTruthPublisher::getState, this);
        
    public:
        GroundTruthPublisher(void)
        {
            nh.param("robot_name", robot_name, std::string("robot1"));
            nh.param("get_selfpos", get_selfpos, true);
            nh.param("get_ballpos", get_ballpos, true);
        }
        
        void getState(const gazebo_msgs::ModelStates::ConstPtr &msg)
        {
            geometry_msgs::Pose2D selfpos;
            opr_msgs::BallPos ballpos;

            for(int i=0; i<msg->name.size(); i++){
                if((msg->name[i] == robot_name)){
                    geometry_msgs::Pose pose = msg->pose[i];

                    selfpos.x = pose.position.x;
                    selfpos.y = pose.position.y;
                    
                    tf::Quaternion q(
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w);
                    tf::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);
                    selfpos.theta = yaw;

                }
                else if((msg->name[i] == "ball")){
                    geometry_msgs::Pose pose = msg->pose[i];
                    ballpos.global.x = pose.position.x;
                    ballpos.global.y = pose.position.y;
                    ballpos.global.theta = 0;
                }
            }

            if(get_selfpos){
                pub_selfpos.publish(selfpos);
            }

            if(get_ballpos){
                ballpos.local.x = cos(-selfpos.theta) * (ballpos.global.x - selfpos.x) - sin(-selfpos.theta) * (ballpos.global.y - selfpos.y);
                ballpos.local.y = sin(-selfpos.theta) * (ballpos.global.x - selfpos.x) + cos(-selfpos.theta) * (ballpos.global.y - selfpos.y);
                ballpos.local.theta = atan2(ballpos.local.y, ballpos.local.x);
                
                pub_ballpos.publish(ballpos);
            }
        }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_truth_publisher");
    
    GroundTruthPublisher ground_truth_publisher;
    while(ros::ok()){
        ros::spinOnce();
    }
}

