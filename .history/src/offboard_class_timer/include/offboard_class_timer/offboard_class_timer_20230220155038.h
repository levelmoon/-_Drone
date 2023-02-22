#ifndef  OFFBOARD_CLASS_TIMER_H_
#define  OFFBOARD_CLASS_TIMER_H_

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <iostream>


class offboard_class_timer
{
private:
    ros::Publisher local_pos_pub;
	ros::Subscriber local_pos_sub ;      
	ros::Subscriber state_sub ;  
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;


    ros::NodeHandle nh;//we will need this, to pass between "main" and constructor
    void init_publisher();
    void init_subscriber();
    void init_service();

    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
   // void output_publish(geometry_msgs::Point ang1,geometry_msgs::Point ang2,geometry_msgs::Point ang3,std_msgs::Float64 thu1,std_msgs::Float64 thu2,std_msgs::Float64 thu3);

public:
    offboard_class_timer(ros::NodeHandle* nodehandle);
    ~offboard_class_timer();

    ros::Timer calc_timer;
    void calc_cb(const ros::TimerEvent&);

	geometry_msgs::PoseStamped pose;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped local_pos;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    ros::Time last_request;
    int step = 0;
	int sametimes = 0;

    Eigen::Vector3d pos_target;
};


offboard_class_timer::~offboard_class_timer()
{
}


#endif
