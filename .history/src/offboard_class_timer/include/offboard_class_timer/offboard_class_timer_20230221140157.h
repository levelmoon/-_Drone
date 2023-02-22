#ifndef  OFFBOARD_CLASS_TIMER_H_
#define  OFFBOARD_CLASS_TIMER_H_

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>

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

    Eigen::Vector3d pos_target;//offboard模式下，发送给飞控的期望值
    float desire_z = 1; //期望高度
    float desire_Radius = 1;//期望圆轨迹半径
    float MoveTimeCnt = 0;
    float priod = 1000.0;   //减小数值可增大飞圆形的速度
    Eigen::Vector3d temp_pos_drone;
    Eigen::Vector3d temp_pos_target;
    mavros_msgs::SetMode mode_cmd;
    ros::Publisher setpoint_raw_local_pub;

    
    enum
    {
        WAITING,		//等待offboard模式
        CHECKING,		//检查飞机状态
        PREPARE,		//起飞到第一个点
        REST,			//休息一下
        FLY,			//飞圆形路经
        FLYOVER,		//结束		
    }FlyState = WAITING;//初始状态WAITING
};


offboard_class_timer::~offboard_class_timer()
{
}


#endif
