#ifndef  MOVE_H_
#define  MOVE_H_

#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCOut.h>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>

class planner
{
private:
    ros::Subscriber position_sub;
    ros::Subscriber state_sub;

    ros::Publisher setpoint_raw_local_pub;

    ros::ServiceClient set_mode_client ;

    ros::NodeHandle nh;//we will need this, to pass between "main" and constructor

    void init_publisher();
    void init_subscriber();
    void init_service();

    void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);

    void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp);  
   // void output_publish(geometry_msgs::Point ang1,geometry_msgs::Point ang2,geometry_msgs::Point ang3,std_msgs::Float64 thu1,std_msgs::Float64 thu2,std_msgs::Float64 thu3);

public:
    planner(ros::NodeHandle* nodehandle);
    ~planner();

    ros::Timer calc_timer;
    void calc_cb(const ros::TimerEvent&);


    Eigen::Vector3d pos_drone;//接收来自飞控的当前飞机位置
    mavros_msgs::State current_state;//接收来自飞控的飞机状态


    Eigen::Vector3d pos_target;//offboard模式下，发送给飞控的期望值
    float desire_z = 1; //期望高度
    float desire_Radius = 1;//期望圆轨迹半径
    float MoveTimeCnt = 0;
    float priod = 1000.0;   //减小数值可增大飞圆形的速度
    Eigen::Vector3d temp_pos_drone;
    Eigen::Vector3d temp_pos_target;
    Eigen::Vector3d pos_drone;
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


planner::~planner()
{
    std::cout << "kill all" << std::endl;
}

#endif