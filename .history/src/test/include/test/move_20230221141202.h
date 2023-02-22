#ifndef  MOVE_H_
#define  MOVE_H_
#include <ros/ros.h>
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCOut.h>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
Eigen::Vector3d pos_target;//offboard模式下，发送给飞控的期望值
float desire_z = 1; //期望高度
float desire_Radius = 1;//期望圆轨迹半径
float MoveTimeCnt = 0;
float priod = 1000.0;   //减小数值可增大飞圆形的速度
Eigen::Vector3d temp_pos_drone;
Eigen::Vector3d temp_pos_target;
mavros_msgs::SetMode mode_cmd;
ros::Publisher setpoint_raw_local_pub;
