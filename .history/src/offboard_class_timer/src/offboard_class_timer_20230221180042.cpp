#include <offboard_class_timer/offboard_class_timer.h>



offboard_class_timer::offboard_class_timer(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    calc_timer = nh.createTimer(ros::Duration(0.05), &offboard_class_timer::calc_cb, this);  //timer used to publish state, should be at least for some minimal frequency
}

void offboard_class_timer::init_publisher()
{
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    ("/mavros/setpoint_raw/local", 10,this);
}


void offboard_class_timer::init_subscriber()
{
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
		("mavros/local_position/pose", 20, &offboard_class_timer::local_pos_cb,this);
	state_sub = nh.subscribe<mavros_msgs::State>
		("mavros/state", 20, &offboard_class_timer::state_cb,this);
}

void offboard_class_timer::init_service()
{
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
		("mavros/cmd/arming",this);
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
		("mavros/set_mode",this);
}

void offboard_class_timer::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_pos = *msg;
	
}

void offboard_class_timer::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void offboard_class_timer::calc_cb(const ros::TimerEvent&)
{
	switch(FlyState)
	{
		case WAITING:
			if(current_state.mode != "OFFBOARD")//等待offboard模式
			{
				pos_target[0] = pos_drone[0];
				pos_target[1] = pos_drone[1];
				pos_target[2] = pos_drone[2];
				temp_pos_drone[0] = pos_drone[0];
				temp_pos_drone[1] = pos_drone[1];
				temp_pos_drone[2] = pos_drone[2];
				send_pos_setpoint(pos_target, 0);
			}
			else
			{
				pos_target[0] = temp_pos_drone[0];
				pos_target[1] = temp_pos_drone[1];
				pos_target[2] = temp_pos_drone[2];
				send_pos_setpoint(pos_target, 0);
				FlyState = CHECKING;
			}
			ROS_INFO("%d", current_state.connected);
			// cout << current_state.connected << endl;
			cout << "WAITING" <<endl;
			break;
		case CHECKING:
			if(pos_drone[0] == 0 && pos_drone[1] == 0) 			//没有位置信息则执行降落模式
			{
				cout << "Check error, make sure have local location" <<endl;
				mode_cmd.request.custom_mode = "AUTO.LAND";
				set_mode_client.call(mode_cmd);
				FlyState = WAITING;	
			}
			else
			{
				FlyState = PREPARE;
				MoveTimeCnt = 0;
			}
			cout << "CHECKING" <<endl;
			break;
		case PREPARE:											//起飞到圆轨迹的第一个点,起点在X负半轴
			temp_pos_target[0] = temp_pos_drone[0] - desire_Radius;
			temp_pos_target[1] = temp_pos_drone[1];
			temp_pos_target[2] = desire_z;
			MoveTimeCnt +=2;
			if(MoveTimeCnt >=500)
			{
				FlyState = REST;
				MoveTimeCnt = 0;
			}
			pos_target[0]=temp_pos_drone[0]+(temp_pos_target[0]-temp_pos_drone[0])*(MoveTimeCnt/500);
			pos_target[1]=temp_pos_drone[1]+(temp_pos_target[1]-temp_pos_drone[1])*(MoveTimeCnt/500);
			pos_target[2]=temp_pos_drone[2]+(temp_pos_target[2]-temp_pos_drone[2])*(MoveTimeCnt/500);
			send_pos_setpoint(pos_target, 0);					
			if(current_state.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			cout << "PREPARE" <<endl;
			break;
		case REST:	
			pos_target[0] = temp_pos_drone[0] - desire_Radius;
			pos_target[1] = temp_pos_drone[1] ;
			pos_target[2] = desire_z;
			send_pos_setpoint(pos_target, 0);
			MoveTimeCnt +=1;
			if(MoveTimeCnt >= 100)
			{
				MoveTimeCnt = 0;
				FlyState = FLY;
			}
			if(current_state.mode != "OFFBOARD")				//如果在REST途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			break;
			cout << "REST" <<endl;
		case FLY:
			{
				float phase = 3.1415926;						//起点在X负半轴
				float Omega = 2.0*3.14159*MoveTimeCnt / priod;	//0~2pi
				MoveTimeCnt += 3;								//调此数值可改变飞圆形的速度
				if(MoveTimeCnt >=priod)							//走一个圆形周期
				{
					FlyState = FLYOVER;
				}
				pos_target[0] = temp_pos_drone[0]+desire_Radius*cos(Omega+phase);			 
				pos_target[1] = temp_pos_drone[1]+desire_Radius*sin(Omega+phase); 
				pos_target[2] = desire_z;
				send_pos_setpoint(pos_target, 0);
				if(current_state.mode != "OFFBOARD")			//如果在飞圆形中途中切换到onboard，则跳到WAITING
				{
					FlyState = WAITING;
				}
			}
			cout << "FLY" <<endl;
			break;
		case FLYOVER:
			{
				mode_cmd.request.custom_mode = "AUTO.LAND";
                set_mode_client.call(mode_cmd);
				FlyState = WAITING;
			}
			cout << "FLYOVER" <<endl;
			break;

		default:
			cout << "error" <<endl;
}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "circular_offboard");
	ros::NodeHandle nh;

    //Constructor
    offboard_class_timer offboard_class_timer_node(&nh);//init some param and then start the controller 
	ros::spin();
    return 0;
}
