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
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "offb_node");
	ros::NodeHandle nh;

    //Constructor
    offboard_class_timer offboard_class_timer_node(&nh);//init some param and then start the controller 
	ros::spin();
    return 0;
}
