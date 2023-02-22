#include <offboard_class_timer/offboard_class_timer.h>

offboard_class_timer::offboard_class_timer(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    calc_timer = nh.createTimer(ros::Duration(0.05), &offboard_class_timer::calc_cb, this);  //timer used to publish state, should be at least for some minimal frequency

}

void offboard_class_timer::init_publisher()
{
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    ("mavros/setpoint_position/local", 10,this);
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
	ROS_INFO("%d", current_state.connected);
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    //在切换到offboard模式之前，你必须先发送一些期望点信息到飞控中。不然飞控会拒绝切换到offboard模式。在这里，100可以是随意设定的一个值。
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
    }
	ROS_INFO("1234:%d", current_state.connected);

	//  if(current_state.connected)
	 if(arm_cmd.request.value)
     {
        // offb_set_mode.request.custom_mode = "OFFBOARD";
        // arm_cmd.request.value = true;

        last_request = ros::Time::now();
        if (current_state.mode != "OFFBOARD" &&
			(ros::Time::now() - last_request > ros::Duration(5.0))) {
			if (set_mode_client.call(offb_set_mode) &&
				offb_set_mode.response.mode_sent) {
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		}
		else {
			if (!current_state.armed &&
				(ros::Time::now() - last_request > ros::Duration(5.0))) {
				if (arming_client.call(arm_cmd) &&
					arm_cmd.response.success) {
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
            else
            {
                switch (step)
				{
				case 0:
					//take off to 2m
					pose.pose.position.x = 0;
					pose.pose.position.y = 0;
					pose.pose.position.z = 2;
					//
					if (local_pos.pose.position.z > 1.9 && local_pos.pose.position.z < 2.1)
					{
						if (sametimes > 100)
						{
							
							sametimes = 0;
							step = 1;
							pose.pose.position.x = 2;
							pose.pose.position.y = 0;
							pose.pose.position.z = 2;
						}
						else
							sametimes++;
					}
					else
					{
						sametimes = 0;
					}
					local_pos_pub.publish(pose);
					break;
				case 1:
					
					if (local_pos.pose.position.x > 1.9 && local_pos.pose.position.x < 2.1)
					{
						if (sametimes > 100)
						{
							
							step = 2;
							pose.pose.position.x = 2;
							pose.pose.position.y = 2;
							pose.pose.position.z = 2;
						}
						else
							sametimes++;
					}
					else
					{
						sametimes = 0;
					}
					local_pos_pub.publish(pose);
					break;
				case 2:
					
					if (local_pos.pose.position.y > 1.9 && local_pos.pose.position.y < 2.1)
					{
						if (sametimes > 100)
						{
							
							step = 3;
							pose.pose.position.x = 0;
							pose.pose.position.y = 2;
							pose.pose.position.z = 2;
						}
						else
							sametimes++;
					}
					else
					{
						sametimes = 0;
					}
					local_pos_pub.publish(pose);
					break;
				case 3:
					
					if (local_pos.pose.position.x > -0.1 && local_pos.pose.position.x < 0.1)
					{
						if (sametimes > 100)
						{
							
							step = 4;
							pose.pose.position.x = 0;
							pose.pose.position.y = 0;
							pose.pose.position.z = 2;
						}
						else
							sametimes++;
					}
					else
					{
						sametimes = 0;
					}
					local_pos_pub.publish(pose);
					break;
				case 4:
					
					if (local_pos.pose.position.y > -0.1 && local_pos.pose.position.y < 0.1)
					{
						if (sametimes > 100)
						{
							step = 5;
						}
						else
							sametimes++;
					}
					else
					{
						sametimes = 0;
					}
					local_pos_pub.publish(pose);
					break;
				case 5:
					offb_set_mode.request.custom_mode = "AUTO.LAND";
					if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
					{

						if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
						{
							ROS_INFO("AUTO.LAND enabled");
						}
						last_request = ros::Time::now();
					}
					break;
				default:
					break;
				}
            }
        }
     }
     else{
         ROS_INFO_STREAM("unconnected");
     }
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
