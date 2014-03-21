#include "ros/ros.h"
#include "pi_rotary_encoder/ParamUpdate.h"

void chatterCallback(const pi_rotary_encoder::ParamUpdate::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s, %d]", msg->parameter.c_str(), msg->value);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("parameter_update", 1000, chatterCallback);

	ros::spin();

}
