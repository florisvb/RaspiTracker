#include "ros/ros.h"
#include "pi_rotary_encoder/ParamUpdate.h"
#include "stdio.h"
extern "C" { 
#include "rotaryencoder.h"
#include "wiringPi.h"
}

bool rootREVOKED = 0;

static void revokeRoot(void) 
{
	if (getuid () + geteuid () == 0)
		return ;
	if (getuid () == 0)
	{
	seteuid(getuid ());
	rootREVOKED = 1;
	return ;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<pi_rotary_encoder::ParamUpdate>("parameter_update", 1000);

	ros::Rate loop_rate(10);

	//int r = wiringPiSetup(); 
	
	int r = wiringPiSetup();
	revokeRoot();
	printf("setup: %d\n", r);
	// wiringPiSetup requires sudo, wiringPiSetupSys requires you add this to your /etc/rc.local:
	//
	// for example, to use pin 23 as an input:
	//
	// gpio export 23 in
	// 
	struct encoder *encoder = setupencoder(4,5);
	long value;

	while (ros::ok())
	{
		updateEncoders();
		long l = encoder->value;
		if (l!=value)
		{
			printf("value: %d\n", (void *)l);
			value = l;
			pi_rotary_encoder::ParamUpdate msg;
			msg.parameter = "shutter_speed";
			msg.value = value*10;
			pub.publish(msg);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
}
