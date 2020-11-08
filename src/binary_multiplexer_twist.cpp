#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "controlled_turtlesim-v2/binary_multiplexer.hpp"


int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "binary_multiplexer_twist");
	
	BinaryMultiplexer<geometry_msgs::Twist> multiplexer;
	multiplexer.loop();
	
	return 0;
}


