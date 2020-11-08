#pragma once
#include <ros/ros.h>

template <typename T>
class BinaryMultiplexer {
	
	public:
		BinaryMultiplexer(void);
		void loop(void);
		
	private:
		bool allow_low_messages = false;
		
		ros::Timer timer;
		ros::NodeHandle nh;
		ros::Subscriber high_sub;
		ros::Subscriber low_sub;
		ros::Publisher out_pub;
};
