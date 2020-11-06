#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// #include "binary_multiplexer.hpp"


class BinaryMultiplexerTwist {
	
	public:
		BinaryMultiplexerTwist(void);
		void loop(void);

	private:
		bool allow_low_messages = false;
		
		ros::Timer timer;
		ros::NodeHandle nh;
		ros::Subscriber high_sub;
		ros::Subscriber low_sub;
		ros::Publisher out_pub;
};


BinaryMultiplexerTwist::BinaryMultiplexerTwist(void) {
	
	// get time parameter
	int time;
	ros::param::get("~time", time);
	
	// setup timer
	timer = nh.createTimer(ros::Duration(time),
		[this](const ros::TimerEvent&) {
			ROS_INFO("TIMER FINISHED. Allowing low priorities messages.");
			allow_low_messages = true;
			timer.stop();
		});

	// setup subscribers
	high_sub = nh.subscribe<geometry_msgs::Twist>("high", 10,
		[this](const geometry_msgs::Twist::ConstPtr &msg) {
			ROS_INFO("RECEIVED high priority message");
			allow_low_messages = false;
			out_pub.publish(msg);
			
			// restart timer
			timer.stop();
			timer.start();
		});

	low_sub  = nh.subscribe<geometry_msgs::Twist>("low",  10,
		[this](const geometry_msgs::Twist::ConstPtr &msg) {
			if (allow_low_messages) {
				ROS_INFO("RELAYING low priority message");
				out_pub.publish(msg);	
			}
		});
	
	// setup publisher
	out_pub  = nh.advertise<geometry_msgs::Twist>("out", 10);
}


void BinaryMultiplexerTwist::loop(void) {
	ros::spin();
}


int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "binary_multiplexer_twist");
	
	BinaryMultiplexerTwist multiplexer;
	multiplexer.loop();
	
	return 0;
}


