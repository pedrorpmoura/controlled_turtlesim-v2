#include <ros/ros.h>
#include "controlled_turtlesim_v2/binary_multiplexer.hpp"
#include <std_msgs/Bool.h>


template<typename T>
BinaryMultiplexer<T>::BinaryMultiplexer(void) {
	
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
	high_sub = nh.subscribe<T>("high", 10,
		[this](const typename T::ConstPtr &msg) {
			ROS_INFO("Received high priority message");
			allow_low_messages = false;
            out_pub.publish(msg);

            // restart timer
            timer.stop();
            timer.start();
		});

	low_sub  = nh.subscribe<T>("low",  10,
		[this](const typename T::ConstPtr &msg) {
			if (allow_low_messages) {
				ROS_INFO("Received low priority message");
				out_pub.publish(msg);	
			}
		});
	

	// setup publisher
	out_pub  = nh.advertise<T>("out", 10);
}


template<typename T>
void BinaryMultiplexer<T>::loop(void) {
	ros::spin();
}

