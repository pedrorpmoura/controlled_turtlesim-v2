#include <ros/ros.h>
#include "binary_multiplexer.hpp"


template<typename T>
BinaryMultiplexer<T>::BinaryMultiplexer(void) {
	
	time = 3;
	
	high_sub = nh.subscribe<T>("high", 10,
		[this](const typename T::ConstPtr &msg) {
			ROS_INFO("Received high priority message");
		});

	low_sub  = nh.subscribe<T>("low",  10,
		[this](const typename T::ConstPtr &msg) {
			ROS_INFO("Received low priority message");
		});

	out_pub  = nh.advertise<T>("out", 10);
}

template<typename T>
void BinaryMultiplexer<T>::loop(void) {

}










