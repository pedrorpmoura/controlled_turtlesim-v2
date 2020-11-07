#include <ros/ros.h>
#include <geometry_msgs/Twist.h>



class RandomMover {
	
	public:
		RandomMover(void);
		void loop(void);

	private:
		ros::NodeHandle nh;
		ros::Publisher pub;

		geometry_msgs::Twist calculateCmdVelMsg(void);
};


geometry_msgs::Twist RandomMover::calculateCmdVelMsg(void) {
	
	geometry_msgs::Twist msg;
	msg.linear.x = 1;
	msg.angular.z = static_cast<float> (rand()) / (static_cast<float> (RAND_MAX / M_PI)) - (M_PI/2);

	return msg;
}


RandomMover::RandomMover(void) {
	
	// setup publisher
	pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

}


void RandomMover::loop(void) {
	
	ros::Rate rate(1);
	while (ros::ok()) {
		geometry_msgs::Twist msg;
		msg = calculateCmdVelMsg();

		pub.publish(msg);
		rate.sleep();
		ros::spinOnce();
	}
}




int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "random_mover");
	
	RandomMover random_mover;
	random_mover.loop();

	return 0;
}












