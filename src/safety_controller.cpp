#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>




bool turtleInCenter(turtlesim::Pose pose) {
	
	return pose.x > 5 and pose.x < 6 and pose.y > 5 and pose.y < 6;
}

geometry_msgs::Twist calculateCmdVelMsg(turtlesim::Pose pose) {
	
	geometry_msgs::Twist msg;

	// align turtle with center
	float alpha = atan2(pose.y - 5.5, pose.x - 5.5);
	
	if (alpha < 0) {
		alpha += 2 * M_PI;
	}
	
	if (pose.theta < 0) {
		pose.theta += 2 * M_PI;
	}
	
	
	msg.angular.z = alpha - pose.theta;
	
	msg.linear.x = -2;

	return msg;
}

class SafetyController {
	
	public:
		SafetyController(void);
		void loop(void);
		
	private:
		bool safety_activated = false;
		turtlesim::Pose turtle_pose;

		ros::NodeHandle nh;
		ros::Subscriber pose_sub;
		ros::Publisher cmd_vel_pub; 
};


SafetyController::SafetyController(void) {
	
	// setup subscriber
	pose_sub = nh.subscribe<turtlesim::Pose>("pose", 10,
		[this](const turtlesim::Pose::ConstPtr &msg) {
			float x = msg->x;
			float y = msg->y;
			if (x < 1.0 or x > 10.0 or y < 1.0 or y > 10.0) {
				ROS_INFO("SAFETY INITATED");
				safety_activated = true;
			}

			turtle_pose.x = x;
			turtle_pose.y = y;
			turtle_pose.theta = msg->theta;
			turtle_pose.linear_velocity = msg->linear_velocity;
			turtle_pose.angular_velocity = msg->angular_velocity;
		});

	// setup publisher
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
}


void SafetyController::loop(void) {
	
	ros::Rate rate(10);
	while (ros::ok()) {
		
		bool in_center = turtleInCenter(turtle_pose);
		if (safety_activated and not in_center) {
			geometry_msgs::Twist msg = calculateCmdVelMsg(turtle_pose);
			cmd_vel_pub.publish(msg);
		}

		if (in_center) {
			safety_activated = false;
		}

		rate.sleep();
		ros::spinOnce();
	}
}


int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "safety_controller");
	
	SafetyController controller;
	controller.loop();

	return 0;
}







