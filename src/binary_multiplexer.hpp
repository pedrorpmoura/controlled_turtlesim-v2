#include <ros/ros.h>

template<typename T>
class BinaryMultiplexer {
	
	public:
		BinaryMultiplexer(void);
		void loop(void);

	private:
		int time;
		
		ros::NodeHandle nh;
		ros::Subscriber high_sub;
		ros::Subscriber low_sub;
		ros::Publisher out_pub;
};
