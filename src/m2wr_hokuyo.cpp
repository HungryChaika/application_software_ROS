#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define PI 3.1415926f

void msgCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
	int distArrLength = 10;
	float distArr[distArrLength];
	float angleIncrement;
	angleIncrement = msg->angle_increment;
	for(int i = 0; i < distArrLength; i++)
	{
 		distArr[i] = msg->ranges[i];
		ROS_INFO("angle: %f, dist: %f", i*angleIncrement*180.0f/PI, distArr[i]);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "m2wr_hokyuyo");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("m2wr/laser/scan", 100, msgCallback);
    ros::spin();
	return 0;
}