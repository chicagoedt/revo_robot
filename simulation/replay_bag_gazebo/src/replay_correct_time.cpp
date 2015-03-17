#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub;

void chatterCallback(const sensor_msgs::LaserScan& msg)
{
	sensor_msgs::LaserScan tmp = msg;
	tmp.header.stamp = ros::Time::now();
	pub.publish(tmp); 
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "rosbag_replay");

	ros::NodeHandle n;
  
	pub = n.advertise<sensor_msgs::LaserScan>("/scan", 1);
	ros::Subscriber sub = n.subscribe("/scan_bag", 1000, &chatterCallback);

	ros::spin();

  	return 0;
}
