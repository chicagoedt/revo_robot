#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <iomanip>

void MessageReceived(const sensor_msgs::NavSatFix& msg) {
	ROS_INFO_STREAM("Latitude: " << msg.latitude 
	  << "Longitude: " << msg.longitude);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "gps_subscriber_node");
	ros::NodeHandle nh;

	ros::Subscriber gps_sub = nh.subscribe("/gps/filtered", 1000,
	  &MessageReceived);

	ros::spin();
}

