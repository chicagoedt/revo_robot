// This program does stuff

#include <ros/ros.h>
#include <opencv_apps>
#include <sensor_msgs/Image.h>
#include <opencv_apps/LineArrayStamped.h>

void findLanes(const sensor_msgs::CompressedImage frame) {


int main( int argc, char **argv ) {
    
    // Initialize the ROS system and become a node.
    ros::init( argc, argv, "lane_finder");
    ros::NodeHandle nh;

    // Create publisher and subscriber objects.
    ros::Publisher lane_pub = nh.advertise<sensor_msgs::Image>("lane_detector/lane_lines", 1000);
    ros::Subscriber hough_sub = nh.subscribe("opencv_apps/hough_lines`", 1000, &findLanes);
    
    // Let ROS take over.
    ros::spin();
}
