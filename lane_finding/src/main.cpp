// This program does all image processing.

#include <ros/ros.h>
#include <opencv_apps>
#include <sensor_msgs/Image.h>
#include <opencv_apps/LineArrayStamped.h>

void findLanes(const sensor_msgs::CompressedImage frame) {
    //TODO: Convert to grayscale
    
    //TODO: Gaussian blur to reduce noise using kernel size 3 or 5

    //TODO: Canny edge detection

    //TODO: Hough transform

    //TODO: Delete everything outside the region of interest

    //TODO: Perspective transform

int main( int argc, char **argv ) {
    
    // Initialize the ROS system and become a node.
    ros::init( argc, argv, "lane_finder");
    ros::NodeHandle nh;

    // Create publisher and subscriber objects.
    ros::Subscriber left_sub = nh.subscribe("/stereo_camera/left/image_color", 1000, &findLanes);
    ros::Subscriber right_sub = nh.subscribe("/stereo_camera/right/image_color", 1000, &findLanes);
    ros::Publisher left_pub = nh.advertise<sensor_msgs::Image>("lane_detector/lane_lines/left", 1000);
    ros::Publisher left_pub = nh.advertise<sensor_msgs::Image>("lane_detector/lane_lines/right", 1000);
    
    // Let ROS take over.
    ros::spin();
}
