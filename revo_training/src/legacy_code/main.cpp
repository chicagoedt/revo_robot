// IGNORE THIS PROGRAM. I went with a different approach and haven't gotten around to deleting this.

// This program creates a node which subscribes to the video feeds from the Zed camera and to cmd_vel.
// The cmd_vel subscriber's callback writes the values to a global variable.
// The sen

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/CompressedImage.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "revo_training_recorder");
    ros::NodeHandle nh;

    ros::Subscriber zed_sub = nh.subscribe("cmd_vel", 1000, 
