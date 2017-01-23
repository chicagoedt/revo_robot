#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <base_controller/XBox_Button_Msg.h>

int main(int argc char **argv) {
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "revo_teleop");
    ros::NodeHandle nh;

    // Create a publisher object.
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
            "TODO/cmd_vel", 1000);

    // Tell joy which js device to listen to.
    // TODO: Pass this as a param or find a way to set it automatically.
    ros::Subscriber sub = nh.subscribe("joy", 1000, &rt_callback);

    // Let ROS take over.
    ros::spin();
}
