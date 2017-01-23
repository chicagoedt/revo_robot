#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int rt_callback(const sensor_msgs::Joy& joystick_input) {
    // Button Index: http://wiki.ros.org/joy#Application
    cmd_vel_

}

int main(int argc char **argv) {
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "revo_teleop");
    ros::NodeHandle nh;

    // Create a publisher object.
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(
            "cmd_vel", 1000);

    // Tell joy which js device to listen to.
    // TODO: Pass this as a param or find a way to set it automatically.
    // nh.setParam("joy_node/dev", "/dev/input/js1");

    // Create a subscriber object.
    ros::Subscriber sub = nh.subscribe("/base_controller/command", 1000, &rt_callback);

    // Let ROS take over.
    ros::spin();

    return 0;
}
