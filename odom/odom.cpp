#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>        // Twist message file
#include <nav_msgs/Odometry.h>

geometry_msgs::Twist twist_velocity;

// Callback function, grabs each Twist message
void twistMessageReceived(const geometry_msgs::Twist& msg) 
{
    twist_velocity = msg;
}

int main(int argc, char **argv) 
{

    ros::init(argc, argv, "sub_to_wheels_twist");

    ros::NodeHandle nh;
    ros::Subscriber velocity_sub    = nh.subscribe("current_velocity", 10, &twistMessageReceived);
    ros::Publisher  odom_pub        = nh.advertise<nav_msgs::Odometry>("raw_odom", 50);

    // Initial position is at 0,0
    double x    = 0.0;
    double y    = 0.0;
    double th   = 0.0;

    float r             = 0.0;
    float theta         = 0.0;
    double vx           = 0.0;
    double vy           = 0.0;
    double vth          = 0.0;

    double delta_x      = 0.0;
    double delta_y      = 0.0;
    double delta_th     = 0.0;

    ros::Time last_time = ros::Time::now();       

    //tf::TransformBroadcaster    odom_broadcaster;
    ros::Rate                   loop_rate(20);

    //geometry_msgs::TransformStamped odom_trans;
    //odom_trans.header.frame_id      = "wheelodom";
    //odom_trans.child_frame_id       = "base_link";

    geometry_msgs::Quaternion       odom_quat;

    nav_msgs::Odometry odom;
    

    while(ros::ok())
    {
        ros::Time current_time    = ros::Time::now();

        double dt   = (current_time - last_time).toSec();

        r       = twist_velocity.linear.x;
        vth     = twist_velocity.angular.z;

        // Calculate the velocities
        vx      = r*cos(theta);
        vy      = r*sin(theta);

        delta_x = vx * dt;
        delta_y = vy * dt;
        delta_th= vth * dt;

        //delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
        //delta_y = (vx * sin(theta) - vy * cos(theta)) * dt;
        //delta_th= (vth * dt);

        x       += delta_x;
        y       += delta_y;
        theta      += delta_th;

        odom_quat = tf::createQuaternionMsgFromYaw(theta);

        //Update the transform
        //odom_trans.header.frame_id          = "base_footprint";
        //odom_trans.child_frame_id           = "base_link";
        //odom_trans.header.stamp             = current_time;
        //odom_trans.transform.translation.x  = x;
        //odom_trans.transform.translation.y  = y;
        //odom_trans.transform.translation.z  = 0.0;
        //odom_trans.transform.rotation       = odom_quat;

        //filling the odometry
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.covariance[0] = 0.00001;
        odom.pose.covariance[7] = 0.00001;
        odom.pose.covariance[14] = 1000000000000.0;
        odom.pose.covariance[21] = 1000000000000.0;
        odom.pose.covariance[28] = 1000000000000.0;
        odom.pose.covariance[35] = 0.001;
          
	    //position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
            //velocity
        odom.twist.twist.linear.x   = r;
        odom.twist.twist.linear.y   = 0.0;
        odom.twist.twist.linear.z   = 0.0;
        odom.twist.twist.angular.x  = 0.0;
        odom.twist.twist.angular.y  = 0.0;
        odom.twist.twist.angular.z  = vth;

        last_time = current_time;

        //odom_broadcaster.sendTransform(odom_trans);
        odom_pub.publish(odom);

        loop_rate.sleep();
        ros::spinOnce();

    }
}


/*
#include "odometry.h"

int main(int argc, char **argv) 
{
    //  Initialize the ROS system and become a node.?

    ros::init(argc, argv, "subscribe_to_twist");

    Odometry::Instance().Initialize();

    ros::spin();

    return 0;

    // Initial position is at 0,0
	/*
    double x    = 0.0;
    double y    = 0.0;
    double th   = 0.0;

    float r             = 0.0;
    float theta         = 0.0;
    double vx           = 0.0;
    double vy           = 0.0;
    double vth          = 0.0;

    double dt           = 0.0;
    double delta_x      = 0.0;
    double delta_y      = 0.0;
    double delta_th     = 0.0;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    ros::Rate                   loop_rate(20);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id      = "odom";
    odom_trans.child_frame_id       = "base_footprint";

    geometry_msgs::Quaternion       odom_quat;
*/
//    while(ros::ok())
//    {
/*
        current_time    = ros::Time::now();
        dt              = (current_time - last_time).toSec();

        r       = twist_velocity.linear.x;
        vth     = twist_velocity.angular.z;

        

        // Calculate the velocities
        vx      = r*cos(theta);
        vy      = r*sin(theta);

        delta_x = vx * dt;
        delta_y = vy * dt;
        delta_th= vth * dt;

        x       += delta_x;
        y       += delta_y;
        th      += delta_th;

        odom_quat = tf::createQuaternionMsgFromYaw(theta);

        //Update the transform
        odom_trans.header.frame_id              = "odom";
        odom_trans.child_frame_id               = "base_footprint";
        odom_trans.header.stamp                 = current_time;
        odom_trans.transform.translation.x      = x;
        odom_trans.transform.translation.y      = y;
        odom_trans.transform.translation.z      = 0.0;
        odom_trans.transform.rotation           = odom_quat;

        //filling the odometry
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        //position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //velocity
        odom.twist.twist.linear.x = r;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = vth;

        last_time = current_time;

        odom_broadcaster.sendTransform(odom_trans);
        odom_pub.publish(_odom);
	*/

        // loop_rate.sleep();
        //ros::spinOnce();

  //  }
//} REMOVE THE COMMENT



