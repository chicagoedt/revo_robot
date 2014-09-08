#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

/* @TODO

	Put the transform offsets of LRF and Camera is ROS Param config file

*/

int main(int argc, char** argv)
{

	ros::init(argc, argv, "robot_tf_publisher");
	ros::NodeHandle n;

	tf::TransformBroadcaster broadcaster;
	
	ros::Rate r(20);

	while(n.ok()) //Check if we can not loop, just send transform data one time, instead of constantly updating a static value. 
	{
			// SLAM PUBLISHES THIS TRANSFORM
			//broadcaster.sendTransform(
			//	tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
  		        //ros::Time::now(),"map", "odom"));	

			broadcaster.sendTransform(
				tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
  		        ros::Time::now(),"base_footprint", "base_link"));

			broadcaster.sendTransform(
				tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.4014, 0.0, 0.1686)),
  		        ros::Time::now(),"base_link", "laser"));

			broadcaster.sendTransform(
				tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1905, 0.0, 0.6112)),
  		        ros::Time::now(),"base_link", "stereo_camera"));

		r.sleep();
	}
}
