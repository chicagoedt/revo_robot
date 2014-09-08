#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>        // Twist message file
#include <nav_msgs/Odometry.h>

class Odometry
{
	public: 
		virtual ~Odometry(void) {}

		static Odometry&  Instance(void)
		{
			static Odometry INSTANCE;
			return INSTANCE;
		}

	void	Initialize(void);
	void	TwistMessageReceived(const geometry_msgs::Twist& msg);
	void	SendOdomTransform(void);

	private:
		ros::NodeHandle           	_nh;
		ros::Time 		  	_last_time;

		ros::Subscriber 		_velocity_sub;
		tf::TransformBroadcaster  	_odom_broadcaster;
		nav_msgs::Odometry        	_odom;
		ros::Publisher 		  	_odom_pub;
		geometry_msgs::TransformStamped _odom_trans;
		geometry_msgs::Quaternion       _odom_quat;
		geometry_msgs::Twist 	  	_twistVelocity; // Vel from cmd_vel
		ros::Timer 			            _timer;
		double 		     	  	_x; // Change in X direction on coordinateframe
		double 			  	_y; // Change in X direction on coordinateframe
		double 			  	_th; // Change in orientation on CF
		double			  	_r; // Linear X velocity (forward/back)
		double			  	_theta; // Angular velocity (turning)
		double 			  	_vx; // Change in x direction from cos(th)
		double 			 	_vy; // Change in y direction from sin(th)
		double 			  	_vth; // NEED TO FIGURE OUT
		double			  	_deltaX; 
		double			  	_deltaY;
		double			  	_deltaTh;	
		
		Odometry(void);
};

#endif
