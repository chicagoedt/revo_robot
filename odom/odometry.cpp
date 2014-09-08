#include "odometry.h"

Odometry::Odometry(void)
 : _x(0.0), _y(0.0), _th(0.0), _r(0.0), _theta(0.0), _vx(0.0), _vy(0.0), _vth(0.0), _deltaX(0.0), _deltaY(0.0), _deltaTh(0.0) 
{
}

void    Odometry::Initialize(void)
{
	_velocity_sub 				= _nh.subscribe("current_velocity", 1, &Odometry::TwistMessageReceived, this);
	_odom_pub  				    = _nh.advertise<nav_msgs::Odometry>("odom", 50);
	_last_time 				    = ros::Time::now();
	
    _timer = _nh.createTimer(ros::Duration(0.2), boost::bind(&Odometry::SendOdomTransform, this));	
}

void    Odometry::SendOdomTransform(void)
{
    _odom_trans.header.frame_id             = "odom";
    _odom_trans.child_frame_id              = "base_footprint";
    _odom_trans.header.stamp                = ros::Time::now(); 
    _odom_trans.transform.translation.x     = _x;
    _odom_trans.transform.translation.y     = _y;
    _odom_trans.transform.translation.z     = 0.0;
    _odom_trans.transform.rotation          = tf::createQuaternionMsgFromYaw(_theta);

    _odom_broadcaster.sendTransform(_odom_trans);
}

void    Odometry::TwistMessageReceived(const geometry_msgs::Twist& msg)
{
	ros::Time _currentTime 		= ros::Time::now();

	double _dt 			= (_currentTime - _last_time).toSec();

	_twistVelocity 			= msg;

	_r       			= _twistVelocity.linear.x;
        _vth     			= _twistVelocity.angular.z;

	_vx      			= _r*cos(_theta);
        _vy      			= _r*sin(_theta);

	_deltaX 			= _vx * _dt;
        _deltaY 			= _vy * _dt;
        _deltaTh			= _vth * _dt;

	_x       			+= _deltaX;
        _y       			+= _deltaY;
        _theta      			+= _deltaTh;

	_odom_quat 			= tf::createQuaternionMsgFromYaw(_theta);
	
	
	//Update the transform
        _odom_trans.header.frame_id             = "odom";
        _odom_trans.child_frame_id              = "base_footprint";
        _odom_trans.header.stamp 		        = _currentTime;
        _odom_trans.transform.translation.x 	= _x;
        _odom_trans.transform.translation.y 	= _y;
        _odom_trans.transform.translation.z 	= 0.0;
        // _odom_trans.transform.rotation 		    = tf::createQuaternionMsgFromYaw(_theta);
    _odom_trans.transform.rotation          = _odom_quat;


	//filling the odometry
        _odom.header.stamp 		    = _currentTime;
        _odom.header.frame_id       = "odom";
        _odom.child_frame_id 		= "base_footprint";

	//position
        _odom.pose.pose.position.x 	    = _x;
        _odom.pose.pose.position.y 	    = _y;
        _odom.pose.pose.position.z 	    = 0.0;
        _odom.pose.pose.orientation 	= _odom_quat;

	//velocity
        _odom.twist.twist.linear.x 	    = _r;
        _odom.twist.twist.linear.y 	    = 0.0;
        _odom.twist.twist.linear.z 	    = 0.0;
        _odom.twist.twist.angular.x 	= 0.0;
        _odom.twist.twist.angular.y 	= 0.0;
        _odom.twist.twist.angular.z 	= _vth;

	
	_odom_pub.publish(_odom);
	_odom_broadcaster.sendTransform(_odom_trans);
	_last_time 			= _currentTime;	
}

