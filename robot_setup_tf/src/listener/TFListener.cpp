#include "TFListener.h"

TFListener::TFListener(void)
 : _listener(ros::Duration(10))
{
}

void TFListener::Initialize(void)
{
	_odomSub = _nh.subscribe("odom", 1, &TFListener::odomCallback, this);

    //we'll transform a point once every second
    _timer = _nh.createTimer(ros::Duration(0.5), boost::bind(&TFListener::transformPoint, this, boost::ref(_listener)));
}

void TFListener::odomCallback(const nav_msgs::Odometry& odomPoint)
{
	_odomPoint.point.x	=	odomPoint.pose.pose.position.x;
	_odomPoint.point.y	=	odomPoint.pose.pose.position.y;
	_odomPoint.point.z  =   odomPoint.pose.pose.position.z;
}

void TFListener::transformPoint(const tf::TransformListener& listener)
{
	_laser_point.header.frame_id    = "laser";
	_camera_point.header.frame_id   = "base_camera";
	_base_point.header.frame_id     = "base_footprint";
	_odomPoint.header.frame_id      = "odom";
	
	_laser_point.header.stamp       = ros::Time();
    	_camera_point.header.stamp      = ros::Time();
    	_base_point.header.stamp        = ros::Time();
    	_odomPoint.header.stamp         = ros::Time();

	try	//Try transforming the LRF
	{
		geometry_msgs::PointStamped baseLinkToBaseLaser;
        	listener.transformPoint("base_link", _laser_point, baseLinkToBaseLaser);
	}
	catch(tf::TransformException& ex)
    	{
          	ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
    	}

	try	//Try Transforming the camera
    	{
		geometry_msgs::PointStamped baseLinkToBaseCamera;
            	listener.transformPoint("base_link", _camera_point, baseLinkToBaseCamera);
            		//1. (Frame being applied offset)
			//2. (Point from sensor)
			//3. ("returned" Point with applied offset) 
     	}
     	catch(tf::TransformException& ex)
     	{
          	ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
     	}
}










