#ifndef	__TFLISTENER_H__
#define	__TFLISTENER_H__

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

class TFListener
{
	public:
		virtual ~TFListener(void) {}

		static TFListener&  Instance(void)
                {
                        static TFListener INSTANCE;
                        return INSTANCE;
                }

		void transformPoint(const tf::TransformListener& listener);
		void odomCallback(const nav_msgs::Odometry& odomPoint);	
		void cameraCallback(const sensor_msgs::PointCloud& cameraPoint);
		void Initialize(void);
		//Check sick_tim3xx package if transform for LRF is being taken care of

	private:
		ros::NodeHandle                 _nh;
        tf::TransformListener           _listener;
		ros::Subscriber 		        _odomSub;
		geometry_msgs::PointStamped    	_laser_point;	
		geometry_msgs::PointStamped    	_camera_point;
		geometry_msgs::PointStamped   	_base_point;	
		geometry_msgs::PointStamped 	_odomPoint;
		ros::Timer 			            _timer;

		TFListener(void);		
};

#endif
