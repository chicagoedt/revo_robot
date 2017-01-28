#ifndef Lane_Finder_h
#define Lane_Finder_h

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class LaneFinder
{
    
    public:
        
        void Initialize();

    private:

        ros::NodeHandle     _nh;

        ros::Publisher      _right_pub;
        ros::Publisher      _left_pub;

        ros::Subscriber     _left_sub;
        ros::Subscriber     _right_sub;

        sensor_msgs::CompressedImage findLanes(const sensor_msgs::Image);
        void left_callback(sensor_msgs::Image);
        void right_callback(sensor_msgs::Image);
};

#endif
