#ifndef Lane_Finder_h
#define Lane_Finder_h

#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class LaneFinder
{
    
    public:
        
        void Initialize();

    private:

        ros::NodeHandle     _nh;

        ros::Publisher      _right_pub;
        ros::Publisher      _left_pub;
        ros::Publisher      _hue_pub;
        ros::Publisher      _saturation_pub;
        ros::Publisher      _value_pub;
        ros::Publisher      _intensity_pub;

        ros::Subscriber     _left_sub;
        ros::Subscriber     _right_sub;

        cv::Mat findLanes(const sensor_msgs::Image&);
        void left_callback(const sensor_msgs::Image&);
        void right_callback(const sensor_msgs::Image&);
        void publishImage(cv::Mat&, ros::Publisher&);
};

#endif
