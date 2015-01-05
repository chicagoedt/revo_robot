/*
 * ROS node that uses Marcos Nieto's LSWMS code
 * Author: Basheer Subei
 *
 * Original Comment Header by Marcos:
 * Project:  lineSegments (LSWMS Line Segment using Weighted Mean-Shift)
 *
 * File:     main.cpp
 *
 * Contents: Creation, initialisation and usage of LSWMS object
 *           for the detection of line segments in images or videos
 *
 * Author:   Marcos Nieto <marcos.nieto.doncel@gmail.com>
 *
 * Homepage: www.marcosnieto.net
 */

#include "LSWMS.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#ifdef WIN32
	#include <windows.h>
	#include <time.h>
#endif

#ifdef linux
	#include <stdio.h>
	#include <sys/time.h>
	#include <time.h>
#endif

#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

using namespace cv;

image_transport::Publisher pub;
image_transport::Subscriber sub;

// Timing
#ifdef WIN32
	double t1, t2;	
#else
	int t1, t2;	
	struct timeval ts;
#endif
double t;

// TODO make it a dynamic_reconfigure server and rosparam topic names

// subscriber image callback that applies lswms on ROS image and publishes another
// image with overlayed lines
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
    	// TODO support mono8 images as well
    	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
    	ROS_ERROR("cv_bridge exception: %s", e.what());
    	return;
    }

    int width = cv_ptr->image.cols;
    int height = cv_ptr->image.rows;

	// Create and init LSWMS
	bool verbose = false;
	int numMaxLSegs = 0;
	cv::Size procSize = cv::Size(width, height);
	int R = 3;
	LSWMS lswms(procSize, 3, 0, false);

    // Color Conversion

    std::vector<LSEG> lSegs;
    std::vector<double> errors;

    // ++++++++++++++++++++++++++++++++++++++++
    // Process LSWMS
    #ifdef WIN32
        t1 = ::GetTickCount();
    #else
        gettimeofday(&ts,0);
        t1 = (ts.tv_sec * 1000 + (ts.tv_usec / 1000));
    #endif
    lswms.run(cv_ptr->image, lSegs, errors);             
    #ifdef WIN32
        t2 = ::GetTickCount();
    #else
        gettimeofday(&ts,0);
        t2 = (ts.tv_sec * 1000 + (ts.tv_usec / 1000));
    #endif  

    // process time = t2 - t1       
    t = (double)t2-(double)t1;
    cv::Scalar mean, stddev;
    cv::meanStdDev(errors, mean, stddev);
    printf("LSWMS: %d segments\nAngular Error: Mean = %.2f (deg), Std = %.2f (deg)\nProcess Time = %.0f (ms)\n", (int)(lSegs.size()), mean.val[0]*180/CV_PI, stddev.val[0]*180/CV_PI,  t);
    lswms.drawLSegs(cv_ptr->image, lSegs, errors);              // drawing according to errors

    // now convert CV image to ROS image and publish it
    pub.publish(cv_ptr->toImageMsg());

}

/** Main function*/
int main(int argc, char** argv)
{	

	// initialize ROS node
	ros::init(argc, argv, "lswms");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	// publisher
	pub = it.advertise("/lswms/line_image", 1);
    // subscriber
    sub = it.subscribe("/brightestpixel_lanedetection/line_image", 1, imageCallback);

	ros::spin();

	return 0;
}
