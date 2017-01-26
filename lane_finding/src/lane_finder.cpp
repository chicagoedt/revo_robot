// This program does all image processing.

#include <ros/ros.h>
//#include <opencv_apps>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
//#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


void findLanes(const sensor_msgs::Image msg) {

    // Convert sensor_msgs/Image to Mat
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat full_frame = cv_ptr->image;
    
    //TODO: Undistort
    // http://docs.opencv.org/2.4/doc/tutorials/core/file_input_output_with_xml_yml/file_input_output_with_xml_yml.html#fileinputoutputxmlyaml 

    // Resize.
    cv::Mat frame;
    cv::resize(full_frame, frame, cv::Size(160,90));
    full_frame.release();


    //TODO: Gaussian blur to reduce noise using kernel size 3 or 5

    // Convert to grayscale and HSV
    cv::Mat gray;
    cv::Mat hsv;
    cv::cvtColor(frame, gray, CV_BGR2GRAY);
    cv::cvtColor(frame, hsv, CV_BGR2HSV);
    frame.release();

    // Isolate S channel of HLS and run Sobel over gray.
    cv::Mat saturation;
    cv::extractChannel(hsv, saturation, 1);
    hsv.release();
    
    cv::Mat grad_x;
    cv::Sobel(gray, grad_x, CV_16S, 1, 0);
    gray.release();

    //TODO: Hough transform

    //TODO: Delete everything outside the region of interest

    //TODO: Perspective transform
}

int main( int argc, char **argv ) {
    
    // Initialize the ROS system and become a node.
    ros::init( argc, argv, "lane_finder");
    ros::NodeHandle nh;

    // Create publisher and subscriber objects.
    ros::Subscriber left_sub = nh.subscribe("/stereo_camera/left/image_color", 1000, &findLanes);
    ros::Subscriber right_sub = nh.subscribe("/stereo_camera/right/image_color", 1000, &findLanes);
    ros::Publisher left_pub = nh.advertise<sensor_msgs::Image>("lane_detector/lane_lines/left", 1000);
    ros::Publisher right_pub = nh.advertise<sensor_msgs::Image>("lane_detector/lane_lines/right", 1000);
    
    // Let ROS take over.
    ros::spin();
}
