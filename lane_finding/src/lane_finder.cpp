// This program does all image processing.

#include "lane_finder.h"
#include <signal.h>

sensor_msgs::CompressedImage LaneFinder::findLanes(const sensor_msgs::Image& msg) {

    // Convert sensor_msgs/Image to Mat
    cv_bridge::CvImagePtr in_msg;
    ROS_INFO_STREAM("SHIT");
  
    try {
        in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        //return;
    }

    ROS_INFO_STREAM("POOP");
    cv::Mat frame;
    frame = in_msg->image;
    
    //TODO: Undistort
    // http://docs.opencv.org/2.4/doc/tutorials/core/file_input_output_with_xml_yml/file_input_output_with_xml_yml.html#fileinputoutputxmlyaml 
    /*
    // Resize.
    cv::Mat frame;
    cv::resize(full_frame, frame, cv::Size(160,90));
    full_frame.release();
    */
    //TODO: Gaussian blur to reduce noise using kernel size 3 or 5

    // Convert to grayscale and HSV
    cv::Mat gray;
    cv::Mat hsv;
    //cv::cvtColor(frame, gray, CV_BGR2GRAY);
 /*   cv::cvtColor(frame, hsv, CV_BGR2HSV);
    //frame.release();

    // Isolate S channel of HLS and run Sobel over gray.
    cv::Mat saturation;
    cv::extractChannel(hsv, saturation, 1);
    //hsv.release();
    
    cv::Mat grad_x;
    cv::Sobel(gray, grad_x, CV_16S, 1, 0);
    //gray.release();

    //TODO: Hough transform

    //TODO: Delete everything outside the region of interest

    //TODO: Perspective transform    

    //TODO: Convert back to sensor_msgs/Image
    sensor_msgs::CompressedImage out_msg;
    out_msg.header = in_msg->header;
    out_msg.format = "png";
    out_msg.data = grad_x;
*/
    sensor_msgs::CompressedImage out_msg;
    return out_msg;

}

void LaneFinder::left_callback(const sensor_msgs::Image& msg) {
    sensor_msgs::CompressedImage lanes = findLanes(msg);
    _left_pub.publish(lanes);
}

void LaneFinder::right_callback(const sensor_msgs::Image& msg) {
    sensor_msgs::CompressedImage lanes = findLanes(msg);
    _right_pub.publish(lanes);
}

void LaneFinder::Initialize() {

    // Create publisher and subscriber objects.
    _left_sub = _nh.subscribe("/stereo_camera/left/image_color", 1000, &LaneFinder::left_callback, this);
    _right_sub = _nh.subscribe("/stereo_camera/right/image_color", 1000, &LaneFinder::right_callback, this);
    _left_pub = _nh.advertise<sensor_msgs::CompressedImage>("lane_finder/lane_lines/left", 1000);
    _right_pub = _nh.advertise<sensor_msgs::CompressedImage>("lane_finder/lane_lines/right", 1000);
}

/*
 void sigIntHandler(int sig) {
     ROS_DEBUG("--> SIGINT Handler called <--");
 
     ros::shutdown();
 }
 */
 int main(int argc, char** argv) {
     ros::init(argc, argv, "lane_finder");
 
     LaneFinder laneFinder;
 
    // signal(SIGINT, sigIntHandler);
 
     laneFinder.Initialize();
 
     ros::spin();
 }

