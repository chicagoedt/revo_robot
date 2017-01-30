// This program does all image processing.

#include "lane_finder.h"
#include <signal.h>
#include <cmath> // for abs()

sensor_msgs::Image LaneFinder::findLanes(const sensor_msgs::Image& msg) {

    // Convert sensor_msgs/Image to Mat
    cv_bridge::CvImagePtr in_msg;
  
    try {
        in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        //return;
    }

    cv::Mat full_frame;
    full_frame = in_msg->image;
    
    //TODO: Undistort
    // http://docs.opencv.org/2.4/doc/tutorials/core/file_input_output_with_xml_yml/file_input_output_with_xml_yml.html#fileinputoutputxmlyaml 
    
    // Resize. Default Zed dimensions are 1600x900
    cv::Mat frame;
    int newCols = 500;
    double ratio = (double)full_frame.rows / (double)full_frame.cols;
    cv::resize(full_frame, frame, cv::Size(newCols,newCols*ratio));
    full_frame.release();
    
    //TODO: Gaussian blur to reduce noise using kernel size 3 or 5

    // Convert to grayscale and HSV
    cv::Mat gray;
    cv::Mat hsv;
    cv::cvtColor(frame, gray, CV_BGR2GRAY);
    cv::cvtColor(frame, hsv, CV_BGR2HSV);
    //frame.release();

    // Isolate S channel of HLS and run Sobel over gray.
    cv::Mat saturation;
    cv::extractChannel(hsv, saturation, 1);
    //hsv.release();
    
    cv::Mat sobelX, sobelY;
    cv::Sobel(gray, sobelX, CV_8U, 1, 0);
    cv::Sobel(gray, sobelY, CV_8U, 0, 1);

    // Threshold the magnitude and direction of the Sobel-ized image.
    cv::Mat abs_sobelX; 
    abs_sobelX = abs(sobelX);

    cv::Mat sobel_mag, sobel_dir;
    sobel_dir = gray;
    cv::threshold(abs_sobelX, sobel_mag, 25.0, 255.0, cv::THRESH_BINARY);
    for (int i=0; i < gray.rows; i++) {
        for (int j=0; j < gray.cols ; j++) {
            double y = (double)sobelY.at<uchar>(i,j);
            double x = (double)sobelX.at<uchar>(i,j);
            int editValue = (int)atan( y / x );
            std::cout << (double)sobelY.at<uchar>(i,j) << std::endl;
            std::cout << y << std::endl;
            std::cout << y / x << std::endl; 
            bool l = ( (editValue > 1.1) && (editValue < 1.5) );
            bool r = ( (editValue < -1.1) && (editValue > -1.5) );
            if ( l || r )
                sobel_dir.at<uchar>(i,j) = 255;
            else
                sobel_dir.at<uchar>(i,j) = 0;
        }
    }       

    //gray.release();

    //TODO: Hough transform

    //TODO: Delete everything outside the region of interest

    //TODO: Perspective transform    

    // Convert back to sensor_msgs/Image
    cv_bridge::CvImage out_bridge;
    sensor_msgs::Image out_msg;
    std_msgs::Header header = msg.header;
    out_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, sobel_dir);
    out_bridge.toImageMsg(out_msg);

    return out_msg;

}

void LaneFinder::left_callback(const sensor_msgs::Image& msg) {
    sensor_msgs::Image lanes = findLanes(msg);
    _left_pub.publish(lanes);
}

void LaneFinder::right_callback(const sensor_msgs::Image& msg) {
    sensor_msgs::Image lanes = findLanes(msg);
    _right_pub.publish(lanes);
}

void LaneFinder::Initialize() {

    // Create publisher and subscriber objects.
    _left_sub = _nh.subscribe("/stereo_camera/left/image_color", 1000, &LaneFinder::left_callback, this);
    _right_sub = _nh.subscribe("/stereo_camera/right/image_color", 1000, &LaneFinder::right_callback, this);
    _left_pub = _nh.advertise<sensor_msgs::Image>("lane_finder/lane_lines/left", 1000);
    _right_pub = _nh.advertise<sensor_msgs::Image>("lane_finder/lane_lines/right", 1000);
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

