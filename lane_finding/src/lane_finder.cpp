// This program does all image processing.

#include "lane_finder.h"
#include <signal.h>
#include <cmath> // for abs() and atan()

void LaneFinder::publishImage( cv::Mat& image, ros::Publisher& publisher ) {
    cv_bridge::CvImage out_bridge;
    sensor_msgs::Image out_msg;
    std_msgs::Header header = std_msgs::Header();
    out_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, image);
    out_bridge.toImageMsg(out_msg);
    publisher.publish(out_msg);
}

cv::Mat LaneFinder::findLanes(const sensor_msgs::Image& msg) {

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
    cv::Mat reframe;
    int newCols = 500;
    double ratio = (double)full_frame.rows / (double)full_frame.cols;
    cv::resize(full_frame, reframe, cv::Size(newCols,newCols*ratio));
    full_frame.release();
    
    // Gaussian blur to reduce noise using kernel size 3 or 5
    cv::Mat frame;
    cv::GaussianBlur(reframe, frame, cv::Size(5,5), 10);
    reframe.release();

    // Convert to grayscale and HSV
    cv::Mat gray;
    cv::Mat hsv;
    cv::cvtColor(frame, gray, CV_BGR2GRAY);
    cv::cvtColor(frame, hsv, CV_BGR2HSV);
    //frame.release();

    // Isolate S channel of HLS and run Sobel over gray.
    cv::Mat hue, saturation, value;
    cv::extractChannel(hsv, hue, 0);
    cv::extractChannel(hsv, saturation, 1);
    cv::extractChannel(hsv, value, 2);
    //hsv.release();
    
    cv::Mat sobelX, sobelY;
    cv::Sobel(saturation, sobelX, -1, 1, 0);
    cv::Sobel(saturation, sobelY, -1, 0, 1);
    //std::cout << hsv.depth() << std::endl;
    // It's CV_8U

    // Threshold the magnitude and direction of the Sobel-ized image.
    cv::Mat abs_sobelX; 
    abs_sobelX = abs(sobelX);

    cv::Mat sobel_mag, sobel_dir;
    frame.copyTo(sobel_dir);
    cv::threshold(abs_sobelX, sobel_mag, 10.0, 255.0, cv::THRESH_BINARY);
    for (int i=0; i < frame.rows; i++) {
        for (int j=0; j < frame.cols ; j++) {
            double y = static_cast<double>( sobelY.at<uchar>(i,j) );
            double x = static_cast<double>( sobelX.at<uchar>(i,j) );
            if ( x < 1 )
                x = 1;
            double pi = 3.14159265358979323846264338;
            double editValue = abs( atan( y / x ) ); 
            std::cout << y << std::endl;
            std::cout << x << std::endl;
            std::cout << y / x << std::endl; 
            std::cout << editValue << std::endl; 
            if ( (editValue > pi / 3.0 ) && (editValue < pi / 2.0) )
                sobel_dir.at<uchar>(i,j) = 255;
            else
                sobel_dir.at<uchar>(i,j) = 0;
        }
    }       

    //gray.release();

    //TODO: Hough transform

    //TODO: Delete everything outside the region of interest

    //TODO: Perspective transform    
    
    publishImage(hue, _hue_pub);
    publishImage(saturation, _saturation_pub);
    publishImage(value, _value_pub);
    publishImage(gray, _intensity_pub);

    return sobel_mag;

}

void LaneFinder::left_callback(const sensor_msgs::Image& msg) {
    cv::Mat lanes = findLanes(msg);
    publishImage(lanes, _left_pub);
}

void LaneFinder::right_callback(const sensor_msgs::Image& msg) {
    cv::Mat lanes = findLanes(msg);
    publishImage(lanes, _right_pub);
}

void LaneFinder::Initialize() {

    // Create publisher and subscriber objects.
    _left_sub = _nh.subscribe("/stereo_camera/left/image_color", 1000, &LaneFinder::left_callback, this);
    _right_sub = _nh.subscribe("/stereo_camera/right/image_color", 1000, &LaneFinder::right_callback, this);
    _left_pub = _nh.advertise<sensor_msgs::Image>("lane_finder/lane_lines/left", 1000);
    _right_pub = _nh.advertise<sensor_msgs::Image>("lane_finder/lane_lines/right", 1000);
    _hue_pub = _nh.advertise<sensor_msgs::Image>("lane_finder/hue", 1000);
    _saturation_pub = _nh.advertise<sensor_msgs::Image>("lane_finder/saturation", 1000);
    _value_pub = _nh.advertise<sensor_msgs::Image>("lane_finder/value", 1000);
    _intensity_pub = _nh.advertise<sensor_msgs::Image>("lane_finder/intensity", 1000);
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
    
    ros::Rate rate(2);
    rate.sleep();
    ros::spin();
 }

