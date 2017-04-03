// Broadcasts a video file as a sensor_msgs::Image topic.

#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

int main( int argc, char **argv ) {
    
    // OpenCV stuff.
    VideoCapture cap(argv[1]); // open the video file for reading

    if ( !cap.isOpened() ) {
        cout << "Cannot open the video file" << endl;
        return -1;
    }

    double fps = cap.get(CV_CAP_PROP_FPS); // get the frames per second of the video

    Mat frame;

    // ROS stuff.
    ros::init(argc, argv, "testcaster");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>( "stereo_camera/left/image_color", 1000 );
    ros::Rate rate(fps);

    cv_bridge::CvImage out_bridge;
    sensor_msgs::Image frame_msg;
    std_msgs::Header header;

    // Main loop.
    int counter = 0;
    int frame_count = cap.get(CV_CAP_PROP_FRAME_COUNT);
    while( ros::ok() ) {
        if (counter == frame_count) {
            cap.open(argv[1]);
            counter = 0;
        }

        bool bSuccess = cap.read( frame ); // read a new frame from video

        if ( !bSuccess) {
            cout << "Cannot read the frame from video file" << endl;
            break;
        }

        out_bridge = cv_bridge::CvImage( header, sensor_msgs::image_encodings::BGR8, frame );
        out_bridge.toImageMsg( frame_msg );

        pub.publish( frame_msg );
        rate.sleep();

        counter++;
    }
}
