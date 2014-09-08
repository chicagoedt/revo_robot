#include "./line_detection_test.cpp"
#include "ros/ros.h"

// #include "./RANSAC.cpp"
int main(int argc, char** argv)
{
	ros::init(argc, argv, "line_detection_test");
	//refer to this for arguments findHough(double cannyThresh1, double cannyThresh2, double cannyApertureSize, bool cannyL2, double houghRho, double houghTheta, double houghMinLineLength, double houghMaxLineGap, int houghMaxNumberOfLines)
	// findHough(100, 200, 3, false, 1.0f, (float) (CV_PI / 180.0f), 50, 5, 100);
	initialize();
	return 0;
}