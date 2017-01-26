#include "opencv2/opencv.hpp" 
#include "opencv2/videoio.hpp"
#include <opencv2/core/core.hpp>
#include <vector>
#include <opencv2/aruco.hpp> 

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        //cvtColor(frame, edges, CV_BGR2GRAY);
        //GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        //Canny(edges, edges, 0, 30, 3);
        //imshow("edges", edges);
        vector< int > markerIds;
        vector< vector<cv::Point2f> > markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> parameters;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);
        cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        imshow("frame", frame);

        if(waitKey(30) >= 0) break;

        
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}