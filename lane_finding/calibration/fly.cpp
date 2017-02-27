#include <opencv2/aruco.hpp> // for ArUco stuff
#include <opencv2/opencv.hpp> // for imshow
#include <opencv2/aruco/charuco.hpp>
#include <vector>

#include <iostream>
#include <ctime>

using namespace cv;
using namespace std;

void posCorrect( Mat & rvec, Mat & tvec ) {
    if (rvec.at<double>(0,0) > 2.8) { // too much roll

    } else if (rvec.at<double>(0,0) < -3.1) {

    }
    if (rvec.at<double>(0,2) > 0.2) { // too much pitch

    } else if (rvec.at<double>(0,2) < -0.2) {

    }
    if (rvec.at<double>(0,1) > 0.2) { // too much yaw

    } else if (rvec.at<double>(0,1) < -0.2) {

    }
    if (tvec.at<double>(0,0) < -0.1) { // too far forward/back
        cout << "Out of bounds! Move left." << endl;
    } else if (tvec.at<double>(0,0) > -0.08) {
        cout << "Out of bounds! Move right." << endl;
    }
    if (tvec.at<double>(0,1) > 0.1) { // too far left/right
        cout << "Out of bounds! Move up." << endl;
    } else if (tvec.at<double>(0,1) < 0.08) {
        cout << "Out of bounds! Move down." << endl;
    }
    if (tvec.at<double>(0,2) > 0.52) { // too high/low
        cout << "Out of bounds! Move forward." << endl;
    } else if (tvec.at<double>(0,2) < 0.48) {
        cout << "Out of bounds! Move back." << endl;
    }
}

int main( void ) {
    cv::Mat board_img;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(3,3,0.0287,0.02413, dictionary);

    vector< int > markerIds;
    vector< vector<cv::Point2f> > markerCorners, rejectedCandidates;
    Mat charucoCorners, charucoIds;

    int keyPress;
    cv::Size imgSize;

    
    
    cv::Mat cameraMatrix, distCoeffs;
    cameraMatrix = Mat::zeros(3, 3, CV_64F);
    distCoeffs = Mat::zeros(1, 5, CV_64F);

    
    cameraMatrix.at<double>(0,0) = 637.67541244019185;
    cameraMatrix.at<double>(0,1) = 0.0;
    cameraMatrix.at<double>(0,2) = 321.40724360408586;
    cameraMatrix.at<double>(1,0) = 0.0;
    cameraMatrix.at<double>(1,1) = 636.89279179344192;
    cameraMatrix.at<double>(1,2) = 253.66955032146944;
    cameraMatrix.at<double>(2,0) = 0.0;
    cameraMatrix.at<double>(2,1) = 0.0;
    cameraMatrix.at<double>(2,2) = 1.0;
    
    distCoeffs.at<double>(0,0) = -0.022969138601307997;
    distCoeffs.at<double>(0,1) = 0.26892981966235663;
    distCoeffs.at<double>(0,2) = 0.0024903695182285947;
    distCoeffs.at<double>(0,3) = -0.00066505016295878995;
    distCoeffs.at<double>(0,4) = -0.74935001823289238;
    cout << cameraMatrix << endl << distCoeffs << endl;
    Mat rvec, tvec;

    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    for(;;)
    {
        Mat frame, temp;
        cap >> frame;

        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);
        if (markerIds.size() > 0) {
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, frame, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs, 2);
            cout << charucoIds << endl;
            
            if (cv::aruco::estimatePoseCharucoBoard( charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec )) {
                cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
                cout << "R:" << endl << rvec << endl << "T:" << endl << tvec << endl;
                posCorrect(rvec, tvec);
            }

        }
        imshow("frame", frame);
        if(waitKey(30) >= 0) break;
    }

}
