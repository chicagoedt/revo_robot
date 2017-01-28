#include <opencv2/aruco.hpp> // for ArUco stuff
#include <opencv2/opencv.hpp> // for imshow
#include <opencv2/aruco/charuco.hpp>
#include <vector>

#include <iostream>
#include <ctime>

using namespace cv;
using namespace std;

static bool saveCameraParams(const string &filename, Size imageSize,
                             const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr) {
    FileStorage fs(filename, FileStorage::WRITE);
    if(!fs.isOpened())
        return false;

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}

int main( void ) {
    cv::Mat board_img;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(6,6,0.034,0.025, dictionary);
    
    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;

    std::vector< std::vector<cv::Point2f> > allCharucoCorners;
    std::vector< std::vector<int> > allCharucoIds;

    int keyPress;
    cv::Size imgSize;

    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    for(;;)
    {
        Mat frame;
        cap >> frame;
        imgSize = frame.size();
        vector< int > markerIds;
        vector< vector<cv::Point2f> > markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> parameters;
        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);
        cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        imshow("frame", frame);

        keyPress = waitKey(30);
        if( keyPress >= 0 ) {
            if ( keyPress == 27 ) {
                break;
            } else {
                cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, frame, board, charucoCorners, charucoIds);
                allCharucoCorners.push_back(charucoCorners);
                allCharucoIds.push_back(charucoIds);
                cout << "Ids: " << charucoIds.size() << endl;
                cout << "Corners: " << charucoCorners.size() << endl;
                cout << "AllIds: " << allCharucoIds.size() << endl;
                cout << "AllCorners: " << allCharucoCorners.size() << endl;
            }
        }
    }

    // After capturing in several viewpoints, start calibration
    cv::Mat cameraMatrix, distCoeffs;
    std::vector< Mat > rvecs, tvecs;
   
    double repError = cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, board, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs);
    saveCameraParams("calibration.yaml", imgSize, cameraMatrix, distCoeffs, repError);

}
