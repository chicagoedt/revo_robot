#include <opencv2/aruco.hpp> // for ArUco stuff
#include <opencv2/opencv.hpp> // for imshow
#include <sstream> 
#include <string>

using namespace cv;
using namespace std;

string toString(int i) {
	 ostringstream ss;
     ss << i;
     return ss.str();
}

int main( void ) {
	cv::Mat markerImage;
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	for (int i = 20; i < 51; i += 3) {
		cv::aruco::drawMarker(dictionary, i, 200, markerImage, 1);
		cv::imwrite("marker" + toString(i) + ".png", markerImage);
	}
}