#include <opencv2/aruco.hpp> // for ArUco stuff
#include <opencv2/opencv.hpp> // for imshow
#include <opencv2/aruco/charuco.hpp>

using namespace cv;
using namespace std;

int main( void ) {
	cv::Mat board_img;
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(3,3,0.0287,0.02413, dictionary);
	board->draw(cv::Size(200,200),board_img,10,1); // adjust pixels so it doesn't print at the wrong size
	cv::imwrite("charuco.png", board_img);
}