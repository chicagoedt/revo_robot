// This program combines bagged data from the left camera, right camera, and cmd_vel topics.

// For each frame from the left camera, it grabs the frame from the right camera that is nearest
// to it in time, along with the linear.x and angular.z velocities from cmd_vel.

// It combines these frames into a png, with the velocities stored as metadata.

// DEPENDS: roscpp, rosbag, geometry_msgs, sensor_msgs, image_transport, image_transport_plugins

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/CompressedImage.h>

//TODO: comment and update this code
char showImages(string title, vector<Mat>& imgs, Size cellSize) {
    char k=0;
    namedWindow(title);
    float nImgs=imgs.size();
    int   imgsInRow=ceil(sqrt(nImgs));     // You can set this explicitly
    int   imgsInCol=ceil(nImgs/imgsInRow); // You can set this explicitly

    int resultImgW=cellSize.width*imgsInRow;
    int resultImgH=cellSize.height*imgsInCol;

    Mat resultImg=Mat::zeros(resultImgH,resultImgW,CV_8UC3);
    int ind=0;
    Mat tmp;
    for(int i=0;i<imgsInCol;i++)
    {
        for(int j=0;j<imgsInRow;j++)
        {
            if(ind<imgs.size())
            {
            int cell_row=i*cellSize.height;
            int cell_col=j*cellSize.width;
            imgs[ind].copyTo(resultImg(Range(cell_row,cell_row+tmp.rows),Range(cell_col,cell_col+tmp.cols)));
            }
            ind++;
        }
    }
    imshow(title,resultImg);
    k=waitKey(10);
    return k;
}

int main(int argc, char **argv) {
    rosbag::Bag bag;
    bag.open( argv[1], rosbag::bagmode::Read );

    // Create a vector of all topics we're interested in.
    std::vector<std::string> topics;
    topics.push_back(std::string("cmd_vel"));
    topics.push_back(std::string("/stereo_camera/left/image_rect_color/compressed"));
    topics.push_back(std::string("/stereo_camera/right/image_rect_color/compressed"));

    // Grab those topics from the bag and create Mats to hold the frames.
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    cv::Mat left,right;

    // Do stuff to them.
    foreach(rosbag::MessageInstance const m, view)
    {
        //TODO: Merge left and right image files into single PNG.
        //cvmerge

        //TODO: Write angular and linear velocities as metadata.
    }


    return 0;
}
