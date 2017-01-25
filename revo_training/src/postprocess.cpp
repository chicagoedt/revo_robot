// This program combines bagged data from the left camera, right camera, and cmd_vel topics.

// For each frame from the left camera, it grabs the frame from the right camera that is nearest
// to it in time, along with the linear.x and angular.z velocities from cmd_vel.

// It combines these frames into a png, with the velocities stored as metadata.

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/CompressedImage.h>

int main(int argc, char **argv) {
    rosbag::Bag bag;
    bag.open( argv[1], rosbag::bagmode::Read );

    // Create a vector of all topic we're interested in.
    std::vector<std::string> topics;
    topics.push_back(std::string("cmd_vel"));
    topics.push_back(std::string("/stereo_camera/left/image_rect_color/compressed"));
    topics.push_back(std::string("/stereo_camera/right/image_rect_color/compressed"));

    // Grab those topics from the bag.
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Do stuff to them.
    foreach(rosbag::MessageInstance const m, view)
    {
        //TODO
    }

    return 0;
}
