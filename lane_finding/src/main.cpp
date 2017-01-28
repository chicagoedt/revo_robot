#include "lane_finder.h"
#include <signal.h>

void sigIntHandler(int sig) {
    ROS_DEBUG("--> SIGINT Handler called <--");

    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lane_finder");

    LaneFinder laneFinder;

    signal(SIGINT, sigIntHandler);

    laneFinder.Initialize();
}

