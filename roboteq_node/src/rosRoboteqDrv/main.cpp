#include "rosRoboteqDrv.h"
#include <boost/thread.hpp>

using namespace oxoocoffee;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roboteq_driver");

    int	 sleepTime(300);
    bool keepRunnning(true);

    while(keepRunnning)
    {
        ROS_WARN_STREAM_NAMED(NODE_NAME,"Stage... 1");

        RosRoboteqDrv roboteqDrv;

        ROS_WARN_STREAM_NAMED(NODE_NAME,"Stage... 2");

        try
        {
            roboteqDrv.Initialize();

            ROS_WARN_STREAM_NAMED(NODE_NAME,"Stage... 3");
            ros::spin();

            ROS_WARN_STREAM_NAMED(NODE_NAME,"Stage... 4");
            keepRunnning = false;
            roboteqDrv.Shutdown();

            ROS_WARN_STREAM_NAMED(NODE_NAME,"Stage... 5");
        }
        catch(std::exception& ex)
        {
            ROS_ERROR_STREAM_NAMED(NODE_NAME,"Exception. Error: " << ex.what());
            roboteqDrv.Shutdown();
            boost::this_thread::sleep(boost::posix_time::milliseconds(sleepTime));
            ROS_WARN_STREAM_NAMED(NODE_NAME,"Restarting...");
        }
        catch(...)
        {
            ROS_ERROR_STREAM_NAMED(NODE_NAME,"Exception. ???");
            roboteqDrv.Shutdown();
            boost::this_thread::sleep(boost::posix_time::milliseconds(sleepTime));
            ROS_WARN_STREAM_NAMED(NODE_NAME,"Restarting...");
        }
    }

    ros::shutdown();
}

