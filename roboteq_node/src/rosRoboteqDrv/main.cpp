#include "rosRoboteqDrv.h"
#include <boost/thread.hpp>

using namespace oxoocoffee;

const int MAX_RETRY = 5;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roboteq_driver");

    int	 sleepTime(300);
    int  keepRunnning(5);

    while(keepRunnning)
    {
        ROS_WARN_STREAM_NAMED(NODE_NAME,"entering loop");

        RosRoboteqDrv roboteqDrv;

        ROS_WARN_STREAM_NAMED(NODE_NAME,"Created");

        try
        {
            ROS_WARN_STREAM_NAMED(NODE_NAME,"Initializing");
            roboteqDrv.Initialize();

            ROS_WARN_STREAM_NAMED(NODE_NAME,"Entering Run Loop");
            ros::spin();

            keepRunnning = 0;
            
            ROS_WARN_STREAM_NAMED(NODE_NAME,"Shutting Down");
            roboteqDrv.Shutdown();
        }
        catch(std::exception& ex)
        {
            keepRunnning--;
            std::ostringstream a2i; a2i << "Restarting... " << keepRunnning << " out of " << MAX_RETRY;
            ROS_ERROR_STREAM_NAMED(NODE_NAME,"Exception. Error: " << ex.what());
            roboteqDrv.Shutdown();
            boost::this_thread::sleep(boost::posix_time::milliseconds(sleepTime));
            ROS_WARN_STREAM_NAMED(NODE_NAME,"Restarting...");
        }
        catch(...)
        {
            keepRunnning--;
            std::ostringstream a2i; a2i << "Restarting... " << keepRunnning << " out of " << MAX_RETRY;
            ROS_ERROR_STREAM_NAMED(NODE_NAME,"Exception. ???");
            roboteqDrv.Shutdown();
            boost::this_thread::sleep(boost::posix_time::milliseconds(sleepTime));
            ROS_WARN_STREAM_NAMED(NODE_NAME,"Restarting...");
        }
    }

    ROS_WARN_STREAM_NAMED(NODE_NAME,"Exiting");
    ros::shutdown();
}

