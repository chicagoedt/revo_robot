#include "TFListener.h"

int main(int argc, char* argv[])
{
  	ros::init(argc, argv, "robot_tf_listener");

	TFListener::Instance().Initialize();

  	//tf::TransformListener listener(ros::Duration(10)); // Check what this value being passed is.

  	//we'll transform a point once every second
  	//ros::Timer timer = _nh.createTimer(ros::Duration(0.1), boost::bind(&TFListener::transformPoint, boost::ref(listener)));

  	ros::spin();

	return 0;
}
