#include "state_machine.h"
#include <roboteq_node/Actuators.h>

StateMachineBase::StateMachineBase(void):
	_moveBaseAC("move_base", true), _nh("state_machine"),
   MAX_GOAL_POINTS(5), _robotState(eInitializing)
{

}

StateMachineBase::~StateMachineBase(void)
{

}

bool StateMachineBase::Initialize()
{
  std::string   goalParamName = "GoalPoint";

  for(int i = 1; i <= MAX_GOAL_POINTS; i++)
  {
    std::ostringstream gpLat; 
    std::ostringstream gpLong; 

    gpLat   << "GoalPoint" << i << "_Latitude";
    gpLong  << "GoalPoint" << i << "_Longitude";

    ROS_DEBUG_STREAM("Loaded param: " << gpLat.str());
    ROS_DEBUG_STREAM("Loaded param: " << gpLong.str());

    if (_nh.hasParam(gpLat.str()) && _nh.hasParam(gpLong.str()))
    {
        std::vector<double>   goalLat;  
        std::vector<double>   goalLong;  

        _nh.getParam(gpLat.str(), goalLat);
        _nh.getParam(gpLong.str(), goalLong);

        geographic_msgs::GeoPoint geoPoint;

        double latitudeVal   = 0;
        double longitudeVal  = 0;

        StateMachineBase::convertDegMinSecToLL( goalLat, goalLong, latitudeVal, longitudeVal ); 

        ROS_INFO_STREAM("Lat: " << latitudeVal);
        ROS_INFO_STREAM("Long: " << longitudeVal);
        ROS_INFO_STREAM(" ");

        geoPoint.latitude   = latitudeVal;
        geoPoint.longitude  = longitudeVal;

        _geoPointsQueue.push(geoPoint);
    }
    else
    {
      ROS_ERROR_STREAM("Please fill the goalpoints.yaml for GoalPoint" << i);
      return false;
    }
  }

  return true;
}

void StateMachineBase::convertDegMinSecToLL(    std::vector<double> latVector,
                                                std::vector<double> longVector,
                                                double& latVal,
                                                double& longVal)
{

    latVal  = latVector[DEGREES] + (latVector[MINUTES] /60) + (latVector[SECONDS] /3600);
    longVal = longVector[DEGREES] + (longVector[MINUTES] /60) + (longVector[SECONDS] /3600);

}

void StateMachineBase::moveToGoalPoint()
{
	
}

void StateMachineBase::run()
{
	while(!_moveBaseAC.waitForServer(ros::Duration(2.0)))
	{
    ROS_INFO("Waiting for the move_base action server to come up");
	}

	ROS_INFO("Established Connection with move_base ActionServer.");

    int x = 0;

    while(!_goalPointsQueue.empty())
    {
        _moveBaseGoal.target_pose.header.frame_id   = "odom";
        _moveBaseGoal.target_pose.header.stamp      = ros::Time::now();

        _moveBaseGoal.target_pose.pose.position     = _goalPointsQueue.front().position;
        _moveBaseGoal.target_pose.pose.orientation  = tf::createQuaternionMsgFromYaw(0.01);

        ROS_INFO_STREAM("Sending goal(X, Y):" << "[ " << _moveBaseGoal.target_pose.pose.position.x << " , "
                                                  << _moveBaseGoal.target_pose.pose.position.y << " ]");

        _moveBaseAC.sendGoal(_moveBaseGoal);

        _moveBaseAC.waitForResult();

        if(_moveBaseAC.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            _goalPointsQueue.pop();
	        x++;

	        if(x == 2)
	        {   
		
	        }

            ROS_INFO("Succesfully moved to GoalPoint.");
        }
        else
        {
            ROS_INFO("Failed to move to GoalPoint.");
        }
    }



  //_moveBaseGoal.target_pose.pose.position.x = 7.0;
  //_moveBaseGoal.target_pose.pose.orientation.w = 1.0;

  //_moveBaseGoal.target_pose = _goalPoints.at(0);


}
