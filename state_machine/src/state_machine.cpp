#include "state_machine.h"

StateMachineBase::StateMachineBase(void):
	_moveBaseAC("move_base", true)
{

}

StateMachineBase::~StateMachineBase(void)
{

}

void StateMachineBase::run()
{
	while(!_moveBaseAC.waitForServer(ros::Duration(2.0)))
	{
    	ROS_INFO("Waiting for the move_base action server to come up");
	}

	ROS_INFO("Established Connection with move_base ActionServer.");

	_moveBaseGoal.target_pose.header.frame_id = "base_link";
  	_moveBaseGoal.target_pose.header.stamp = ros::Time::now();

  	_moveBaseGoal.target_pose.pose.position.x = 1.0;
  	_moveBaseGoal.target_pose.pose.orientation.w = 1.0;

  	ROS_INFO("Sending goal");
  	_moveBaseAC.sendGoal(_moveBaseGoal);

  	_moveBaseAC.waitForResult();

  	if(_moveBaseAC.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    	ROS_INFO("Hooray, the base moved 1 meter forward");
  	else
    	ROS_INFO("The base failed to move forward 1 meter for some reason");
}
