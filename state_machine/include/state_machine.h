#ifndef State_Machine_h
#define State_Machine_h

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class StateMachineBase
{
	public:

		StateMachineBase(void);
		~StateMachineBase(void);

		void run();

	private:

		MoveBaseClient 						_moveBaseAC;
		move_base_msgs::MoveBaseGoal 		_moveBaseGoal;

		bool startConnectionAC;
		bool sendGoalToAC(geometry_msgs::Pose goalPose);

};


#endif