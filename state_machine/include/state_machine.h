#ifndef State_Machine_h
#define State_Machine_h

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>

#include <vector>
#include <queue>
#include <sstream>
#include <stack>
   
#define DEGREES 0
#define MINUTES 1
#define SECONDS 2


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class StateMachineBase
{

	const int MAX_GOAL_POINTS; // Refer to Goal Points vector

	public:

		StateMachineBase(void);
		~StateMachineBase(void);

		void run();
		bool Initialize();

	private:

		ros::NodeHandle					_nh;

        ros::Subscriber                 _servoSub; 

		MoveBaseClient 					_moveBaseAC;
		move_base_msgs::MoveBaseGoal 	_moveBaseGoal;

		bool startConnectionAC;
		bool sendGoalToAC(geometry_msgs::Pose goalPose);
        void moveToGoalPoint();

        /*
            Decimal Degrees = Degrees + minutes/60 + seconds/3600
        */
        static void convertDegMinSecToLL(   std::vector<double> latVector,
                                            std::vector<double> longVector,
                                            double& latVal,
                                            double& longVal);
                                

        std::queue<geographic_msgs::GeoPoint>   _geoPointsQueue;
		std::queue<geometry_msgs::Pose>         _goalPointsQueue;

        enum eState
        {
            eInitializing,
            eRelocalize,
            eDriveToDig,
            eDigging,
            eDriveToDump,
            eDumping
        };

        eState _robotState;

        std::stack<eState> _stateStack;

};


#endif
