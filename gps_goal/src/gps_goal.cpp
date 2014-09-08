// gps_goal.cpp
// Maintained by Matt Kelly --- mattkelly@gmail.com
// Creates a simple goal for the robot to move based on current GPS location
// and a hardcoded GPS goal, currently the middle of the EDT parking lot.
// Output message is in the form of distance x in meters and orientation w.

// 3/27/14 Version 0.1 - Basic functionality
// 5/26/14 Version 0.2 - Class runs everything, added relative angle to goal
// 5/28/14 Version 1.0 - Full functionality
// 5/31/14 Version 1.1 - Goals now defined in launch file, each goal sent only once
// 6/02/14 Version 1.2 - Supports Action Server Responses

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <gps_goal/conversions.h>
#include <math.h>
#include <tf/transform_broadcaster.h>

#define TWOPI 6.2831853

using namespace gps_goal;  // This is necesary for LL
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Goal {
  int goalIndex;
  float x, w;
  double lonGoal[5], latGoal[5];
  move_base_msgs::MoveBaseGoal goal;
  MoveBaseClient ac;

    public:
      Goal(): ac("move_base", true)  // Constructor
      {
        // Declare Subscribers
        fix_sub = nh.subscribe("fix", 1, &Goal::fix_callback, this);
        heading_sub = nh.subscribe("heading", 1, &Goal::heading_callback, this);

        // Populate goal arrays
        goalIndex = 0;
        nh.param("long0"     , lonGoal[0],    lonGoal[0] );
        nh.param("long1"     , lonGoal[1],    lonGoal[1] );
        nh.param("long2"     , lonGoal[2],    lonGoal[2] );
        nh.param("long3"     , lonGoal[3],    lonGoal[3] );
        nh.param("long4"     , lonGoal[4],    lonGoal[4] );
        nh.param("lat0"      , latGoal[0],    latGoal[0] );
        nh.param("lat1"      , latGoal[1],    latGoal[1] );
        nh.param("lat2"      , latGoal[2],    latGoal[2] );
        nh.param("lat3"      , latGoal[3],    latGoal[3] );
        nh.param("lat4"      , latGoal[4],    latGoal[4] );

        for(int i = 0; i <5; i++) {
          ROS_INFO("Goal %d is %f, %f", i, lonGoal[i], latGoal[i] );
        }

        // Wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))) 
         ROS_INFO("Waiting for the move_base action server to come up");
      }

      // Fires when a new fix comes in and publishes the goal
      void fix_callback(const sensor_msgs::NavSatFixConstPtr& fix) {
        double northing, easting, angle, goal_northing, goal_easting;  // LLtoUTM data buffers
        std::string zone;  // Another LLtoUTM buffer

        // Compare goal to current fix
        LLtoUTM(latGoal[goalIndex], lonGoal[goalIndex], goal_northing, goal_easting, zone);  // Convert goal lat/long to meters north/east of origin
        LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);  // Convert current lat/long to meters north/east of origin
        goal_northing -= northing;
        goal_easting -= easting;
        ROS_INFO("I need to move %fm north and %fm east.", goal_northing, goal_easting);

        // Compute distance and relative angle to goal
        x = sqrt(goal_northing*goal_northing + goal_easting*goal_easting);  // Find magnitude of distance
        angle = atan(goal_northing/goal_easting) + ( (goal_easting < 0)?acos(-1):0 );  // Find angle to goal
        w = angle - w;  // Compute angle to turn
        if (w < 0)                w += TWOPI;  // Correct if angle was negative
        else if (w > 6.2831853)   w -= TWOPI;  // Correct if angle was greater than 2pi
        ROS_INFO("I need to move %fm total at %fradians", x, w);

//	tf::Quaternion q;
//	q.setRPY(0, 0, w);

        // Send the new goal 
        goal.target_pose.header.frame_id = "base_footprint";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.orientation.w = w;
        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        // Get response from action server
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
         goalIndex++;  // Get the next goal
        
        // Shut down the node if we've made it to the end
        if (goalIndex > 4)
          ros::shutdown();
      }

      // Fires when a new heading comes in
      void heading_callback(const sensor_msgs::Imu::ConstPtr& heading) {
        w = heading->orientation.z;  // Grab the heading and store it in w
      }

    private:
      ros::NodeHandle nh; 
      ros::Subscriber fix_sub, heading_sub;
};


int main(int argc, char** argv){
  ros::init(argc, argv, "gps_goal");  // Initialize ROS node
  Goal aGoal;  // This object will handle everything
  ros::spin();  // Run forever

  return 0;
}
