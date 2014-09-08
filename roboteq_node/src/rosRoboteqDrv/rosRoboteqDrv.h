#ifndef __ROBOTEQ_DRV_H__
#define __ROBOTEQ_DRV_H__

#include "roboteqCom.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>    // Twist message file
#include <string>
#include <roboteq_node/wheels_msg.h>

/// Scipio dimensions and stuff
#define METERS_PER_TICK_SCIPIO  0.0011169116
#define WHEEL_DIAMETER_SCIPIO   0.3556
#define TICK_COLLECTION_PERIOD  0.05
#define TRACK_WIDTH             0.8636          //34 inches
#define WHEEL_BASE              0.5334    //21 inches
#define SLEEP_INTERVAL          0.05
#define RPM_TO_RAD_PER_SEC      0.1047

//#define ROBO_DEV_PATH		"/dev/ttyUSB0"
#define ROBO_DEV_PATH 		"/dev/ttyACM0"
#define NODE_NAME		"roboteq_node"

// ROS Roboteq Driver
// Robert J. Gebis (oxoocoffee) <rjgebis@yahoo.com>
// Krystian R. Gebis            <krgebis@gmail.com>
// EDT Chicago (UIC) 2014
//
// Version 1.0 (05/20/14) - Created Template for all functions needed
// Version 1.1 (05/28/14) - Wrote functions for basic communication 
//                          using ros to physical Roboteq
// Version 1.2 (05/29/14) - Twist and Velocity conversion function work
//                          correctly. No more ros::init() error. 
//
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Public License for more details at
// http://www.gnu.org/copyleft/gpl.html

// Our Roboteq HDC2450 is in CLOSED-LOOP speed mode. Set the MXRPM
// through the Roborun/RoboteqDbg tools. 

using namespace oxoocoffee;

class RosRoboteqDrv : public SerialLogger, public IEventListener<IEventArgs>
{
    typedef roboteq_node::wheels_msg    TWheelMsg;
    typedef geometry_msgs::Twist        TTwist;

    public:
        RosRoboteqDrv(void);

        void        Initialize(void);
        void        Run(void);
        void        Shutdown(void);
        void        CmdVelCallback(const TTwist::ConstPtr& twist_velocity);
        static TWheelMsg   ConvertTwistToWheelVelocity(const TTwist::ConstPtr& twist_velocity);   
        static TTwist      ConvertWheelVelocityToTwist(float left_velocity, float right_velocity);
 
        ros::NodeHandle     _nh;
    
    protected:
        // RoboteqCom Events
        virtual void    OnMsgEvent(IEventArgs& evt);

        virtual bool    IsLogOpen(void) const;

        // Does write new line at the end 
        virtual void    LogLine(const char* pBuffer, unsigned int len);
        virtual void    LogLine(const std::string& message);

        // Does NOT write new line at end
        virtual void    Log(const char* pBuffer, unsigned int len);
        virtual void    Log(const std::string& message);

	void    Process_S(const IEventArgs& evt);
	void    Process_G(const IEventArgs& evt);
	void    Process_N(const IEventArgs& evt);

    private:
        RoboteqCom          _comunicator;
        ros::Subscriber     _sub;
        ros::Publisher      _pub;
        TWheelMsg           _wheelVelocity;
};

#endif // __ROBOTEQ_DRV_H__

