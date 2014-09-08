#include "rosRoboteqDrv.h"

typedef std::vector<std::string> TStrVec;
void    Split(TStrVec& vec, const string& str);

RosRoboteqDrv::RosRoboteqDrv(void)
 : _comunicator(*this, *this)
{

}

void    RosRoboteqDrv::Initialize()
{
    try
    {
        _pub = _nh.advertise<geometry_msgs::Twist>("current_velocity", 1); 
        // Make sure roboteq configuration has roboteq on!! "^ECHOF 1"
        _comunicator.Open(ROBO_DEV_PATH);
        
        if(_comunicator.Version().empty() )
            THROW_RUNTIME_ERROR("Failed to receive Roboteq Version");

        if(_comunicator.Model().empty() )
            THROW_RUNTIME_ERROR("Failed to receive Roboteq Model");

        if(_comunicator.IsThreadRunning() == false)
            THROW_RUNTIME_ERROR("Failed to spawn RoboReader Thread");
        
        _comunicator.IssueCommand("# C");   // Clears out telemetry strings
        _comunicator.IssueCommand("?S");    // Query for speed and enters this speed 
                                            // request into telemetry system
        _comunicator.IssueCommand("# 100"); // auto message response is 500ms

        _sub = _nh.subscribe("cmd_vel", 1, &RosRoboteqDrv::CmdVelCallback, this);
    }
    catch(std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME,"Open Port Failed. Error: " << ex.what());
	throw;
    }
}

void    RosRoboteqDrv::Shutdown(void)
{
    _comunicator.Close();
}

void    RosRoboteqDrv::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& twist_velocity)
{
    _wheelVelocity = RosRoboteqDrv::ConvertTwistToWheelVelocity(twist_velocity);

    float leftVelRPM  = _wheelVelocity.left  / RPM_TO_RAD_PER_SEC;
    float rightVelRPM = _wheelVelocity.right / RPM_TO_RAD_PER_SEC;

    // now round the wheel velocity to int

    std::stringstream ss;

    if (leftVelRPM >= 0)
        ss << "!G 2 " << (((int)leftVelRPM) * 100);
    else
        ss << "!G 2 " << (((int)leftVelRPM) * 100);

    try
    {
    	_comunicator.IssueCommand(ss.str());
    }
    catch(std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME,"IssueCommand G2 Error: " << ex.what());
	throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME,"IssueCommand G2");
	throw;
    }

    ss.str(""); // Clear previouse contents

    if (rightVelRPM >= 0)
        ss << "!G 1 " << (((int)rightVelRPM) * 100);
    else
        ss << "!G 1 " << (((int)rightVelRPM) * 100);

    try
    {
        _comunicator.IssueCommand(ss.str());
    }
    catch(std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME,"IssueCommand G1 : " << ex.what());
	throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME,"IssueCommand G1 : ?");
	throw;
    }

}

geometry_msgs::Twist RosRoboteqDrv::ConvertWheelVelocityToTwist(float left_velocity, float right_velocity)
{
    // using the two equations for left and right, we solve for long. vel and we get two equations for it. Add them together, and we end up with VL = (right - left) * r / 2 
    float longitudinal_velocity = (right_velocity - left_velocity) * (WHEEL_DIAMETER_SCIPIO / 4);

    geometry_msgs::Twist twistVelocity;

    // linear.x is just the average of left and right wheel velocities converted to linear by multiplying it by radius
    twistVelocity.linear.x = ((left_velocity + right_velocity) / 2) * (WHEEL_DIAMETER_SCIPIO / 2);

    twistVelocity.angular.z = (longitudinal_velocity * 2 * TRACK_WIDTH) / (TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE);

    return twistVelocity;
}

roboteq_node::wheels_msg RosRoboteqDrv::ConvertTwistToWheelVelocity(const geometry_msgs::Twist::ConstPtr& twist_velocity) // look into removing permanently
{
    float longitudinalVelocity = ((twist_velocity->angular.z) * (TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE)) / (2 * TRACK_WIDTH);

    roboteq_node::wheels_msg wheelVelocity;

    wheelVelocity.left      = -1*longitudinalVelocity + twist_velocity->linear.x;
    wheelVelocity.right     = longitudinalVelocity + twist_velocity->linear.x;

    return wheelVelocity;
}

// RoboteqCom Events
void    RosRoboteqDrv::OnMsgEvent(IEventArgs& evt)
{
	ROS_DEBUG_STREAM_NAMED(NODE_NAME, "OnMsgEvent: " << evt.Reply());

	switch( evt.Reply()[0] )
	{
		case 'S':
			Process_S( evt );
		break;

		case 'G':
			Process_G( evt );
		break;

		case 'N':
			Process_N( evt );
		break;

		default:
		break;
			
	}
}

void	RosRoboteqDrv::Process_S(const IEventArgs& evt)
{
	try
    {
      	string::size_type idx = evt.Reply().find_first_of('=');

        if( idx != string::npos )
        {
                //idx++;
                string::size_type idy = evt.Reply().find_first_of(':', idx);

                if( idy != string::npos )
                {
                        char* pVal1 = (char*)(evt.Reply().c_str() + idx + 1);
                        char* pVal2 = (char*)(evt.Reply().c_str() + idy);

                        *pVal2 = 0L;
                        pVal2++;

                        int firstVal  = atoi( pVal1 );
                        int secondVal = atoi( pVal2 );

			            roboteq_node::wheels_msg wheelVelocity;

			            wheelVelocity.right 	= firstVal  * RPM_TO_RAD_PER_SEC;
			            wheelVelocity.left		= secondVal * RPM_TO_RAD_PER_SEC;

                        _pub.publish(RosRoboteqDrv::ConvertWheelVelocityToTwist(wheelVelocity.left, wheelVelocity.right));
                        ROS_INFO_STREAM("Wheel RPM's: " << firstVal << " :: " << secondVal);
                }
                else
		        {
                  	ROS_ERROR_STREAM("Invalid(2) S Reply Format");
		        }
        }
        else
	    {
                ROS_ERROR_STREAM("Invalid(1) S Reply Format");
	    }
	}
	catch(std::exception& ex)
	{
		ROS_ERROR_STREAM_NAMED(NODE_NAME,"Process_S : " << ex.what());
	}
	catch(...)
	{
		ROS_ERROR_STREAM_NAMED(NODE_NAME,"Process_S : ?");
	}
}

void    RosRoboteqDrv::Process_G(const IEventArgs& evt)
{

}

void	RosRoboteqDrv::Process_N(const IEventArgs& evt)
{

}

bool    RosRoboteqDrv::IsLogOpen(void) const
{
    return false;
}

// RoboteqCom and app Log Messages. Do append newline
void    RosRoboteqDrv::LogLine(const char* pBuffer, unsigned int len)
{
    ROS_INFO_STREAM(std::string(pBuffer, len));
}

// RoboteqCom and app Log Messages. Do append newline
void    RosRoboteqDrv::LogLine(const std::string& message)
{
	ROS_INFO_STREAM(message);
}

// RoboteqCom and app Log Messages. Do not append newline
void    RosRoboteqDrv::Log(const char* pBuffer, unsigned int len)
{
    ROS_INFO_STREAM(std::string(pBuffer, len));
}

// RoboteqCom and app Log Messages. Do not append newline
void    RosRoboteqDrv::Log(const std::string& message)
{
	ROS_INFO_STREAM(message);
}

void    Split(TStrVec& vec, const string& str)
{
    if( str.empty() )
        return;

    string::size_type startIdx(0);

    while(true)
    {
        string::size_type startIdy = str.find_first_of(' ', startIdx);

        if( startIdy == string::npos )
        {
            vec.push_back( str.substr( startIdx ) );
            break;
        }
        else
        {
            vec.push_back( str.substr( startIdx, startIdy -  startIdx) );
            startIdx = startIdy+1;
        }
    }
}


