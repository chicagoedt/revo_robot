#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <base_controller/Xbox_Button_Msg.h>
#include <iostream>

using namespace std;

class TeleopJoy
{ 
public: 
	TeleopJoy();
private:
	void callback(const sensor_msgs::Joy::ConstPtr& joy);
	ros::NodeHandle 	n;
	ros::Publisher 		pub;
	ros::Publisher		button_pub;
	ros::Subscriber 	sub;

	int velLinear;  //
	int velAngular; //in launch file start.launch in base_controller directory,
	int buttonA;
	int buttonB;
	int buttonX;				//look for <param name="axis_linear" value="(input index value here from link below)"
	int buttonY;				// http://wiki.ros.org/joy?distro=groovy        (Section 5.3)
	int left_bumper;
	int right_bumper;				// 
	int back_but;
	int start_but;
	int power_but;
	int left_click;
	int right_click;
	int left_trig;
	int right_trig;
	int dpad_LR;
	int dpad_UD;
public:
	int counter;
};

TeleopJoy::TeleopJoy()
{
	counter = 0;

	n.param("axis_linear" 		, velLinear,  	0);
	n.param("axis_angular"		, velAngular,   1);
	n.param("button_a"			, buttonA, 		buttonA);
	n.param("button_b"			, buttonB, 		buttonB);
	n.param("button_x"			, buttonX, 		buttonX);
	n.param("button_y"			, buttonY, 		buttonY);
	n.param("left_bumper"		, left_bumper, 	left_bumper);
	n.param("right_bumper"		, right_bumper, right_bumper);
	n.param("back_button"		, back_but, 	back_but);
	n.param("start_button"		, start_but, 	start_but);
	n.param("power_button"		, power_but, 	power_but);
	n.param("button_stick_left"	, left_click, 	left_click);
	n.param("button_stick_right", right_click, 	right_click);
	n.param("left_trigger"		, left_trig, 	left_trig);
	n.param("right_trigger"		, right_trig, 	right_trig);
	n.param("dpad_leftright"	, dpad_LR, 		dpad_LR);
	n.param("dpad_updown"		, dpad_UD, 		dpad_UD);

	pub 		= n.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
	button_pub	= n.advertise<base_controller::Xbox_Button_Msg> ("xbox_controller", 1);
	sub 		= n.subscribe<sensor_msgs::Joy> ("joy", 1, &TeleopJoy::callback, this);
}

void TeleopJoy::callback(const sensor_msgs::Joy::ConstPtr& joy)
{  
	++counter;

	geometry_msgs::Twist 				vel;
	base_controller::Xbox_Button_Msg	buttonPressed;

	vel.angular.z 	= joy->axes[0];
	vel.linear.x 	= joy->axes[1];

/*	buttonPressed.a 					= joy->buttons[0];
	buttonPressed.b 					= joy->buttons[buttonB];
	buttonPressed.x 					= joy->buttons[buttonX];
	buttonPressed.y 					= joy->buttons[buttonY];
	buttonPressed.left_bumper 			= joy->buttons[left_bumper];
	buttonPressed.right_bumper 			= joy->buttons[right_bumper];
	buttonPressed.back_button 			= joy->buttons[back_but];		// Part of Buttons index
	buttonPressed.start_button 			= joy->buttons[start_but];
	buttonPressed.power_button 			= joy->buttons[power_but];
	buttonPressed.button_stick_left 	= joy->buttons[left_click];
	buttonPressed.button_stick_right 	= joy->buttons[right_click];
	buttonPressed.left_trigger 			= joy->axes[left_trig];
	buttonPressed.right_trigger 		= joy->axes[right_trig];
	buttonPressed.dpad_leftright 		= joy->axes[dpad_LR];		// Part of the Axes Index
	buttonPressed.dpad_updown 			= joy->axes[dpad_UD];
*/

	button_pub.publish(buttonPressed);
	pub.publish(vel);
}

int main(int argc, char** argv)
{  
	ros::init(argc, argv, "teleopJoy");
	TeleopJoy 	joystick;
	
    ros::spin();

	std::cout << "Count: " << joystick.counter << std::endl;
}
