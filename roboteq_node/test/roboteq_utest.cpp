#include <gtest/gtest.h>
#include "../src/rosRoboteqDrv/rosRoboteqDrv.h"


TEST(TestRoboteq, standStill)
{
	geometry_msgs::Twist response;

	response = RosRoboteqDrv::ConvertWheelVelocityToTwist(0.0, 0.0);

	EXPECT_EQ(response.linear.x,  0.0);
	EXPECT_EQ(response.angular.z, 0.0);
}

TEST(TestRoboteq, convertTwistToWheelVels)
{
	float twist_angular_velocity_z 	= 1.0;
	float twist_linear_velocity_x	= 0.0;

	float longitudinalVelocity = ((twist_angular_velocity_z) * (TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE)) / (2 * TRACK_WIDTH);

	float wheelVel_left      = -1*longitudinalVelocity + twist_linear_velocity_x;
	float wheelVel_right     = longitudinalVelocity + twist_linear_velocity_x;

	//Value really is 0.5965, but you cant compare floating 
	//points, so i want at least a 4 digit accuracy 
	//(hence *10000) and then cast as int
	EXPECT_EQ((int)(wheelVel_left*10000), -5965); 
	EXPECT_EQ((int)(wheelVel_right*10000), 5965);

}

/*
TEST(TestRoboteq, convertWheelVelsToTwist)
{
	geometry_msgs::Twist response;

	response = RosRoboteqDrv::ConvertWheelVelocityToTwist((-570.0*0.1047), (570.0*0.1047));

	//EXPECT_EQ(response.linear.x,  0.0);
	EXPECT_EQ(response.angular.z, 1.0);
}
*/

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
