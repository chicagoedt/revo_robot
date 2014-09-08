#include <gtest/gtest.h>
#include "../src/roboteq_driver.h"

TEST(TestRoboteq, testZeroCase)
{
	geometry_msgs::Twist response;

	response = convertWheelVelocityToTwist(0.0, 0.0);

	EXPECT_EQ(response.linear.x,  0.0);
	EXPECT_EQ(response.angular.z, 0.0);
}

TEST(TestRoboteq, testTurnCase)
{
	geometry_msgs::Twist response;

	response = convertWheelVelocityToTwist(-1.0, 1.0);

	EXPECT_EQ(response.linear.x,  0.0);
	EXPECT_EQ(response.angular.z, -1.0);
}

TEST(TestRoboteq, testGoStraightForwardCase)
{
	geometry_msgs::Twist response;

	response = convertWheelVelocityToTwist(1.0, 1.0);

	EXPECT_EQ(response.linear.x,  1.0);
	EXPECT_EQ(response.angular.z, 0.0);
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
