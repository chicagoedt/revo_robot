#include <ros/ros.h>
#include "sys/types.h"
#include "sys/sysinfo.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <state_machine/CPU.h>

struct sysinfo memInfo;

int main(int argc, char **argv) {
	sysinfo (&memInfo);
	long long totalVirtualMem = memInfo.totalram;
	totalVirtualMem += memInfo.totalswap;
	totalVirtualMem *= memInfo.mem_unit;
	long long ramConsumption = memInfo.totalram - memInfo.freeram;
	//Multiply in net statement to avoid int overflow on right hand side
	ramConsumption *= memInfo.mem_unit;
	ramConsumption = (double)ramConsumption;
	ramConsumption = ramConsumption;

	ros::init(argc, argv, "ramMessage");
	ros::NodeHandle nh;

	ros::Publisher ramPublisher = nh.advertise<state_machine::CPU> ("ramMessage",
	  1000);
	ros::Rate loop_rate(1);
	
	while(ros::ok()) 
	{
	state_machine::CPU msg;
	msg.ram = ramConsumption;

	ROS_INFO_STREAM("Ram consumption is: " << msg.ram);	

	ramPublisher.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();
	}

	return 0;
}
