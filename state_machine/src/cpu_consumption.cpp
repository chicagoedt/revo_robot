#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ros/ros.h>
#include <state_machine/CPU.h>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cpuMessage");
	ros::NodeHandle nh;

	ros::Publisher cpuPublisher = nh.advertise<state_machine::CPU>("cpuMessage", 1000);
	ros::Rate loop_rate(1);

    long double cpu1[7], cpu2[7], totalCPU1, totalCPU2, totalOverPeriod,
	  workCPU1, workCPU2, workOverPeriod, averageConsumption;
    FILE *fp;
    int i;

    while(ros::ok())
    {
	state_machine::CPU msg;

        fp = fopen("/proc/stat", "r");
        fscanf(fp, "%*s %LF %LF %LF %LF %LF %LF %LF", &cpu1[0], &cpu1[1], &cpu1[2], &cpu1[3], &cpu1[4], &cpu1[5], &cpu1[6]);
        fclose(fp);

        totalCPU1 = 0.00;
        for(i = 0; i < 7; ++i) {
            totalCPU1 += cpu1[i];
        }
        workCPU1 = 0.00;
        for (i = 0; i < 3; ++i) {
            workCPU1 += cpu1[i];
        }

        usleep(1000000);

        fp = fopen("/proc/stat", "r");
        fscanf(fp, "%*s %LF %LF %LF %LF %LF %LF %LF", &cpu2[0], &cpu2[1], &cpu2[2], &cpu2[3], &cpu2[4], &cpu2[5], &cpu2[6]);
        fclose(fp);

        totalCPU2 = 0.00;
        for(i = 0; i < 7; ++i) {
            totalCPU2 += cpu2[i];
        }
        workCPU2 = 0.00;
        for (i = 0; i < 3; ++i) {
            workCPU2 += cpu2[i];
        }

        workOverPeriod = workCPU2 - workCPU1;
        totalOverPeriod = totalCPU2 - totalCPU1;

        averageConsumption = (workOverPeriod / totalOverPeriod) * 100;

        // printf("The current CPU utilization is : %Lf \n", averageConsumption);
		msg.average = averageConsumption;
		ROS_INFO_STREAM("The current CPU utilization is:" << msg.average);
		cpuPublisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
    }

    return(0);
}

