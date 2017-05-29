#include <ros/ros.h>
#include <state_machine/CPU.h>
#include <iomanip>
#include <subscribe.h>

//C Libraries:
#include <stdio.h>    /* Standard input/output definitions */
#include <stdlib.h>
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>

void SubscribeAndSend::CpuMessageReceived(const state_machine::CPU& msg) {
	ROS_INFO_STREAM("The cpu consumption is: " << msg.average << " %");
	averageConsumption = msg.average;
}

void SubscribeAndSend::NodeMessageReceived(const state_machine::CPU& msg) {
	ROS_INFO_STREAM("Number of running nodes: " << msg.numNodes);
	runningNodes = msg.numNodes;
}

void SerialPort(double sendCpuConsumption, double sendNumNodes) {
	int fd;
	char data[2];
	data[0] = sendNumNodes;
	data[1] = sendCpuConsumption;

	fd = open("/dev/ttyUSB1",O_RDWR | O_NOCTTY);
    if (fd <= 0)
    {
        printf("\n  Error! in Opening ttyUSB0\n");
    }
#ifdef __SERIAl_DEBUG__
    else
        printf("\n  ttyUSB0 Opened Successfully\n fd = %d\n", fd);
#endif
    struct termios SerialPortSettings;

    tcgetattr(fd, &SerialPortSettings);

    cfsetispeed(&SerialPortSettings,B115200);
    cfsetospeed(&SerialPortSettings,B115200);

    SerialPortSettings.c_cflag &= ~PARENB;   // No Parity

    SerialPortSettings.c_cflag &= ~CSTOPB;   // One stop bit

    SerialPortSettings.c_cflag &= ~CSIZE; /* Clears the Mask       */
    SerialPortSettings.c_cflag |=  CS8;   /* Set the data bits = 8 */

    SerialPortSettings.c_cflag &= ~CRTSCTS;

    SerialPortSettings.c_cflag |= (CREAD | CLOCAL);

        /* Setting Time outs */
    SerialPortSettings.c_cc[VMIN]  = 2; //Read X characters */
    SerialPortSettings.c_cc[VTIME] = 0;  /* Wait indefinitely   */

    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);

    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

    if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
        printf("\n  ERROR ! in Setting attributes");
#ifdef __SERIAl_DEBUG__

     else
        printf("\nfd = %d \n BaudRate =  115200\n  StopBits = 1 \n  Parity   = none\n", fd);
#endif

	tcflush(fd, TCIFLUSH);

    int bytes_sent = 0;

    bytes_sent = write(fd, data ,2);
    printf("Bytes sent: %d\n", bytes_sent);
    //emit Data_Ready(data);
    if(bytes_sent <= 0)
    {
        //ERROR
    }
    else if(bytes_sent == 2)
    {
        printf("Sent!");
    }

    close(fd);
}

int main(int argc, char **argv) {
	//Initialize the ROS system and become a node.
	ros::init(argc, argv, "cpuSubscriber");
	ros::NodeHandle nh;

	SubscribeAndSend subscribeObject;

	ros::Subscriber nodeSub = nh.subscribe("running_nodes", 1000,
	  &SubscribeAndSend::NodeMessageReceived, &subscribeObject);

	ros::Subscriber cpuSub = nh.subscribe("cpuMessage", 1000,
	  &SubscribeAndSend::CpuMessageReceived, &subscribeObject);
	
	ros::Publisher cpuPub = nh.advertise<state_machine::CPU>("numNodes", 1000);
	ros::Rate rate(2);	
	
	while(ros::ok()) {
	
	ros::spinOnce();
		
	rate.sleep();
	
	SerialPort(subscribeObject.averageConsumption, subscribeObject.runningNodes);
	}
}


