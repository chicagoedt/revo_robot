#ifndef Subscribe_h
#define Subscribe_h

#include <ros/ros.h>
#include <state_machine/CPU.h>

class SubscribeAndSend 
{
public:
	void CpuMessageReceived(const state_machine::CPU& msg);
	void NodeMessageReceived(const state_machine::CPU& msg);
	void RamMessageReceived(const state_machine::CPU& msg);
	double averageConsumption;
	double runningNodes;
	double ramConsumption;
};

#endif

