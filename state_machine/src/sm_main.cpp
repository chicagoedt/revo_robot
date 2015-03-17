#include "state_machine.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "duo3d_node");

    StateMachineBase stateMachine;

    stateMachine.run();

	return 0;
}
