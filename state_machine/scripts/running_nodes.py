#!/usr/bin/env python
import rospy
import rosnode
from state_machine.msg import CPU

def talker():
	pub = rospy.Publisher('running_nodes', CPU, queue_size = 1000)
	rospy.init_node('talker')
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		msg = CPU()
		msg.names = rosnode.get_node_names()
		msg.numNodes = 0.0
		for node in msg.names:
			msg.numNodes += 1
		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
