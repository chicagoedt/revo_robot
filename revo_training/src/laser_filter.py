#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class laser_filter:

  def __init__(self):
    self.pub = rospy.Publisher("/scan/filtered",LaserScan, queue_size=10)

    self.sub = rospy.Subscriber("/scan",LaserScan,self.callback)

  def callback(self,data):
    ranges = list(data.ranges)
    for place, value in enumerate(ranges):
        if value == 0.0:
            ranges[place] = 9.0
            #print ranges

    newdata = data
    newdata.ranges = tuple(ranges)
    self.pub.publish(newdata)
    #print newdata

def main(args):
  rospy.init_node('laser_filter', anonymous=True)
  lf = laser_filter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
