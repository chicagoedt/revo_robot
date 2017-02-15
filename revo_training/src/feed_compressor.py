#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#from __future__ import print_function

class feed_compressor:

  def __init__(self):
    self.image_pub = rospy.Publisher("/revo_training/zed_feed",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/stereo_camera/left/image_color",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    height,width = cv_image.shape[:2]
    resized = cv2.resize(cv_image, (width/3, height/3))

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(resized, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('feed_compressor', anonymous=True)
  fc = feed_compressor()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
