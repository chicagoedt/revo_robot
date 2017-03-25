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

img = cv2.imread('frame.jpg')
height, width = img.shape[:2]
fourcc = cv2.cv.CV_FOURCC(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (width,height))

class feed_writer:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/stereo_camera/left/image_color",Image,self.callback)
    self.fourcc = cv2.cv.CV_FOURCC(*'XVID')

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #height,width = cv_image.shape[:2]
    #resized = cv2.resize(cv_image, (width/3, height/3))

    try:
        out.write(cv_image)
        cv2.imwrite('frame.jpg',cv_image)
    except CvBridgeError as e:
      print(e)

def myhook():
    out.release()
    print "Releasing..."

def main(args):
  rospy.init_node('feed_writer', anonymous=True)
  fw = feed_writer()
  try:
    rospy.spin()
    rospy.on_shutdown(myhook)
  except KeyboardInterrupt:
    fw.out.release()
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
