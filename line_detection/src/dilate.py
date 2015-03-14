#!/usr/bin/env python
import sys
import numpy as np
import cv2
import math
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from cv_bridge import CvBridge, CvBridgeError
from lane_detection import lane_detection
from line_detection.cfg import LineDetectionConfig

###############################################################################
## Chicago Engineering Design Team
## Dilate filter using Python OpenCV for autonomous robot Scipio
##    (IGVC competition).
## @author Basheer Subei
## @email basheersubei@gmail.com

class dilate(lane_detection):
    image_height = 0
    image_width = 0
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000
    dilate_size = 0
    dilate_iterations = 0

    def __init__(self, namespace, node_name):
        lane_detection.__init__(self, namespace, node_name)

    # this is what gets called when an image is recieved
    def image_callback(self, ROS_image):
        
        cv2_image = lane_detection.ROS_to_cv2_image(self, ROS_image)

        # dilate each pixel using kernel with dilate_size
        if self.dilate_size > 0 and self.dilate_iterations > 0:
            kernel = np.ones((self.dilate_size, self.dilate_size), np.uint8)
            final_image = cv2.dilate(cv2_image, kernel, iterations = self.dilate_iterations)
        else:
            rospy.logwarn("dilate parameters invalid! Won't perform dilate!")
            final_image = cv2_image

        #### Create CompressedImage to publish ####
        final_image_message = CompressedImage()
        final_image_message.header.stamp = rospy.Time.now()
        final_image_message.format = "jpeg"
        final_image_message.data = np.array(cv2.imencode(
                                            '.jpg',
                                            final_image)[1]).tostring()

        # publishes image message with line pixels in it
        self.line_image_pub.publish(final_image_message)
    ## end image_callback()


def main(args):
    node_name = "dilate_lanedetection"
    namespace = rospy.get_namespace()
    if namespace == "/":
        namespace = ""

    # create a dilate object
    d = dilate(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("dilate_lanedetection", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, d.reconfigure_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
