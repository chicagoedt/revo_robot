#!/usr/bin/env python
import sys
import numpy as np
import cv2
import rospy
from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
from line_detection.cfg import LineDetectionConfig

###############################################################################
# Chicago Engineering Design Team
# Blur Filter using Python OpenCV for autonomous robot Scipio
#    (IGVC competition).
#
# Runs a blur on the image
#
# @author Basheer Subei
# @email basheersubei@gmail.com


class Blur(LaneDetection):

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = self.ros_to_cv2_image(ros_image)
        if self.use_roi:
            roi = self.get_roi(cv2_image)
        else:
            roi = cv2_image
        final_image = cv2.blur(roi, (self.blur_size, self.blur_size))
        final_image_message = self.cv2_to_ros_message(final_image)
        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "blur"
    namespace = rospy.get_namespace()

    # create a BlobDetection object
    b = Blur(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("blur", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, b.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
