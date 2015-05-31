#!/usr/bin/env python
import sys
import rospy
from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
from line_detection.cfg import LineDetectionConfig
import cv2
import numpy as np

###############################################################################
# Chicago Engineering Design Team
# Threshold filter node using Python OpenCV for autonomous robot Scipio
# (IGVC competition).
#
# @author Basheer Subei
# @email basheersubei@gmail.com


class Threshold(LaneDetection):

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = self.ros_to_cv2_image(ros_image)
        # this filter needs a mono image, no colors
        mono = self.convert_to_mono(cv2_image)

        roi = self.get_roi(mono)

        retval, final_image = cv2.threshold(
            roi,
            self.global_threshold,
            255,
            cv2.THRESH_BINARY
        )

        final_image_message = self.cv2_to_ros_message(final_image)
        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "threshold"
    namespace = rospy.get_namespace()

    # create a hough object
    t = Threshold(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("threshold", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, t.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
