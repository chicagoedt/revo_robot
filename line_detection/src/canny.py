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
# Canny Edge detector using Python OpenCV for autonomous robot Scipio
#    (IGVC competition).
#
# Filters the image using canny edge detector
#
# @author Basheer Subei
# @email basheersubei@gmail.com


class Canny(LaneDetection):
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)
        roi = LaneDetection.get_roi(self, cv2_image)

        final_image = cv2.Canny(roi, 100, 10)

        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )
        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "canny"
    namespace = rospy.get_namespace()

    # create a BlobDetection object
    c = Canny(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("canny", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, c.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
