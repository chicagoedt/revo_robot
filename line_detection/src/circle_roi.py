#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
from line_detection.cfg import LineDetectionConfig

###############################################################################
# Chicago Engineering Design Team
# Node to extract circular region of interest using Python OpenCV for
# autonomous robot Scipio
# (IGVC competition).
#
# @author Steven Taylor
# @email steven.taylor1001@yahoo.com.com


class Circle(LaneDetection):

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)

        cv2_image_ellipse = np.zeros(cv2_image.shape, dtype=np.uint8)
        cv2.ellipse(
            cv2_image_ellipse,
            (self.circle_center_x, self.circle_center_y),
            (self.circle_major_axis, self.circle_minor_axis),
            0,
            0,
            360,
            (255, 255, 255),
            thickness=-1
        )

        final_image = np.bitwise_and(cv2_image, cv2_image_ellipse)
        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )

        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "circle_roi"
    namespace = rospy.get_namespace()

    # create a gabor object
    c = Circle(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("circle_roi", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, c.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
