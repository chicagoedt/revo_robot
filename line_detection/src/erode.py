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
# Erosion filter using Python OpenCV for autonomous robot Scipio
# (IGVC competition).
#
#
# @author Basheer Subei
# @email basheersubei@gmail.com


class Erode(LaneDetection):
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000
    max_erode_iterations = 150

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = self.ros_to_cv2_image(ros_image)
        # this filter needs a mono image, no colors
        mono = self.convert_to_mono(cv2_image)

        if self.use_roi:
            roi = self.get_roi(mono)
        else:
            roi = mono

        # erode image
        size = np.size(roi)
        temp = roi.copy()
        eroded = np.zeros(roi.shape, np.uint8)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        # iteratively erode the image until it's 1 pixel thick
        for i in range(self.max_erode_iterations):
            temp = cv2.erode(temp, element)

            zeros = size - cv2.countNonZero(roi)
            # if all pixels are zero, enough eroding
            if zeros == size:
                break
            # otherwise save current copy
            eroded = temp

        final_image = eroded
        final_image_message = self.cv2_to_ros_message(final_image)

        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "erode"
    namespace = rospy.get_namespace()

    # create a Skeletonize object
    e = Erode(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("erode", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, e.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
