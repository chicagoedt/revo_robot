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
# Hough Transform using Python OpenCV for autonomous robot Scipio
# (IGVC competition).
#
# This node runs the hough transform on input images, draws lines from the
# hough output, and publishes these images.
#
# @author Basheer Subei
# @email basheersubei@gmail.com


class Hough(LaneDetection):
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000
    hough_rho = 1
    hough_theta = 0.01
    hough_threshold = 100
    hough_min_line_length = 60
    hough_max_line_gap = 7
    hough_thickness = 1
    hough_number_of_lines = 2

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)
        # this filter needs a mono image, no colors
        roi = LaneDetection.convert_to_mono(self, cv2_image)

        roi = LaneDetection.get_roi(self, roi)

        # apply hough line transform
        lines = cv2.HoughLinesP(
            roi,
            self.hough_rho,
            self.hough_theta,
            self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap)
        # TODO remove hough lines that are very close together

        final_image = np.zeros(roi.shape)
        if lines is not None:
            number_of_lines = min(self.hough_number_of_lines, len(lines[0]))
            # draw only the first number_of_lines lines
            for x1, y1, x2, y2 in lines[0, :number_of_lines]:
                cv2.line(
                    final_image,
                    (x1, y1),
                    (x2, y2),
                    255,
                    self.hough_thickness
                )

        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )
        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "hough"
    namespace = rospy.get_namespace()

    # create a hough object
    h = Hough(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("hough", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, h.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
