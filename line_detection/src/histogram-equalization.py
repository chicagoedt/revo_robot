#!/usr/bin/env python
import sys
import cv2
import numpy as np
import rospy
from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
from line_detection.cfg import LineDetectionConfig

###############################################################################
# Chicago Engineering Design Team
# Histogram Equalization node using Python OpenCV for autonomous robot Scipio
# (IGVC competition).
#
# This node performs histogram equalization on input images and publishes them.
#
# TODO consider using adaptive CLAHE when OpenCV 3.0 works on ROS Indigo
#
# @author Basheer Subei
# @email basheersubei@gmail.com


class HistogramEqualization(LaneDetection):
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)
        self.use_adaptive_histogram = rospy.get_param(
            namespace + node_name + "/use_adaptive_histogram",
            False
        )

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)

        roi = LaneDetection.get_roi(self, cv2_image)

        if self.use_adaptive_histogram:
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))

        # given RGB images, it equalizes histogram for each channel separately!
        if roi.ndim == 3:
            if self.use_adaptive_histogram:
                r = clahe.apply(roi[:, :, 0])
                g = clahe.apply(roi[:, :, 1])
                b = clahe.apply(roi[:, :, 2])
            else:
                r = cv2.equalizeHist(roi[:, :, 0])
                g = cv2.equalizeHist(roi[:, :, 1])
                b = cv2.equalizeHist(roi[:, :, 2])
            final_image = np.dstack((r, g, b))
        elif roi.ndim == 2:
            if self.use_adaptive_histogram:
                final_image = clahe.apply(roi)
            else:
                final_image = cv2.equalizeHist(roi)
        else:
            rospy.logerr("unknown color format! Won't perform equalization!")
            final_image = roi

        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )

        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "histogram_equalization"
    namespace = rospy.get_namespace()

    # create a HistogramEqualization object
    he = HistogramEqualization(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("histogram_equalization", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, he.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
