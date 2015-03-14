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
# Backprojection filter using Python OpenCV for autonomous robot Scipio
#    (IGVC competition).
# @author Basheer Subei
# @email basheersubei@gmail.com


class Backproject(LaneDetection):
    image_height = 0
    image_width = 0
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000

    # hsv range of values to consider for backprojection
    hue_low = 20
    hue_high = 50

    saturation_low = 0
    saturation_high = 255

    backprojection_kernel_size = 5
    histogram = np.zeros((180, 256), dtype=float)

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)
        self.histogram = np.loadtxt(
            self.package_path + "/misc/training_images/histogram.txt"
        )
        cv2.normalize(self.histogram, self.histogram, 0, 255, cv2.NORM_MINMAX)

    # this is what gets called when an image is recieved
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)

        # run backprojection to remove grass
        mask = self.get_backprojection_mask(cv2_image)
        mask = np.dstack((mask, mask, mask))
        # only include pixels from thresh
        final_image = cv2.bitwise_and(cv2_image, mask)

        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )
        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()

    # returns mask based on backprojection
    def get_backprojection_mask(self, image):

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # begin HISTOGRAM BACKPROJECTION
        # backprojection on 3d histogram
        backproject = cv2.calcBackProject(
            [hsv], [0, 1], self.histogram,
            [
                self.hue_low, self.hue_high,
                self.saturation_low, self.saturation_high
            ],
            1
        )

        dst = backproject
        # Now convolute with circular disc
        disc = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.backprojection_kernel_size, self.backprojection_kernel_size)
        )
        cv2.filter2D(dst, -1, disc, dst)  # do we need this convolution???

        # invert dst (because the backprojection chooses what we DON'T want)
        dst = 255 - dst

        return dst
        # end HISTOGRAM BACKPROJECTION


def main(args):
    node_name = "backproject"
    namespace = rospy.get_namespace()
    if namespace == "/":
        namespace = ""

    # create a Backproject object
    bp = Backproject(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("backproject", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, bp.reconfigure_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
