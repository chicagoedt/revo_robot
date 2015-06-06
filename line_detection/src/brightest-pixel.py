#!/usr/bin/env python
import sys
import numpy as np
import rospy
from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
from line_detection.cfg import LineDetectionConfig
from itertools import izip

###############################################################################
# Chicago Engineering Design Team
# Brightest Pixel filter using Python OpenCV for autonomous robot Scipio
#    (IGVC competition).
#
# This node finds removes all pixels in every frame except the brightest pixels
# from each row. If the given image was previously filtered properly, this
# should leave us with the lanes on the grass being the brightest, and this
# method would pick these pixels corresponding to the lanes (also reduces the
# amount of data to be converted later into pointclouds).
#
# @author Basheer Subei
# @email basheersubei@gmail.com

# TODO consider using parameters to pick nth brightest pixels instead of one


class BrightestPixel(LaneDetection):
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

    # this is what gets called when an image is received
    def image_callback(self, ros_image):
        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)

        # this filter needs a mono image, no colors
        roi = LaneDetection.convert_to_mono(self, cv2_image)

        roi = LaneDetection.get_roi(self, roi)

        # make an empty image
        brightest_pixels = np.zeros(roi.shape)

        # perform this brightest_pixel_number of times
        for p in xrange(self.brightest_pixel_number):
            # get indices of max pixels along each row
            indices = np.argmax(roi, axis=1)
            rows = np.arange(roi.shape[0])
            # get the values of max pixels along each row
            values = roi[rows, indices]

            # now fill the image only with the brightest_pixels from each row
            for row, (col, pix) in enumerate(izip(indices, values)):
                brightest_pixels[row, col] = pix
                roi[row, col] = 0  # remove that chosen pixel for next loop iteration

        final_image = brightest_pixels
        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )
        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "brightest_pixel"
    namespace = rospy.get_namespace()

    # create a BrightestPixel object
    bp = BrightestPixel(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("brightest_pixel", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, bp.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
