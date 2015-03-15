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
# Brightest Pixel filter using Python OpenCV for autonomous robot Scipio
#    (IGVC competition).
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

    # this is what gets called when an image is recieved
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)

        # get indices of max pixels along each row
        brightest_pixel_indices = np.argmax(cv2_image, axis=1)
        # get the values of max pixels along each row
        brightest_pixel_values = np.amax(cv2_image, axis=1)

        # make an empty image
        brightest_pixels = np.zeros(cv2_image.shape)

        count = 0
        # now fill the image only with the brightest_pixels
        for index in brightest_pixel_indices:
            brightest_pixels[count, index] = brightest_pixel_values[count]
            count += 1

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
    if namespace == "/":
        namespace = ""

    # create a BrightestPixel object
    bp = BrightestPixel(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("brightest_pixel", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, bp.reconfigure_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
