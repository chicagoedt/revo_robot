#!/usr/bin/env python
import sys
import rospy
import numpy as np
import cv2
from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
from line_detection.cfg import LineDetectionConfig

###############################################################################
# Chicago Engineering Design Team
# K-means clustering node using Python OpenCV for autonomous robot Scipio
# (IGVC competition).
#
# @author Basheer Subei
# @email basheersubei@gmail.com


class Kmeans(LaneDetection):
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

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        Z = hsv.reshape((-1,3))

        # convert to np.float32
        Z = np.float32(Z)

        # define criteria, number of clusters(K) and apply kmeans()
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        K = 5

        ret, label, center = cv2.kmeans(Z, K, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

        # Now convert back into uint8, and make original image
        center = np.uint8(center)
        res = center[label.flatten()]
        res2 = res.reshape((hsv.shape))

        tmp2 = np.uint8(label.flatten()).reshape((hsv.shape[0], hsv.shape[1], 1))
        # mask1 = np.dstack((tmp2, tmp2, tmp2))
        mask1 = tmp2
        # print np.nonzero(mask1)
        # print mask1[np.nonzero(mask1)]

        # final_image = cv2.bitwise_and(roi, mask1)
        final_image = cv2.bitwise_and(roi, roi, mask=mask1)
        # final_image = mask
        # print tmp2
        # print tmp2.shape
        # print tmp2.dtype

        # print final_image
        # print final_image.shape
        # print final_image.dtype
        # print final_image[np.nonzero(final_image)]
        # finished doing stuff to roi and save it to final_image
        # final_image = res2  # here we just set them equal which does nothing
        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )

        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "kmeans"
    namespace = rospy.get_namespace()

    # create a gabor object
    g = Kmeans(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("kmeans", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, g.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
