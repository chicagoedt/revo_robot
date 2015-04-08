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
# Blob Detector using Python OpenCV for autonomous robot Scipio
#    (IGVC competition).
#
# Finds N biggest blobs of pixels and filters everything else out.
#
# @author Basheer Subei
# @email basheersubei@gmail.com


class BlobDetection(LaneDetection):
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # # Change thresholds
    # params.minThreshold = 200
    # params.maxThreshold = 255

    # params.minDistBetweenBlobs = 100

    # # Filter by Area.
    # params.filterByArea = True
    # params.minArea = 1000

    # # Don't filter by Circularity
    # params.filterByCircularity = False

    # # Don't filter by Convexity
    # params.filterByConvexity = False

    # # Filter by Inertia
    # params.filterByInertia = True
    # params.minInertiaRatio = 0.01
    # params.maxInertiaRatio = 0.2

    # Change thresholds
    params.minThreshold = 10
    params.maxThreshold = 200
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 100
     
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
     
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.87
     
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01

    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3:
        detector = cv2.SimpleBlobDetector(params)
    else:
        detector = cv2.SimpleBlobDetector_create(params)

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)
        roi = LaneDetection.get_roi(self, cv2_image)

        # Detect blobs
        keypoints = self.detector.detect(roi)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        final_image = cv2.drawKeypoints(
            roi, keypoints,
            np.array([]),
            (0, 0, 255),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )

        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )
        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "blob_detection"
    namespace = rospy.get_namespace()

    # create a BlobDetection object
    b = BlobDetection(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("blob_detection", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, b.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
