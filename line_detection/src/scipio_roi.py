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
# Node to extact region of interest of Scipio using Python OpenCV for autonomous robot Scipio
# (IGVC competition).
#
# @author Steven Taylor
# @email steven.taylor1001@yahoo.com.com


class ScipioROI(LaneDetection):

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

        print(namespace + node_name + '/bottom_left_y')
        ## Scipio ROI points ##
        # bottom-left x coordinate of Scipio ROI
        self.scipio_roi_bottom_left_x = rospy.get_param(
            namespace + node_name + '/bottom_left_x',
            0
        )

        # bottom-left y coordinate of Scipio ROI
        self.scipio_roi_bottom_left_y = rospy.get_param(
            namespace + node_name + '/bottom_left_y',
            0
        )
        # bottom-right x coordinate of Scipio ROI
        self.scipio_roi_bottom_right_x = rospy.get_param(
            namespace + node_name + '/bottom_right_x',
            0
        )
        # bottom-right y coordinate of Scipio ROI
        self.scipio_roi_bottom_right_y = rospy.get_param(
            namespace + node_name + '/bottom_right_y',
            0
        )
        # mid-right x coordinate of Scipio ROI
        self.scipio_roi_mid_right_x = rospy.get_param(
            namespace + node_name + '/mid_right_x',
            0
        )
        # mid-right y coordinate of Scipio ROI
        self.scipio_roi_mid_right_y = rospy.get_param(
            namespace + node_name + '/mid_right_y',
            0
        )
        # top-right x coordinate of Scipio ROI
        self.scipio_roi_top_right_x = rospy.get_param(
            namespace + node_name + '/top_right_x',
            0
        )
        # top-right y coordinate of Scipio ROI
        self.scipio_roi_top_right_y = rospy.get_param(
            namespace + node_name + '/top_right_y',
            0
        )
        # top-left x coordinate of Scipio ROI
        self.scipio_roi_top_left_x = rospy.get_param(
            namespace + node_name + '/top_left_x',
            0
        )
        # top-left y coordinate of Scipio ROI
        self.scipio_roi_top_left_y = rospy.get_param(
            namespace + node_name + '/top_left_y',
            0
        )
        # mid-left x coordinate of Scipio ROI
        self.scipio_roi_mid_left_x = rospy.get_param(
            namespace + node_name + '/mid_left_x',
            0
        )
        # mid-left y coordinate of Scipio ROI
        self.scipio_roi_mid_left_y = rospy.get_param(
            namespace + node_name + '/mid_left_y',
            0
        )


    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)
        
        cv2_image_scipio = np.zeros(cv2_image.shape,dtype=np.uint8)
#        scipio_poly = np.array([ [0, 1079], [1919, 1079], [1850, 900], [1600, 750], [319, 750], [69, 900] ], np.int32)
        scipio_poly = np.array([ [self.scipio_roi_bottom_left_x, self.scipio_roi_bottom_left_y], 
    [self.scipio_roi_bottom_right_x, self.scipio_roi_bottom_right_y], 
    [self.scipio_roi_mid_right_x, self.scipio_roi_mid_right_y], 
    [self.scipio_roi_top_right_x, self.scipio_roi_top_right_y], 
    [self.scipio_roi_top_left_x, self.scipio_roi_top_left_y],
    [self.scipio_roi_mid_left_x, self.scipio_roi_mid_left_y] ], np.int32)
        cv2.fillConvexPoly(cv2_image_scipio, scipio_poly, (255,255,255))
            
#        print(cv2_image_ellipse.dtype)
#        final_image = (cv2_image == 1) & (cv2_image_ellipse == 1)        
#        print(cv2_image.dtype)
        cv2_image_scipio_inverted = np.bitwise_not(cv2_image_scipio)
        final_image = np.bitwise_and(cv2_image, cv2_image_scipio_inverted)
#        final_image = cv2_image_scipio
        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )

        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "scipio_roi"
    namespace = rospy.get_namespace()

    # create a gabor object
    scipio_roi = ScipioROI(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node(node_name, anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, scipio_roi.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
