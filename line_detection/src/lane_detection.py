#!/usr/bin/env python
import sys
import numpy as np
import cv2
import math
#import time
import rospy
import sensor_msgs
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

###############################################################################
## Chicago Engineering Design Team
## Line Detection main class using Python OpenCV for autonomous robot Scipio
##    (IGVC competition).
## @author Basheer Subei
## @email basheersubei@gmail.com


class lane_detection:

    def __init__(self, namespace, node_name):

        # grab parameters from launch file
        self.use_mono = rospy.get_param( + node_name + "/use_mono")
        self.use_compressed_format = rospy.get_param(namespace + node_name + "/use_compressed_format")
        self.subscriber_image_topic = rospy.get_param(namespace + node_name + "/subscriber_image_topic")
        self.publisher_image_topic = rospy.get_param(namespace + node_name + "/publisher_image_topic")
        self.buffer_size = rospy.get_param(namespace + node_name + "/buffer_size")

        # initialize ROS stuff

        # set publisher and subscriber

        # publisher for image of line pixels (only for debugging, not used in
        # map)
        self.line_image_pub = rospy.Publisher( namespace +
                                              "/" + node_name +
                                              self.publisher_image_topic +
                                              '/compressed',
                                              sensor_msgs.msg.CompressedImage,
                                              queue_size=1)

        if self.use_compressed_format:
        # subscriber for ROS image topic
            self.image_sub = rospy.Subscriber(self.subscriber_image_topic +
                                             "/compressed",
                                              CompressedImage, self.image_callback,
                                              queue_size=1, buff_size=self.buffer_size)
        else:
            # use this for uncompressed raw format
            self.image_sub = rospy.Subscriber(self.subscriber_image_topic,
                                           Image,
                                           self.image_callback, queue_size=1,
                                           buff_size=self.buffer_size)

        self.bridge = CvBridge()

        
        # use this if you need to use the camera_info topic (has intrinsic
        #                                                     parameters)
        # self.camera_info_sub = rospy.Subscriber("/camera/camera_info",
        #                                          CameraInfo,
        #                                          self.camera_info_callback,
        #                                          queue_size=1 )

    def reconfigure_callback(self, config, level):

        # TODO check if the keys exist in the config dictionary or else error

        self.blur_size = config['blur_size']
        self.hue_low = config['hue_low']
        self.hue_high = config['hue_high']
        self.saturation_low = config['saturation_low']
        self.saturation_high = config['saturation_high']
        self.value_low = config['value_low']
        self.value_high = config['value_high']

        # gabor filter parameters
        self.gabor_ksize = config['gabor_ksize']
        self.gabor_sigma = config['gabor_sigma']
        self.gabor_theta = config['gabor_theta']
        self.gabor_lambda = config['gabor_lambda']
        self.gabor_gamma = config['gabor_gamma']

        self.hough_rho = config['hough_rho']
        self.hough_theta = config['hough_theta']
        self.hough_threshold = config['hough_threshold']
        self.hough_min_line_length = config['hough_min_line_length']
        self.hough_max_line_gap = config['hough_max_line_gap']

        self.roi_top_left_x = config['roi_top_left_x']
        self.roi_top_left_y = config['roi_top_left_y']
        self.roi_width = config['roi_width']
        self.roi_height = config['roi_height']

        self.dilate_size = config['dilate_size']
        self.dilate_iterations = config['dilate_iterations']

        self.validate_parameters()

        return config

    # makes sure the parameters are valid and don't crash the
    # openCV calls. Changes them to valid values if invalid.
    def validate_parameters(self):
        
        # these parameters need validation:

        # blur_size can be an odd number only
        if self.blur_size % 2 == 0:
            self.blur_size -= 1

        # hue, saturation, and value parameters cannot have
        # larger or equal low limits than high limits
        if self.hue_low >= self.hue_high:
            self.hue_low = self.hue_high - 1
        if self.saturation_low >= self.saturation_high:
            self.saturation_low = self.saturation_high - 1
        if self.value_low >= self.value_high:
            self.value_low = self.value_high - 1

        # gabor filter parameters don't need validation

        # hough parameters cannot be nonzero
        if self.hough_rho <= 0:
            self.hough_rho = 1
        if self.hough_theta <= 0:
            self.hough_theta = 0.01
        if self.hough_threshold <= 0:
            self.hough_threshold = 1
        if self.hough_min_line_length <= 0:
            self.hough_min_line_length = 1
        if self.hough_max_line_gap <= 0:
            self.hough_max_line_gap = 1

        # now check if ROI parameters are out of bounds
        # only do this if image dimensions have been set
        if self.image_width > 0 and self.image_height > 0:
            if self.roi_width > self.image_width - self.roi_top_left_x:
                self.roi_width = self.image_width - self.roi_top_left_x
            if self.roi_top_left_x < 0:
                self.roi_top_left_x = 0

            if self.roi_height > self.image_height - self.roi_top_left_y:
                self.roi_height = self.image_height - self.roi_top_left_y
            if self.roi_top_left_y < 0:
                self.roi_top_left_y = 0

        if self.dilate_size % 2 == 0:
            self.dilate_size += 1
            rospy.logwarn("dilate_size should not be even! Changed to %d", self.dilate_size)
