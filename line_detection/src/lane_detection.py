#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import sensor_msgs
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospkg

###############################################################################
# Chicago Engineering Design Team
# Lane Detection main class (interface) using Python OpenCV for autonomous
# Robot Scipio (IGVC competition).
# @author Basheer Subei
# @email basheersubei@gmail.com
###############################################################################


class LaneDetection(object):

    def __init__(self, namespace, node_name):

        # grab parameters from launch file
        self.use_mono = rospy.get_param(
            namespace + node_name + "/use_mono"
        )
        self.use_compressed_format = rospy.get_param(
            namespace + node_name + "/use_compressed_format"
        )
        self.subscriber_image_topic = rospy.get_param(
            namespace + node_name + "/subscriber_image_topic"
        )
        self.publisher_image_topic = rospy.get_param(
            namespace + node_name + "/publisher_image_topic"
        )
        self.buffer_size = rospy.get_param(
            namespace + node_name + "/buffer_size"
        )
        self.package_path = rospkg.RosPack().get_path('line_detection')
        # initialize ROS stuff

        # set publisher and subscriber

        # publisher for image of line pixels (only for debugging, not used in
        # map)
        self.line_image_pub = rospy.Publisher(
            namespace + "/" + node_name + self.publisher_image_topic +
            '/compressed',
            sensor_msgs.msg.CompressedImage,
            queue_size=1
        )

        if self.use_compressed_format:
            # subscriber for ROS image topic
            self.image_sub = rospy.Subscriber(
                self.subscriber_image_topic + "/compressed",
                CompressedImage,
                self.image_callback,
                queue_size=1,
                buff_size=self.buffer_size
            )
        else:
            # use this for uncompressed raw format
            self.image_sub = rospy.Subscriber(
                self.subscriber_image_topic,
                Image,
                self.image_callback,
                queue_size=1,
                buff_size=self.buffer_size
            )

        self.bridge = CvBridge()

        # use this if you need to use the camera_info topic (has intrinsic
        #                                                     parameters)
        # self.camera_info_sub = rospy.Subscriber("/camera/camera_info",
        #                                          CameraInfo,
        #                                          self.camera_info_callback,
        #                                          queue_size=1 )

    def reconfigure_callback(self, config, level):
        self.global_threshold = config['global_threshold']
        self.global_threshold_factor = config['global_threshold_factor']
        self.adaptive_threshold_block_size = config[
            'adaptive_threshold_block_size'
        ]
        self.adaptive_threshold_C = config['adaptive_threshold_C']
        self.blur_size = config['blur_size']
        self.canny_threshold = config['canny_threshold']
        self.max_erode_iterations = config['max_erode_iterations']
        self.bandpass_low_cutoff = config['bandpass_low_cutoff']
        self.bandpass_high_cutoff = config['bandpass_high_cutoff']
        self.hue_low = config['hue_low']
        self.hue_high = config['hue_high']
        self.saturation_low = config['saturation_low']
        self.saturation_high = config['saturation_high']
        self.value_low = config['value_low']
        self.value_high = config['value_high']
        self.backprojection_kernel_size = config['backprojection_kernel_size']
        self.histogram_view_value = config['histogram_view_value']
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
        self.hough_thickness = config['hough_thickness']
        self.roi_top_left_x = config['roi_top_left_x']
        self.roi_top_left_y = config['roi_top_left_y']
        self.roi_width = config['roi_width']
        self.roi_height = config['roi_height']
        self.dilate_size = config['dilate_size']
        self.dilate_iterations = config['dilate_iterations']
        self.image_width = config['image_width']
        self.image_height = config['image_height']
        # check that all these params are legal values
        self.validate_parameters()
        return config

    # makes sure the parameters are valid and don't crash the
    # openCV calls. Changes them to valid values if invalid.
    def validate_parameters(self):
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

        # dilate kernel size cannot be even
        if self.dilate_size % 2 == 0:
            self.dilate_size += 1
            rospy.logwarn("dilate_size should not be even! Changed to %d",
                          self.dilate_size)

    def cv2_to_ros_message(self, image):
        # Create CompressedImage to publish
        final_image_message = CompressedImage()
        final_image_message.header.stamp = rospy.Time.now()
        final_image_message.format = "jpeg"
        final_image_message.data = np.array(
            cv2.imencode('.jpg', image)[1]).tostring()
        return final_image_message

    def ros_to_cv2_image(self, image):
        # if it's not compressed and isn't actually mono,
        # but we expected mono, then error
        if (self.use_mono and
                not self.use_compressed_format and
                image.encoding != 'mono8'):
            rospy.logerr("image is not mono8! Aborting!")
            return

        if self.use_compressed_format:
            # direct conversion from ROS CompressedImage to CV2 #
            np_arr = np.fromstring(image.data, np.uint8)
            if self.use_mono:
                img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_GRAYSCALE)
            else:
                img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        else:
            # direct conversion from ROS Image to CV2
            # first, we need to convert image from sensor_msgs/Image to numpy
            # (or cv2). For this, we use cv_bridge
            try:
                img = self.bridge.imgmsg_to_cv2(image,
                                                desired_encoding="passthrough")
            except CvBridgeError, e:
                rospy.logerr(e)
        if img is None:
            rospy.logerr("error! img is empty!")
            return
        self.image_height = img.shape[0]
        self.image_width = img.shape[1]
        return img

    def get_roi(self, image):
        # if mono, don't take 3rd dimension since there's only one channel
        if self.use_mono:
            roi = image[
                self.roi_top_left_y:self.roi_top_left_y + self.roi_height,
                self.roi_top_left_x:self.roi_top_left_x + self.roi_width,
            ]
        else:
            roi = image[
                self.roi_top_left_y:self.roi_top_left_y + self.roi_height,
                self.roi_top_left_x:self.roi_top_left_x + self.roi_width,
                :
            ]

        # in case roi settings aren't correct, just use the entire image
        if roi.size <= 0:
            rospy.logerr("Incorrect roi settings! Will use the entire image\
                         instead!")
            roi = image
        return roi
