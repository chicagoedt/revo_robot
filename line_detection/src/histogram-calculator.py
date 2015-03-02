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
import rospkg
from dynamic_reconfigure.server import Server
from line_detection.cfg import LineDetectionConfig
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


###############################################################################
## Chicago Engineering Design Team
## @author Basheer Subei
## @email basheersubei@gmail.com
#######################################################
##
## Histogram Calculator node
##
## This ROS node reads image rosbags and sums up the
## HSV histograms, and writes it (every frame) to a text file.
##
## The histogram can then be used as input to the histogram backprojection node.
##
###############################################################################


class line_detection:

    node_name = "histogram_calculator_lanedetection"
    namespace = rospy.get_namespace()
    if namespace == "/":
        namespace = ""

    use_compressed_format = rospy.get_param(rospy.get_namespace() + node_name + "/use_compressed_format")
    subscriber_image_topic = rospy.get_param(rospy.get_namespace() + node_name + "/subscriber_image_topic")
    buffer_size = rospy.get_param(rospy.get_namespace() + node_name + "/buffer_size")

    package_path = ''
    
    roi_top_left_x = 16
    roi_top_left_y = 143
    roi_width = 2000
    roi_height = 2000


    image_height = 0
    image_width = 0

    histogram_dimension = rospy.get_param(rospy.get_namespace() + node_name + "/histogram_dimension")
    
    # initializes 2d histogram
    # cumulative_histogram = np.zeros((histogram_dimension,histogram_dimension,histogram_dimension))
    # cumulative_histogram = np.zeros((histogram_dimension,histogram_dimension))
    cumulative_histogram = np.zeros((180,256), dtype=float)

    # holds total number of frames we processed already
    frames_processed = 0

    def __init__(self):

        # initialize ROS stuff

        #set hsv histogram plot ROS image publisher
        self.current_histogram_plot_pub = rospy.Publisher('/histogram_calculator/current_histogram_plot/compressed',
                                              sensor_msgs.msg.CompressedImage,
                                              queue_size=1)
        # set cumulative histogram ROS image publisher
        self.cumulative_histogram_plot_pub = rospy.Publisher('/histogram_calculator/cumulative_histogram_plot/compressed',
                                              sensor_msgs.msg.CompressedImage,
                                              queue_size=1)
        # set ROS image publisher that publishes whatever it subscribes to + ROI changes
        self.input_image_pub = rospy.Publisher('/histogram_calculator/input_image/compressed',
                                              sensor_msgs.msg.CompressedImage,
                                              queue_size=1)

        # set subscriber

        # this returns the path to the current package
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('line_detection')

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

    ## end __init__

    # this is what gets called when an image is recieved
    def image_callback(self, image):

        # use this to record start time for each frame
        #start_time = time.time()

        
        if self.use_compressed_format:
            #### direct conversion from ROS CompressedImage to CV2 ####
            np_arr = np.fromstring(image.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        else:
            #### direct conversion from ROS Image to CV2 ####
            # first, we need to convert image from sensor_msgs/Image to numpy (or
            # cv2). For this, we use cv_bridge
            try:
                # img = self.bridge.imgmsg_to_cv2(image, "bgr8")
                img = self.bridge.imgmsg_to_cv2(image,
                                                desired_encoding="passthrough")
            except CvBridgeError, e:
                rospy.logerr(e)

        if img is None:
            rospy.logerr("error! img is empty!")
            return

        self.image_height = img.shape[0]
        self.image_width = img.shape[1]

        roi = img[
            self.roi_top_left_y:self.roi_top_left_y + self.roi_height,
            self.roi_top_left_x:self.roi_top_left_x + self.roi_width,
            :
        ]

        # in case roi settings aren't correct, just use the entire image
        if roi.size <= 0:
            rospy.logerr("Incorrect roi settings! Will use the entire image instead!")
            roi = img

        # convert from BGR to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # histogram = cv2.calcHist([hsv],
        #     [0,1,2],
        #     None,
        #     [self.histogram_dimension,self.histogram_dimension,self.histogram_dimension],
        #     [0,180,0,256,0,256])
        histogram = cv2.calcHist([hsv],
            [0,1],
            None,
            [180,256],
            [0,179,0,255])


        # we'll just take a slice (at a V channel value) of the 3d histogram and plot it in 2d for debugging
        # current_histogram_plot = histogram[:,:,self.histogram_view_value]
        current_histogram_plot = histogram


        # #### Create CompressedImage to publish current histogram (for debugging) ####
        final_image_message = CompressedImage()
        final_image_message.header.stamp = rospy.Time.now()
        final_image_message.format = "jpeg"
        final_image_message.data = np.array(cv2.imencode(
                                            '.jpg',
                                            current_histogram_plot)[1]).tostring()
        # publishes current histogram plot image
        self.current_histogram_plot_pub.publish(final_image_message)


        # just add the current histogram to the cumulative histogram
        self.cumulative_histogram += histogram

        # we'll just take a slice (at a V channel value) of the 3d histogram and plot it in 2d for debugging
        # cumulative_histogram_plot = self.cumulative_histogram[:,:,self.histogram_view_value]
        cumulative_histogram_plot = self.cumulative_histogram

        # #### Create CompressedImage to publish cumulative histogram (for debugging) ####
        final_image_message = CompressedImage()
        final_image_message.header.stamp = rospy.Time.now()
        final_image_message.format = "jpeg"
        final_image_message.data = np.array(cv2.imencode(
                                            '.jpg',
                                            cumulative_histogram_plot)[1]).tostring()
        # publishes current histogram plot image
        self.cumulative_histogram_plot_pub.publish(final_image_message)


        #### Create CompressedImage to publish roi (for debugging) ####
        final_image_message = CompressedImage()
        final_image_message.header.stamp = rospy.Time.now()
        final_image_message.format = "jpeg"
        final_image_message.data = np.array(cv2.imencode(
                                            '.jpg',
                                            roi)[1]).tostring()
        # publishes current histogram plot image
        self.input_image_pub.publish(final_image_message)

        # now write the new cumulative histogram data to a file
        np.savetxt(self.package_path + "/misc/training_images/histogram.txt", self.cumulative_histogram)

        # increment number of frames since this frame is done
        self.frames_processed += 1

    ## end image_callback()

    def reconfigure_callback(self, config, level):

        self.roi_top_left_x = config['roi_top_left_x']
        self.roi_top_left_y = config['roi_top_left_y']
        self.roi_width = config['roi_width']
        self.roi_height = config['roi_height']

        self.validate_parameters()

        return config

    # makes sure the parameters are valid and don't crash the
    # openCV calls. Changes them to valid values if invalid.
    def validate_parameters(self):
        
        # these parameters need validation:

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

def main(args):
    # create a line_detection object
    ld = line_detection()

    # start the line_detector node and start listening
    rospy.init_node("line_detection", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, ld.reconfigure_callback)
    rospy.spin()
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
