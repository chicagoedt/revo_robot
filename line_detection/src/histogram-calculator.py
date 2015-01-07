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

###############################################################################
## Chicago Engineering Design Team
## @author Basheer Subei
## @email basheersubei@gmail.com
#######################################################
##
## Histogram Calculator node
##
## This ROS node reads image rosbags and calculates the moving average of the
## HSV histograms, and writes it (every frame) to csv files.
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

    # this NumPy array will hold the normalized HSV histogram values
    cumulative_average_histogram = np.zeros((256,3))
    # holds total number of frames we processed already
    frames_processed = 0

    def __init__(self):

        # initialize ROS stuff

        #set hsv histogram plot ROS image publisher
        self.current_histogram_plot_pub = rospy.Publisher('/histogram_calculator/current_histogram_plot/compressed',
                                              sensor_msgs.msg.CompressedImage,
                                              queue_size=1)
        # set cumulative average histogram ROS image publisher
        self.cumulative_average_histogram_plot_pub = rospy.Publisher('/histogram_calculator/cumulative_average_histogram_plot/compressed',
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

        # TODO check whether we need the third channel (V) or only H and S
        # calculate current (for this frame) histogram for 3 HSV channels
        h = cv2.calcHist([hsv], [0], None, [180], [0, 180])
        s = cv2.calcHist([hsv], [1], None, [256], [0, 256])
        v = cv2.calcHist([hsv], [2], None, [256], [0, 256])

        # normalize the current histogram
        cv2.normalize(h, h, 0, 179, cv2.NORM_MINMAX)
        cv2.normalize(s, s, 0, 255, cv2.NORM_MINMAX)
        cv2.normalize(v, v, 0, 255, cv2.NORM_MINMAX)

        # hue range is 0-179, so we resize it to 255 and fill with zeros
        # just so it's the same size as other channels (easier to deal with)
        h.resize((256,1))

        # prepare to plot current histogram
        bins = np.arange(256).reshape(256,1)
        current_histogram_plot = np.zeros((300,256,3))

        # plot hue
        pts = np.int32(np.column_stack((bins,np.around(h))))  # prepare to plot
        cv2.polylines(current_histogram_plot,[pts],False,(0,0,255))  # plot hue points in red (bgr)

        # plot saturation
        pts = np.int32(np.column_stack((bins,np.around(s))))  # prepare to plot
        cv2.polylines(current_histogram_plot,[pts],False,(0,255,0))  # plot saturation points in green

        # plot value
        pts = np.int32(np.column_stack((bins,np.around(v))))  # prepare to plot
        cv2.polylines(current_histogram_plot,[pts],False,(255,0,0))  # plot saturation points in blue

        current_histogram_plot = np.flipud(current_histogram_plot)  # flip upside down

        #### Create CompressedImage to publish current histogram (for debugging) ####
        final_image_message = CompressedImage()
        final_image_message.header.stamp = rospy.Time.now()
        final_image_message.format = "jpeg"
        final_image_message.data = np.array(cv2.imencode(
                                            '.jpg',
                                            current_histogram_plot)[1]).tostring()
        # publishes current histogram plot image
        self.current_histogram_plot_pub.publish(final_image_message)


        # now that we're done getting and plotting the current histogram, we need to average the
        # histogram for every frame we get and store it in cumulative_average_histogram and also publish that


        # we need to calculate the cumulative moving average of the histogram
        # for every new frame.
        # the equation for cumulative moving average is:
            # new avg = (new value + (old avg. * timesteps)) / timesteps + 1
        # where timesteps is the number of frames processed so far and new avg. is the hsv histogram values
        # check wikipedia: https://en.wikipedia.org/wiki/Moving_average#Cumulative_moving_average
        
        # merge three channels together, this represents current histogram (new value)
        hsv = np.column_stack((h, s, v))

        # perform cumulative moving average on the histogram
        self.cumulative_average_histogram = (hsv + (self.cumulative_average_histogram * self.frames_processed)) / (self.frames_processed+1)

        ## now plot the moving average into a ROS image message and publish it
        # prepare to plot moving average histogram
        bins = np.arange(256).reshape(256,1)
        cumulative_average_histogram_plot = np.zeros((300,256,3))

        # plot hue
        pts = np.int32(np.column_stack((bins,np.around(self.cumulative_average_histogram[:,0]))))  # prepare to plot
        cv2.polylines(cumulative_average_histogram_plot,[pts],False,(0,0,255))  # plot hue points in red (bgr)

        # plot saturation
        pts = np.int32(np.column_stack((bins,np.around(self.cumulative_average_histogram[:,1]))))  # prepare to plot
        cv2.polylines(cumulative_average_histogram_plot,[pts],False,(0,255,0))  # plot saturation points in green

        # plot value
        pts = np.int32(np.column_stack((bins,np.around(self.cumulative_average_histogram[:,2]))))  # prepare to plot
        cv2.polylines(cumulative_average_histogram_plot,[pts],False,(255,0,0))  # plot saturation points in blue

        cumulative_average_histogram_plot = np.flipud(cumulative_average_histogram_plot)  # flip upside down

        #### Create CompressedImage to publish cumulative histogram (for debugging) ####
        final_image_message = CompressedImage()
        final_image_message.header.stamp = rospy.Time.now()
        final_image_message.format = "jpeg"
        final_image_message.data = np.array(cv2.imencode(
                                            '.jpg',
                                            cumulative_average_histogram_plot)[1]).tostring()
        # publishes current histogram plot image
        self.cumulative_average_histogram_plot_pub.publish(final_image_message)


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
        # open files to write new data to
        h_file = open(self.package_path + "/misc/training_images/h.txt", 'w')
        s_file = open(self.package_path + "/misc/training_images/s.txt", 'w')
        v_file = open(self.package_path + "/misc/training_images/v.txt", 'w')

        # write h to "h.txt"
        h_file.write(','.join(['%.5f' % val for val in self.cumulative_average_histogram[:,0] ]))
        # write s to "s.txt"
        s_file.write(','.join(['%.5f' % val for val in self.cumulative_average_histogram[:,1] ]))
        # write v to "v.txt"
        v_file.write(','.join(['%.5f' % val for val in self.cumulative_average_histogram[:,2] ]))

        # done with writing, close files
        h_file.close()
        s_file.close()
        v_file.close()
    
        # increment number of frames since this frame is done
        self.frames_processed += 1

    ## end image_callback()

    def reconfigure_callback(self, config, level):

        # TODO check if the keys exist in the config dictionary or else error
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
