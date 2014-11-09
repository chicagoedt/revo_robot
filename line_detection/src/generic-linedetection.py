#!/usr/bin/env python
import sys
import numpy as np
import cv2
#import time
import rospy
import sensor_msgs
from sensor_msgs.msg import CompressedImage
import rospkg
from dynamic_reconfigure.server import Server
from line_detection.cfg import LineDetectionConfig

###############################################################################
## Chicago Engineering Design Team
## Line Detection using Python OpenCV for autonomous robot Scipio
##    (IGVC competition).
## @author Basheer Subei
## @email basheersubei@gmail.com
#######################################################
## line detection overview as of 3/27/2014:
##
## main procedure:
## 1. GUI elements created
## 2. image rendered for first time
## 3. wait for user key and render image if trackbars change
##
## Rendering:
## 1. obtain image and region of interest (ROI)
## 2. mask out green areas using hsv filter (uses absolute values, not
##    relative yet)
## 3. gaussian blur (removes high-frequency noise)
## 4. global threshold (depends on brightness or something)
## 5. equalize histogram (not adaptively)
## 6. adaptive threshold
## 7. display image
##
###############################################################################


class line_detection:

    # this is where we define our variables in the class.
    # these are changed dynamically using dynamic_reconfig and affect
    # the image processing algorithm. A lot of these are not used in the
    # current algorithm.
    global_threshold = 160
    global_threshold_factor = 2
    adaptive_threshold_block_size = 191
    adaptive_threshold_C = 30
    blur_size = 49
    canny_threshold = 100
    max_erode_iterations = 100
    bandpass_low_cutoff = 1
    bandpass_high_cutoff = 30

    # hsv threshold variables
    hue_low = 20
    hue_high = 50

    saturation_low = 0
    saturation_high = 255

    value_low = 0
    value_high = 255

    backprojection_threshold = 50

    training_file_name = 'training_for_backprojection_1.png'
    package_path = ''

    def __init__(self):

        # initialize ROS stuff

        # set publisher and subscriber

        # publisher for pointcloud data.
        # the code for this is not implemented yet.
        # self.line_pub = rospy.Publisher(
        #    'line_data',
        #    sensor_msgs.msg.PointCloud2)

        # publisher for image of line pixels (only for debugging, not used in
        # map)
        self.line_image_pub = rospy.Publisher('line_image/compressed',
                                              sensor_msgs.msg.CompressedImage)

        # subscriber for ROS image topic
        self.image_sub = rospy.Subscriber("/camera/image_raw/compressed",
                                          CompressedImage, self.image_callback,
                                          queue_size=1)

        # this returns the path to the current package
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('line_detection')

        # use this for uncompressed raw format
        # self.image_sub = rospy.Subscriber("/camera/image_raw", Image,
        #                                    self.image_callback, queue_size=1)
        
        # use this if you need to use the camera_info topic (has intrinsic
        #                                                     parameters)
        # self.camera_info_sub = rospy.Subscriber("/camera/camera_info",
        #                                          CameraInfo,
        #                                          self.camera_info_callback,
        #                                          queue_size=1 )
    
    # returns packprojection image as OpenCV image in HSV format
    def get_backprojection_training_image(self, filename):
        
        training_file_path = (self.package_path
                              + '/misc/training_images/'
                              + filename)
        # reads in the backprojection training image from file
        backprojection_training = cv2.imread(training_file_path)
        # converts it from BGR to HSV
        backprojection_training = cv2.cvtColor(backprojection_training,
                                               cv2.COLOR_BGR2HSV)
        return backprojection_training

    # returns mask based on backprojection
    def get_backprojection_mask(self, image, filename):

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # make HSV histogram and look for regions with lots of green Hue. For
        # those regions find their range of HSV so that they can be masked out.
        # Grass Hue ranges from 20:50 , Saturation ranges from 50:125 , Value
        # ranges from 0:150.
        # White lines Hue ranges from 120:150 , Saturation ranges from 0:50 ,
        # Value ranges from 150:255.
        # all above values are in OpenCV HSV ranges

        # note that Values depend on overall brightness (need to use adaptive
        # method or dynamic one).

        backprojection_training = self.get_backprojection_training_image(
            filename)
        
        ## begin HISTOGRAM BACKPROJECTION
        # calculating object histogram
        roihist = cv2.calcHist([backprojection_training],
                               [0, 1],
                               None,
                               [180, 256],
                               [self.hue_low,
                                self.hue_high,
                                0,
                                256]
                               )

        # normalize histogram and apply backprojection
        cv2.normalize(roihist, roihist, 1, 255, cv2.NORM_MINMAX)

        dst = cv2.calcBackProject([hsv], [0, 1], roihist,
                                  [self.hue_low,
                                   self.hue_high,
                                   0,
                                   256],
                                  1)

        # Now convolute with circular disc
        disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        cv2.filter2D(dst, -1, disc, dst)  # do we need this convolution???

        # invert dst (because the backprojection chooses what we DON'T want)
        dst = 255 - dst

        return dst
        ## end HISTOGRAM BACKPROJECTION

    # this is what gets called when an image is recieved
    def image_callback(self, image):

        # use this to record start time for each frame
        #start_time = time.time()

        #### direct conversion from ROS Image to CV2 ####
        np_arr = np.fromstring(image.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        # assuming our region of interest is only in bottom half of image
        horizon = img.shape[0] / 2
        roi = img[horizon:, :]  # refer to how Python uses colons in lists

        # convert BGR image (by default using imshow) to GRAY
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # use green color in HSV to mask out all the uninteresting points
        
        backprojection_image =\
            self.get_backprojection_mask(roi, self.training_file_name)

        # threshold the backprojection to only grab the more probable ones
        ret, thresh = cv2.threshold(backprojection_image,
                                    self.backprojection_threshold,
                                    0,
                                    cv2.THRESH_TOZERO)

        # bitwise AND the remaining backprojection pixels with the original
        # gray image (we will only use gray so far so we don't need to use BGR
        # or HSV. If we did, then we could've merged thresh into a 3-channel
        # image then AND'ed with our original BGR or HSV)
        after_backprojection = cv2.bitwise_and(gray_roi, thresh)

        # perform gaussian blur on grayscale image
        blur = cv2.GaussianBlur(after_backprojection,
                                (self.blur_size,
                                 self.blur_size),
                                0)
        # perform median blur on grayscale image
        # blur = cv2.medianBlur(roi, self.blur_size)
        # blur = cv2.bilateralFilter(roi, self.blur_size,150,150)

        # global threshold (to zero out below threshold and leave other stuff
        # as is), using normalized brightness
        # first returned object (retval) is ignored

        # find (normalized to 1) mean of image brightness
        normalized_brightness = cv2.mean(gray_roi)[0] / 255
        retval, global_thresh = cv2.threshold(blur,
                                              self.global_threshold
                                              * normalized_brightness
                                              * self.global_threshold_factor,
                                              0,
                                              cv2.THRESH_TOZERO)

        # equalize histogram (globally)
        equ = cv2.equalizeHist(global_thresh)
        # TODO fix up histogram (maybe too many 0-value pixels in the histogram
        # that skew it?)
        
        ## CLAHE not available before OpenCV 3.0
        # perform CLAHE (adaptive histogram equalization) to fix contrast
        #clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        #equ = clahe.apply(roi)

        #testing Laplacian filter (not useful)
        # laplacian = cv2.Laplacian(equ,cv2.CV_64F)
        # cv2.imshow('laplacian', laplacian)

        # perform adaptive threshold
        # thresh = cv2.adaptiveThreshold(equ,
        #                                255,
        #                                cv2.ADAPTIVE_THRESH_MEAN_C,
        #                                cv2.THRESH_BINARY,
        #                                self.adaptive_threshold_block_size,
        #                                self.adaptive_threshold_C - 30)
        final_image = equ
        ## DEBUG: prints out side by side for comparison (analyzing effect of
        ## histogram equalization)
        # res = np.hstack((roi,equ)) #stacking images side-by-side

        ## skeletonize image
        # count = 0
        # size = np.size(thresh)
        # skel = np.zeros(thresh.shape,np.uint8)
        # element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
        # done = False
        # # iteratively erode, dilate, subtract, then OR the image until it's 1
        # # pixel thick
        # while(not done and count < 50 + self.max_erode_iterations):
        #     eroded = cv2.erode(thresh,element)
        #     temp = cv2.dilate(eroded,element)
        #     temp = cv2.subtract(thresh,temp)
        #     skel = cv2.bitwise_or(skel,temp)
        #     thresh = eroded.copy()
     
        #     zeros = size - cv2.countNonZero(thresh)
        #     if zeros==size:
        #         done = True

        #     count = count + 1

        # perform canny edge detection on blurred image
        # canny_image = cv2.Canny(equ,
        #                         self.canny_threshold,
        #                         self.canny_threshold * 2)
        # find contours from canny image
        # contours, hierarchy = cv2.findContours(equ,
        #                                        cv2.RETR_TREE,
        #                                        cv2.CHAIN_APPROX_SIMPLE)
        
        # draws contours on canny image
        # cv2.drawContours(equ, contours, -1, (255,255,0), 3)

        # skel = cv2.cvtColor(skel, cv2.COLOR_BGR2GRAY)
        # final_image = skel
        
        # print final_image.dtype
        # print final_image.shape
        # print final_image

        # use this to record end time
        # end_time = time.time()
        # print "time elapsed: ", (end_time - start_time)
        
        #### Create CompressedImage to publish ####
        final_image_message = CompressedImage()
        final_image_message.header.stamp = rospy.Time.now()
        final_image_message.format = "jpeg"
        final_image_message.data = np.array(cv2.imencode(
                                            '.jpg',
                                            final_image)[1]).tostring()

        # publishes image message with line pixels in it
        self.line_image_pub.publish(final_image_message)

    ## end image_callback()

    def reconfigure_callback(self, config, level):

        # TODO check if the keys exist in the config dictionary or else error
        # TODO also check if invalid values

        self.global_threshold = config['global_threshold']
        self.global_threshold_factor = config['global_threshold_factor']
        self.adaptive_threshold_block_size =\
            config['adaptive_threshold_block_size']
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
        self.backprojection_threshold = config['backprojection_threshold']
        self.training_file_name = config['training_file_name']

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


def main(args):
    # TODO make this file into a class
    # create a line_detection object
    ld = line_detection()

    # start the line_detector node and start listening
    rospy.init_node('generic_linedetection')
    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, ld.reconfigure_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
