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
import lane_detection
###############################################################################
## Chicago Engineering Design Team
## Dilate filter using Python OpenCV for autonomous robot Scipio
##    (IGVC competition).
## @author Basheer Subei
## @email basheersubei@gmail.com


class dilate(lane_detection):

    image_height = 0
    image_width = 0

    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000

    dilate_size = 0
    dilate_iterations = 0

    # this is what gets called when an image is recieved
    def image_callback(self, image):

        # use this to record start time for each frame
        #start_time = time.time()
        

        if(self.use_mono and not self.use_compressed_format and image.encoding != 'mono8'):
            rospy.logerr("image is not mono8! Aborting!")
            return
        
        if self.use_compressed_format:
            #### direct conversion from ROS CompressedImage to CV2 ####
            np_arr = np.fromstring(image.data, np.uint8)
            if self.use_mono:
                img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_GRAYSCALE)
            else:
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

        # if mono, don't take 3rd dimension since there's only one channel
        if self.use_mono:
            roi = img[
            self.roi_top_left_y:self.roi_top_left_y + self.roi_height,
            self.roi_top_left_x:self.roi_top_left_x + self.roi_width,
            ]
        else:
            roi = img[
                self.roi_top_left_y:self.roi_top_left_y + self.roi_height,
                self.roi_top_left_x:self.roi_top_left_x + self.roi_width,
                :
            ]

        # in case roi settings aren't correct, just use the entire image
        if roi.size <= 0:
            rospy.logerr("Incorrect roi settings! Will use the entire image instead!")
            roi = img

        # use entire image as roi (don't cut any parts out)
        # roi = img

        final_image = roi

        # dilate each pixel using kernel with dilate_size
        if self.dilate_size > 0 and self.dilate_iterations > 0:
            kernel = np.ones((self.dilate_size, self.dilate_size), np.uint8)
            final_image = cv2.dilate(roi, kernel, iterations = self.dilate_iterations)
        else:
            rospy.logwarn("dilate parameters invalid! Won't perform dilate!")

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

    

def main(args):

    node_name = "dilate_lanedetection"
    namespace = rospy.get_namespace()
    if namespace == "/":
        namespace = ""

    # create a dilate object
    d = dilate()

    # start the line_detector node and start listening
    rospy.init_node("dilate_lanedetection", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, d.reconfigure_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
