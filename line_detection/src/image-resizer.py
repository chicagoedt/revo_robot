#!/usr/bin/env python
import sys
import numpy as np
import cv2
import rospy
import sensor_msgs
from sensor_msgs.msg import CompressedImage
from dynamic_reconfigure.server import Server
from line_detection.cfg import ImageResizerConfig

###############################################################################
## Chicago Engineering Design Team
## Image Resizer using Python OpenCV for autonomous robot Scipio (IGVC)
## @author Basheer Subei
## @email basheersubei@gmail.com
#######################################################
## This ROS node listens to a compressed image topic, resizes it (using
## dynamic reconfigure), and publishes it again.
###############################################################################

image_width = 640
image_height = 480


class image_resizer:

    def __init__(self):

        # initialize ROS stuff

        # set publisher and subscriber
        self.resized_image_pub = rospy.Publisher('resized_image/compressed',
                                                 sensor_msgs.msg.CompressedImage)

        self.image_sub = rospy.Subscriber("/camera/image_raw/compressed",
                                          CompressedImage, self.image_callback,
                                          queue_size=1)
        # self.image_sub = rospy.Subscriber("/camera/image_raw", Image,
        #                                    self.image_callback, queue_size=1)

    def image_callback(self, image):

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(image.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        #TODO throw these resolution resizing things into dynamic_reconfigure
        #resize the large image (if it is large) to a 640x480 image
        # print img.shape
        if img.shape[0] > image_height or img.shape[1] > image_width:
            # print "size is " + str(img.shape[1]) + "x" + str(img.shape[0])
            # print "resized to " + str(image_width) + "x" + str(image_height)
            img = cv2.resize(img, (image_width, image_height), 0, 0, 0)
        # print img.shape

        #### Create CompressedImage to publish ####
        final_image_message = CompressedImage()
        final_image_message.header.stamp = rospy.Time.now()
        final_image_message.format = "jpeg"
        final_image_message.data = np.array(cv2.imencode('.jpg',
                                            img)[1]).tostring()

        self.resized_image_pub.publish(final_image_message)

    ## end image_callback()

    def reconfigure_callback(self, config, level):

        # TODO fix this ugly hack by refactoring entire file into a class
        global image_width
        global image_height

        # TODO check if the keys exist in the config dictionary or else error

        image_width = config['image_width']
        image_height = config['image_height']
        return config


def main(args):
    # TODO make this file into a class
    # create a line_detection object
    resizer = image_resizer()

    # start the line_detector node and start listening
    rospy.init_node('image_resizer')
    # starts dynamic_reconfigure server
    srv = Server(ImageResizerConfig, resizer.reconfigure_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
