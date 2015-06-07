#!/usr/bin/env python
import sys
import rospy
from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
from line_detection.cfg import LineDetectionConfig
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import CompressedImage

###############################################################################
# Chicago Engineering Design Team
# Max Mask filter that applies a mask on an input image
#
# @author Basheer Subei
# @email basheersubei@gmail.com


class MaxMask(LaneDetection):

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

        # we will re-register this under message_filters instead
        self.image_sub.unregister()
        self.image_sub = None

        self.subscriber_mask_topic = rospy.get_param(
            namespace + node_name + "/subscriber_mask_topic",
            "/mask_image/compressed"
        )

        self.image_sub = message_filters.Subscriber(
            self.subscriber_image_topic,
            CompressedImage,
            queue_size=10,
            buff_size=self.buffer_size
        )
        self.mask_sub = message_filters.Subscriber(
            self.subscriber_mask_topic,
            CompressedImage,
            queue_size=10,
            buff_size=self.buffer_size
        )

        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.mask_sub], 10, 1)
        ts.registerCallback(self.image_callback)

    # this is what gets called when an image is received
    def image_callback(self, ros_image, mask_message):

        first_image = self.ros_to_cv2_image(ros_image)

        # convert mask_message to cv2
        np_arr = np.fromstring(mask_message.data, np.uint8)
        mask_image = cv2.imdecode(np_arr, -1)

        mask_image = self.convert_to_mono(mask_image)

        # need to make mask 3 channels
        mask = np.dstack((mask_image, mask_image, mask_image))

        final_image = np.maximum(first_image, mask)

        final_image_message = self.cv2_to_ros_message(final_image)
        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "max_mask"
    namespace = rospy.get_namespace()

    # create a hough object
    m = MaxMask(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("max_mask", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, m.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
