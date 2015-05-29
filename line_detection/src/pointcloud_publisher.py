#!/usr/bin/env python
import sys
import numpy as np
import cv2
import rospy
# from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
# from line_detection.cfg import LineDetectionConfig
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
import rospkg
from itertools import izip
import std_msgs.msg


###############################################################################
# Chicago Engineering Design Team
# Line PointCloud publisher node
#
# Publishes a pointcloud of every non-zero pixel in the input image
#
# @author Basheer Subei
# @email basheersubei@gmail.com


class PointcloudPublisher(LaneDetection):

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

        self.publisher_cloud_topic = rospy.get_param(
            namespace + node_name + "/publisher_cloud_topic",
            "/line_pointcloud"
        )
        # removes trailing slash in namespace
        if (namespace.endswith("/")):
            namespace = namespace[:-1]
        # publisher for line pointcloud
        self.line_cloud_pub = rospy.Publisher(
            namespace + "/" + node_name + self.publisher_cloud_topic,
            PointCloud,
            queue_size=10
        )

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)
        # print cv2_image.shape
        roi = LaneDetection.get_roi(self, cv2_image)

        non_zeros = roi.nonzero()
        self.number_of_points = len(non_zeros[0])
        self.line_pointcloud = PointCloud()
        self.line_pointcloud.header = std_msgs.msg.Header()
        self.line_pointcloud.header.stamp = rospy.Time.now()
        self.line_pointcloud.header.frame_id = "base_footprint"
        # create an empty list of correct size
        self.line_pointcloud.points = [None] * self.number_of_points
        count = 0
        for (row, column) in izip(non_zeros[0], non_zeros[1]):
            # print "row: " + str(row) + ", column: " + str(column)
            # print("intersection with ground: " + str(self.intersection_array[column][row]) + ")")
            point = self.intersection_array[row][column]
            self.line_pointcloud.points[count] = Point(point[0], point[1], 0)
            count += 1
        # publishes pointcloud message
        self.line_cloud_pub.publish(self.line_pointcloud)
    # end image_callback()


def main(args):
    node_name = "pointcloud_publisher"
    namespace = rospy.get_namespace()

    # create a BlobDetection object
    pp = PointcloudPublisher(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("pointcloud_publisher", anonymous=True)

    pp.intersection_array = np.load(
        pp.package_path + "/misc/training_images/pixel_coordinates.npy"
    )
    # starts dynamic_reconfigure server
    # srv = Server(LineDetectionConfig, c.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
