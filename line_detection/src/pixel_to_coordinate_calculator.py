#!/usr/bin/env python
import tf2_ros
import tf2_geometry_msgs
import rospy
from sensor_msgs.msg import CameraInfo
import image_geometry
import itertools
from camera_info_manager import *
import rospkg
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud
import std_msgs.msg
import numpy as np

###############################################################################
# Chicago Engineering Design Team
# 
#
# 
#
# @author Basheer Subei
# @email basheersubei@gmail.com

DEBUG_POINTCLOUD = True


class PixelToCoordinateCalculator:

    package_path = rospkg.RosPack().get_path('line_detection')

    # set up camera info manager instance
    camera_info_url = rospy.get_param(
        'pixel_to_coordinate_calculator/camera_info_url',
        "file://" + package_path + "/misc/calibration_data/aptina_960.yaml"
    )
    camera_name = rospy.get_param(
        'pixel_to_coordinate_calculator/camera_name',
        "camera"
    )

    cam_info_manager = CameraInfoManager(cname=camera_name, url=camera_info_url)
    # get camera info
    cam_info_manager.loadCameraInfo()
    camera_info = cam_info_manager.getCameraInfo()

    def __init__(self):
        rospy.init_node('pixel_to_coordinate_calculator')
        # tf2 listener and buffer
        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cloud_pub = rospy.Publisher(
            'pointcloud_debug',
            PointCloud,
            queue_size=10
        )

    def initialize_camera_model(self):
        # create a pinhole model from camera info
        self.cam_frame = self.camera_info.header.frame_id
        self.cam_model = image_geometry.PinholeCameraModel()
        self.cam_model.fromCameraInfo(self.camera_info)

        # top-left x coordinate of ROI rectangle
        self.roi_x = rospy.get_param(
            'pixel_to_coordinate_calculator/roi_x',
            0
        )
        # top-left y coordinate of ROI rectangle
        self.roi_y = rospy.get_param(
            'pixel_to_coordinate_calculator/roi_y',
            self.camera_info.height / 2
        )
        assert(self.roi_x >= 0)
        assert(self.roi_x < self.camera_info.width)
        assert(self.roi_y >= 0)
        assert(self.roi_y < self.camera_info.height)

        # width of ROI rectangle
        self.roi_width = rospy.get_param(
            'pixel_to_coordinate_calculator/roi_width',
            self.camera_info.width - self.roi_x
        )
        # height of ROI rectangle
        self.roi_height = rospy.get_param(
            'pixel_to_coordinate_calculator/roi_height',
            self.camera_info.height - self.roi_y
        )
        # self.image_height = self.camera_info.height
        # self.image_width = self.camera_info.width

        # array that holds (x,y) point for each pixel
        self.intersection_array = np.zeros((self.roi_height, self.roi_width, 2), dtype=float)

    def get_transform(self):
        self.trans = self.tf_buffer.lookup_transform("base_footprint", "camera_optical", rospy.Time())

    def get_3d_rays_from_camera_model(self):

        # go through every pixel in the image, and get
        # the 3d ray corresponding to it

        # get cartesian product of all pixel indices
        all_pixels = itertools.product(
            xrange(self.roi_x, self.roi_x + self.roi_width),
            xrange(self.roi_y, self.roi_y + self.roi_height)
        )

        # self.number_of_pixels = len(list(all_pixels))  # for some reason this line ruins the iterator
        self.number_of_pixels = self.roi_width * self.roi_height

        start_time = rospy.get_rostime().secs

        # for debugging, create a Pointcloud and fill it with these 3d rays (just the point values)
        self.debug_pointcloud = PointCloud()
        self.debug_pointcloud.header = std_msgs.msg.Header()
        self.debug_pointcloud.header.stamp = rospy.Time.now()
        self.debug_pointcloud.header.frame_id = "camera_optical"
        # create an empty list of correct size
        self.debug_pointcloud.points = [None] * self.number_of_pixels

        rospy.loginfo("Calculating %d rays for each pixel. This might take a while...", self.number_of_pixels)
        # loop over them and create a 3d ray for each pixel
        count = 0  # using a counter here because I have to loop over all elements in all_pixels iterator
        for pixel in all_pixels:
            if rospy.is_shutdown():  # loop takes a while, use this to respond to shutdown signal
                return
            (r_x, r_y, r_z) = self.cam_model.projectPixelTo3dRay(pixel)

            # create a point stamped in camera_optical frame
            point_stamped = PointStamped()
            point_stamped.point = Point(r_x, r_y, r_z)
            point_stamped.header = std_msgs.msg.Header()
            point_stamped.header.stamp = rospy.Time.now()
            point_stamped.header.frame_id = "camera_optical"

            # transform the point to base_footprint
            while True:
                try:
                    transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, self.trans)
                    rospy.logdebug("transformed_point is (%f, %f, %f)", transformed_point.point.x, transformed_point.point.y, transformed_point.point.z)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logwarn("tf2 exception raised! retrying!")
                    # rate.sleep()
                    continue  # stay in infinite while loop until point is successfully transformed
                break  # break out of infinite while loop because this point was successfully transformed

            # self.debug_pointcloud.points[count] = point_stamped.point

            # now the rays are from origin of camera_optical frame to this transformed_point.
            # so the intersection
            inter = isect_line_plane_v3(self.trans.transform.translation, transformed_point.point, Point(0, 0, 0), Point(0, 0, 1))
            # rospy.logdebug("intersection of 3d ray with ground plane is (%f, %f, %f)", inter.x, inter.y, inter.z)

            # TEMPORARY TESTING
            self.debug_pointcloud.header.frame_id = "base_footprint"
            self.debug_pointcloud.points[count] = inter

            self.intersection_array[pixel[1]-self.roi_y, pixel[0]-self.roi_x, 0] = inter.x
            self.intersection_array[pixel[1]-self.roi_y, pixel[0]-self.roi_x, 1] = inter.y

            if(count % 100000 == 0):
                rospy.loginfo("Done with %d rays...", count)
            count += 1

        end_time = rospy.get_rostime().secs
        rospy.loginfo("finished getting 3d rays in %i seconds!", (end_time - start_time))

    def write_intersection_array_to_file(self):
        np.save(
            self.package_path + "/misc/training_images/pixel_coordinates.npy",
            self.intersection_array
        )

# end class


# p0 is origin of camera_optical frame (the transform.translation)
# p1 is the current point for this pixel (transformed_point.point)
# p_co is just (0, 0, 0) origin of base_footprint frame
# p_no is the vector pointing up (0, 0, 1) normal to ground plane z=0
# intersection function adapted from http://stackoverflow.com/a/18543221/341505
def isect_line_plane_v3(p0, p1, p_co, p_no, epsilon=1e-6):
    """
    p0, p1: define the line
    p_co, p_no: define the plane:
        p_co is a point on the plane (plane coordinate).
        p_no is a normal vector defining the plane direction; does not need to be normalized.

    return a Vector or None (when the intersection can't be found).
    """

    u = sub_v3v3(p1, p0)
    dot = dot_v3v3(p_no, u)

    if abs(dot) > epsilon:
        # the factor of the point between p0 -> p1 (0 - 1)
        # if 'fac' is between (0 - 1) the point intersects with the segment.
        # otherwise:
        #  < 0.0: behind p0.
        #  > 1.0: infront of p1.
        w = sub_v3v3(p0, p_co)
        fac = -dot_v3v3(p_no, w) / dot
        mul_v3_fl(u, fac)
        return add_v3v3(p0, u)
    else:
        # The segment is parallel to plane
        return None


# generic math functions
def add_v3v3(a, b):
    return Point(a.x + b.x, a.y + b.y, a.z + b.z)


def sub_v3v3(a, b):
    return Point(a.x - b.x, a.y - b.y, a.z - b.z)


def dot_v3v3(a, b):
    return (a.x * b.x + a.y * b.y + a.z * b.z)


def len_squared_v3(a):
    return dot_v3v3(a, a)


def mul_v3_fl(a, f):
    a.x *= f
    a.y *= f
    a.z *= f


if __name__ == '__main__':

    p = PixelToCoordinateCalculator()

    rospy.sleep(1.0)  # fix for lookup_transform() being called too soon
    p.initialize_camera_model()
    p.get_transform()
    p.get_3d_rays_from_camera_model()
    p.write_intersection_array_to_file()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if(DEBUG_POINTCLOUD):
            # don't forget to update time stamp on message
            p.debug_pointcloud.header.stamp = rospy.Time.now()
            # now publish the debug pointcloud
            p.cloud_pub.publish(p.debug_pointcloud)
        rate.sleep()

    rospy.spin()
