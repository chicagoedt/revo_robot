#!/home/revo/anaconda2/envs/ROS+TF/bin/python

from keras.models import Sequential, load_model
from keras.utils import plot_model
from keras.layers import Conv2D, MaxPooling2D, Dropout, UpSampling2D
import sys, time
import roslib, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tensorflow as tf
import cv2

class lane_finder:

    def __init__(self):
        self.model = load_model(sys.argv[1])
        self.graph = tf.get_default_graph()
        self.image_pub = rospy.Publisher("/stereo_camera/left/lanes",Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/stereo_camera/left/image_color",Image,self.callback, queue_size=1, buff_size=100000000)

    def callback(self,data):
        start = time.clock()
        try:
            raw_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        model_input = np.array([raw_img]).astype(np.float32) / 255
        with self.graph.as_default():
            model_output = self.model.predict(model_input)[0] * 255
            lane_lines = model_output.astype(np.uint8)
            lane_lines_3 = cv2.cvtColor(lane_lines, cv2.COLOR_GRAY2RGB)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(lane_lines_3, "rgb8"))
        except CvBridgeError as e:
            print(e)
        end = time.clock()
        print("Latency: " + str((end - start) * 1000) + " milliseconds.")

def main(args):
    rospy.init_node('lane_finder', anonymous=True)
    lf = lane_finder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down.")

if __name__ == '__main__':
    main(sys.argv)
