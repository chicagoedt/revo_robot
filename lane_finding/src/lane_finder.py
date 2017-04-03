from keras.models import Sequential, load_model
from keras.utils import plot_model
from keras.layers import Conv2D, MaxPooling2D, Dropout, UpSampling2D
import sys
import roslib, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class lane_finder:

    def __init__(self):
        self.image_pub = rospy.Publisher("/stereo_camera/left/lanes",Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/stereo_camera/left/image_color",Image,self.callback)

        self.model = load_model(sys.argv[1])

    def callback(self,data):
        try:
            raw_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        model_input = np.array([raw_img]).astype(np.float32) / 255
        model_output = self.model.predict(model_input)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(model_output * 255, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node('lane_finder', anonymous=True)
    lf = lane_finder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down.")

if __name == '__main__':
    main(sys.argv)
