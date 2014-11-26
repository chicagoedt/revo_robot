#!/usr/bin/env python
import sys
import numpy as np
import cv2
from cv2 import *
from matplotlib import pyplot as plt
import time
import rospy
import std_msgs
from sensor_msgs import point_cloud2
import sensor_msgs
import math
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
import geometry_msgs
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
import rospkg
from dynamic_reconfigure.server import Server
from line_detection.cfg import LineDetectionConfig

###################################################################################################
## Chicago Engineering Design Team
## Line Detection using Python OpenCV for autonomous robot Scipio (IGVC competition)
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
## 2. mask out green areas using hsv filter (uses absolute values, not relative yet)
## 3. gaussian blur (removes high-frequency noise)
## 4. global threshold (depends on brightness or something)
## 5. equalize histogram (not adaptively)
## 6. adaptive threshold
## 7. display image
##
## TODO conform to PEP8 style standard and consider using Flake8 to check it. 
###########################################################################################################################

global_threshold = 160
global_threshold_factor = 2
adaptive_threshold_block_size = 191
adaptive_threshold_C = 30
blur_size = 49
canny_threshold = 100
max_erode_iterations = 100
bandpass_low_cutoff = 1
bandpass_high_cutoff = 30

#hsv threshold variables 
hue_low = 20
hue_high = 50

saturation_low = 0
saturation_high = 255

value_low = 0
value_high = 255

backprojection_threshold = 50

training_file_name = 'training_for_backprojection_1.png'


class line_detection:

    def __init__(self):

        # initialize ROS stuff        

        # set publisher and subscriber
        ## TODO find out what name the topic should have
        self.line_pub = rospy.Publisher('line_data', sensor_msgs.msg.PointCloud2)
        self.line_image_pub = rospy.Publisher('line_image/compressed', sensor_msgs.msg.CompressedImage)
        # self.warped_line_image_pub = rospy.Publisher('warped_line_image', sensor_msgs.msg.Image)
        # self.ray_pub = rospy.Publisher('ray', geometry_msgs.msg.Vector3Stamped)


        self.bridge = CvBridge()
        # self.img_geo = image_geometry.PinholeCameraModel()
        self.image_sub = rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, self.image_callback, queue_size=1)
        # self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback, queue_size=1)
    
        # self.camera_info_sub = rospy.Subscriber("/camera/camera_info", CameraInfo, self.camera_info_callback, queue_size=1 )




    def nothing(x):
        pass

    def camera_info_callback(self, camera_info):
        # print "camera_info received!"
        self.img_geo.fromCameraInfo(camera_info)


    # # converts pixels from camera space to 3-d space
    # ## TODO take all these constants from ROS parameters
    # def publish_line_data(self, camera_image, horizon = 0) :


    #     # get indices of nonzero elements (pixels that are lines)
    #     pixels = np.transpose(camera_image.nonzero())
    #     # print len(pixels)
    #     # pixels = camera_image.nonzero()
    #     # print len(pixels)
    #     #pixels contains the line obstacle pixels that need to be converted to PCL



    #     # use image_geometry to get vectors for each pixel
    #     # print pixels[0][0]
    #     # print pixels[1][0]
    #     # ray = self.img_geo.projectPixelTo3dRay( (pixels[0][0], pixels[1][0]) )
    #     # ray = self.img_geo.projectPixelTo3dRay( (960, 540) )
        
    #     # (pixelx, pixely) = self.img_geo.project3dToPixel()

    #     # ray_vector = geometry_msgs.msg.Vector3Stamped()
    #     # ray_vector.header.stamp = rospy.Time.now()
    #     # ray_vector.header.frame_id = "base_camera"
        
    #     # ray_vector.vector.x = ray[0]
    #     # ray_vector.vector.y = ray[1]
    #     # ray_vector.vector.z = ray[2]

    #     # self.ray_pub.publish(ray_vector)
    #     # print ray

        
    #     #height = height of the camera from the ground in meters
    #     # height = 0.1194
    #     height = 0.7 # 0.7 m
    #     orien = 65 # 90 - 25
    #     #orien = orientation of the camera (pitch angle from vertical)
        
    #     ##TODO find dimensions dynamically

    #     image_width = camera_image.shape[1]
    #     image_height = camera_image.shape[0]

    #     # print "image height"
    #     # print image_height
    #     # print "image width"
    #     # print image_width

    #     horAngl = 120.0 #field of view
    #     verAngl = 120.0 * 9.0 / 16.0 #field of view

    #     #reference horizon that will be used in calculations.
    #     refHor = image_height - horizon

    #     #difference in angle that each pixel corresponds to
    #     dAV = verAngl / image_height
    #     dAV /= 2
    #     #create the cloud
    #     # cloud = sensor_msgs.msg.PointCloud(rospy.Time.now(), pixels, 0)


    #     # cloud = sensor_msgs.msg.PointCloud2()
    #     #create the header
    #     # cloud.header.stamp = rospy.Time.now()
    #     # cloud.header.frame_id = "sensor_frame"

    #     # cloud.channels.resize(1)
    #     #sets the size of the cloud. This was need in c++ so i think it will be needed in python
    #     # cloud.resize(len(pixels))
    #     # cloud.set_points_size(len(pixels))

    #     '''
    #     data for 70cm height camera with 25 degree incline (below horizon) 1080p:
    #     all pixels are measure from top left corner
    #     24 inches   -->    1017
    #     32 inches   -->    877
    #     40 inches   -->    786
    #     48 inches   -->    720
    #     56 inches   -->    674
    #     64 inches   -->    635
    #     72 inches   -->    602
    #     80 inches   -->    578
    #     88 inches   -->    556
    #     96 inches   -->    540
    #     104 inches  -->    526
    #     112 inches  -->    514
    #     120 inches  -->    502
        
    #     x is (1017, 877, 786, 720, 674, 635, 602, 578, 556, 540, 526, 514, 502) pixels
    #     y is (0.6096, 0.8128, 1.016, 1.219, 1.422, 1.626, 1.829, 2.032, 2.235, 2.438, 2.642, 2.845, 3.048) meters

    #     polyfit returns: [     ]   
    #     '''

    #     x = np.array([1017, 877, 786, 720, 674, 635, 602, 578, 556, 540, 526, 514, 502])
    #     x = x - image_height # flip the pixels
    #     y = (0.6096, 0.8128, 1.016, 1.219, 1.422, 1.626, 1.829, 2.032, 2.235, 2.438, 2.642, 2.845, 3.048)
    #     # fits a 4th degree polynomial onto x and y data (pixels vs distance)
    #     poly = np.polyfit(x, y, 4)
    #     rospy.loginfo("x is: %s", x)
    #     rospy.loginfo("polyfit returns: %s", poly)

    #     cloud_points = np.array([[0 for i in range(0,3)] for j in range(len(pixels))], dtype = float)
    #     # print "length of cloud points is ", len(cloud.points)
    #     # print pixels[0][0]
    #     #if there is no horizon argument passed into the function it will default to this method
        
    #     # old polynomial fit for 0.11m height and 720p
    #     #poly = np.array([0.000000055899,-4.7924E-5,0.015794,-2.7904,321.04],dtype = float)
    #     if horizon == 0 :
    #         for i in range(0,len(pixels)) :
    #             '''
    #             temp = height * math.tan(math.radians(orien - pixels[i][1] * dAV))
    #             cloud_points[i][0] = (float(pixels[i][0] - (image_width / 2)) / (image_width / 2)) * temp * math.tan((horAngl)/2 * math.pi / 180.0)
    #             cloud_points[i][1] = -temp
    #             cloud_points[i][2] = 0.0
    #             '''
    #             #make sure not to flip pixels[i][x]
    #             horizontal_ratio = float((pixels[i][1] - (image_width/2.0))/(image_width/2.0))
    #             # use 4-th degree polynomial curve (that we fit onto verticalPixel-yDistance data we gathered), this curve was done at 0.7m height.
    #             cloud_points[i][0] = poly[0] * (pixels[i][0]**4) + poly[1] * (pixels[i][0]**3) + poly[2] * (pixels[i][0]**2) + poly[3] * (pixels[i][0]) + poly[4]
    #             # cloud_points[i][0] /= 100   #convert cm to m
    #             cloud_points[i][1] = -horizontal_ratio * cloud_points[i][0] * math.tan(math.radians(horAngl / 2.0))
    #             cloud_points[i][2] = 0.0






    #     else :
    #         #reference horizon that will be used in calculations
    #         refHor = image_height - horizon

    #         #fills the cloud with the line data
    #         for i in range(0,len(pixels)) :
    #             temp = height * math.tan( (90 - (refHor - abs(pixels[i][1] - image_height)) * dAV) * math.pi / 180.0)
    #             cloud_points[i][0] = (float(pixels[i][0] - (image_width / 2)) / (image_width / 2)) * temp * math.tan((horAngl)/2 * math.pi / 180.0)
    #             cloud_points[i][1] = -temp
    #             cloud_points[i][2] = 0.0


    #     # cloud_header = std_msgs.header.Header()

    #     # print cloud_points

    #     cloud = sensor_msgs.msg.PointCloud2()
    #     cloud = sensor_msgs.point_cloud2.create_cloud_xyz32(cloud.header, cloud_points)

    #     cloud.header.frame_id = "base_camera"
    #     cloud.header.stamp = rospy.Time.now()

    #     # self.line_pub.publish(cloud)

    #     return cloud
        

    # given a line_image, it returns a pointcloud of the white pixels in the line_image (applies perspectiveTransform to
    # transform from pixel space to camera space)
    def get_pointcloud2_from_line_image(self, camera_image) :

    #     pixels = np.transpose(camera_image.nonzero())

        image_width = camera_image.shape[1]
        image_height = camera_image.shape[0]

        horAngl = 120.0 #field of view
        verAngl = 120.0 * 9.0 / 16.0 #field of view


        ''' 

        we will now perform a perspective transform from pixel space to camera space (coordinates)
        this is done by picking for corner points from the image, and measuring the x,y points for the pixels and world coordinates.



        '''

        perspective_image_height = 484 # 720 - 236
        perspective_image_width = 1249 # 1280 - 31
        top_right_pixel = [1246, 241]
        top_left_pixel = [31, 236]
        bottom_right_pixel = [1280, 720]
        bottom_left_pixel = [0, 720]

        top_right_coordinate = [-9.4488, 7.1374] 
        top_left_coordinate = [9.4488, 7.1374]
        bottom_right_coordinate = [1.1684, 0]
        bottom_left_coordinate = [-1.1684, 0]
        


        pixel_corners = np.float32([top_right_pixel, top_left_pixel, bottom_right_pixel, bottom_left_pixel])
        camera_corners = np.float32([top_right_coordinate, top_left_coordinate, bottom_right_coordinate, bottom_left_coordinate])

        transform_matrix = cv2.getPerspectiveTransform(pixel_corners, camera_corners)

        warped_image = cv2.warpPerspective(camera_image, transform_matrix,(perspective_image_height, perspective_image_width))



    ## publish the warped_image as a ROS image
        # we have to resize the warped_image to include the number of channels (because the function cv2_to_imgmsg expects a third dimension shape[2])
        warped_image.resize((warped_image.shape[0], warped_image.shape[1], 1))


        try:
            warped_image_message = self.bridge.cv2_to_imgmsg(warped_image, "mono8")
            # final_image_message = self.bridge.cv2_to_imgmsg(final_image, "bgr8" )
        except CvBridgeError, e:
            print e

        # resize it again and remove third dimension
        warped_image.resize(warped_image.shape[0], warped_image.shape[1])

        # self.warped_line_image_pub.publish(warped_image_message)



        # now find the white pixels in the warped image and place them  as pointcloud points


        # create laser_scan messages


        # cloud = sensor_msgs.msg.PointCloud2()
        # cloud = sensor_msgs.point_cloud2.create_cloud_xyz32(cloud.header, cloud_points)

        # cloud.header.frame_id = "base_camera"
        # cloud.header.stamp = rospy.Time.now()



        # return cloud
 

    def image_callback(self, image):

        # print "oh, got something on /camera/image_rect_color"
        start_time = time.time()

        # first, we need to convert image from sensor_msgs/Image to numpy (or cv2). For this, we use cv_bridge
        # try:
        #     img = self.bridge.imgmsg_to_cv2(image, "bgr8")
        #     # img = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        # except CvBridgeError, e:
        #     print e

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(image.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        
        # cv2.imshow('img', img)
        # cv2.waitKey(1)
        # our region of interest is only in bottom half of image
        horizon = img.shape[0]/2
        roi = img[horizon:, :]

        # gray_roi = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY) # convert BGR image (by default using imshow) to GRAY
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) # convert BGR image (by default using imshow) to GRAY


    # note: GIMP color picker has ranges for HSV from 0 to 360 (H), 100 (S), and 100 (V)
    # need to convert those ranges into opencv HSV ranges 179 (H), 255 (S), and 255 (V).
    ## use green color in HSV to mask out all the uninteresting points
        # Convert BGR to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # make HSV histogram and look for regions with lots of green Hue. For those regions find their range of HSV so that they can be masked out
        # Grass Hue ranges from 20:50 , Saturation ranges from 50:125 , Value ranges from 0:150.
        # White lines Hue ranges from 120:150 , Saturation ranges from 0:50 , Value ranges from 150:255.
        # all above values are in opencv HSV ranges

        # note that Values depend on overall brightness (need to use adaptive method or dynamic one).

        rospack = rospkg.RosPack() #to find package path
        training_file_path = rospack.get_path('line_detection') + '/misc/training_images/' + training_file_name
        backprojection_training = cv2.imread(training_file_path)
        backprojection_training = cv2.cvtColor(backprojection_training, cv2.COLOR_BGR2HSV)
        
    # begin HISTOGRAM BACKPROJECTION
        # calculating object histogram
        roihist = cv2.calcHist([backprojection_training],[0, 1], None, [180, 256], [hue_low, hue_high, 0, 256] )

        # plt.hist(roihist.ravel(),256,[40,256]); plt.show() #roi histogram (starting from value 1, skipping zero values)

        # normalize histogram and apply backprojection
        cv2.normalize(roihist,roihist,1,255,cv2.NORM_MINMAX)

        # plt.hist(roihist.ravel(),256,[40,256]); plt.show() #roi histogram (starting from value 1, skipping zero values)

        dst = cv2.calcBackProject([hsv],[0,1],roihist,[hue_low,hue_high,0,256],1)

        # Now convolute with circular disc
        disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        cv2.filter2D(dst,-1,disc,dst) # do we need this convolution???

        # invert dst (because the backprojection chooses what we DON'T want)
        dst = 255 - dst 

        # threshold the backprojection stuff to only grab the more probable ones
        ret,thresh = cv2.threshold(dst,backprojection_threshold,0,cv2.THRESH_TOZERO)
        
        # AND the remaining backprojection pixels with the original gray image (we will only use gray so far so
        # we don't need to use BGR or HSV. If we did, then we could've merged thresh into a 3-channel image then AND'ed 
        # with our original BGR or HSV)
        after_backprojection = cv2.bitwise_and(gray_roi, thresh)
        
        # cv2.imshow('backprojection_matrix', thresh)
        # cv2.imshow('backprojection_result', after_backprojection)

    ## end HISTOGRAM BACKPROJECTION
    # TODO actually connect backprojection output to rest of filter
    # TODO consider revising bitwise AND to a weighting algorithm (because it's too exclusive, we might miss good points)

    ## begin hsv threshold 
        # define range of blue color in HSV
        # lower_grass = np.array([hue_low, saturation_low, value_low])
        # upper_grass = np.array([hue_high, saturation_high, value_high])

        # # Threshold the HSV image to get only blue colors
        # mask = cv2.inRange(hsv, lower_grass, upper_grass)

        # mask_inv = 255-mask #invert the mask (because the mask selected the parts we did NOT want)

        # # Bitwise-AND mask and original image
        # after_hsv = cv2.bitwise_and(gray_roi, gray_roi, mask=mask_inv)
    ## end hsv threshold


        #roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    ## abandoning bandpass filter because I couldn't figure out how to convert the float np array back to meaningful int arrays to use it for the mask
    ## and yet, opencv can display those float np arrays just fine... ???
    ## testing out bandpass filter thingie (from fourier transform page on opencvPython tut)
        # dft = cv2.dft(np.float32(roi),flags = cv2.DFT_COMPLEX_OUTPUT)
        # dft_shift = np.fft.fftshift(dft)

        # rows, cols = roi.shape
        # crow,ccol = rows/2 , cols/2

        # #TODO improve mask for bandpass filter
        # # create a mask (square with a smaller square hole in it)
        # mask = np.zeros((rows,cols,2),np.uint8)
        # mask[crow-bandpass_high_cutoff:crow+bandpass_high_cutoff, ccol-bandpass_high_cutoff:ccol+bandpass_high_cutoff] = 1
        # mask[crow-bandpass_low_cutoff:crow+bandpass_low_cutoff, ccol-bandpass_low_cutoff:ccol+bandpass_low_cutoff] = 0


        # # apply mask and inverse DFT
        # fshift = dft_shift*mask
        # f_ishift = np.fft.ifftshift(fshift)
        # img_back = cv2.idft(f_ishift)
        # img_back = cv2.magnitude(img_back[:,:,0],img_back[:,:,1])


        # cv2.imshow('afterBandpass', img_back)
        # # use bandpass filter as mask (only include pixels that are part of bandpass) multiplying img_back and roi
        # print img_back
        # print img_back.dtype

        # img_back = img_back.astype(int) * 256 # convert the bandpass mask from float32 to uint8
        # print img_back
        # print img_back.dtype

        # img_back = cv2.bitwise_not(img_back) # invert the image (to make the mask match the stuff within bandpass frequencies)
        

        # cv2.imshow('afterBandpass2', img_back)

        # masked_bandpass = cv2.bitwise_and(img_back, roi)

        # plot the histogram for analysis
        #plt.hist(gray_image.ravel(),256,[0,256]); plt.show() # original image histogram
        #plt.hist(roi.ravel(),256,[0,256]); plt.show() #roi histogram
        
        blur = cv2.GaussianBlur(after_backprojection, (blur_size, blur_size), 0) # perform gaussian blur on grayscale image
        # blur = cv2.medianBlur(roi, blur_size) # perform median blur on grayscale image
        # blur = cv2.bilateralFilter(roi,blur_size,150,150)

        # global threshold (to zero out below threshold and leave other stuff as is)
        # first returned object is ignored

        # find (normalized to 1) mean of image brightness
        normalized_brightness = cv2.mean(gray_roi)[0] / 255
        # print "normalized brightness: ", normalized_brightness
        retval, global_thresh = cv2.threshold(blur, global_threshold * normalized_brightness * global_threshold_factor, 0, cv2.THRESH_TOZERO)

        # equalize histogram (globally)
        equ = cv2.equalizeHist(global_thresh) 
        # TODO fix up histogram (maybe too many 0-value pixels in the histogram that skew it?)
        
        ## CLAHE not available before OpenCV 3.0
        # perform CLAHE (adaptive histogram equalization)
        #clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        #equ = clahe.apply(roi)

        #plt.hist(equ.ravel(),256,[1,256]); plt.show() #roi histogram (starting from value 1, skipping zero values)

        #testing Laplacian filter (not useful)
        # laplacian = cv2.Laplacian(equ,cv2.CV_64F)
        # cv2.imshow('laplacian', laplacian)

        # perform adaptive threshold
        # cv2.imshow('before adaptive threshold', equ)
        thresh = cv2.adaptiveThreshold(equ, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, adaptive_threshold_block_size, adaptive_threshold_C - 30)
        # cv2.imshow("after adaptive threshold", thresh)
        ## DEBUG: prints out side by side for comparison (analyzing effect of histogram equalization)
        # res = np.hstack((roi,equ)) #stacking images side-by-side

        ## skeletonize image
        count = 0
        size = np.size(thresh)
        skel = np.zeros(thresh.shape,np.uint8)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
        done = False
        # iteratively erode, dilate, subtract, then OR the image until it's 1 pixel thick
        while(not done and count < 50 + max_erode_iterations):
            eroded = cv2.erode(thresh,element)
            temp = cv2.dilate(eroded,element)
            temp = cv2.subtract(thresh,temp)
            skel = cv2.bitwise_or(skel,temp)
            thresh = eroded.copy()
     
            zeros = size - cv2.countNonZero(thresh)
            if zeros==size:
                done = True

            count = count + 1


        # canny_image = cv2.Canny(equ, canny_threshold, canny_threshold*2) # perform canny edge detection on blurred image
        # contours, hierarchy = cv2.findContours(equ, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # find contours from canny image
        
        # cv2.drawContours(equ, contours, -1, (255,255,0), 3) # draws contours on canny image


        end_time = time.time()

        # print "time elapsed: ", (end_time - start_time)    

        

        # skel = cv2.cvtColor(skel, cv2.COLOR_BGR2GRAY)
        final_image = skel
        
        # print final_image.dtype
        # print final_image.shape
        # print final_image

        # we have to resize the final_image to include the number of channels (because the function cv2_to_imgmsg expects a third dimension shape[2])
        # final_image.resize((final_image.shape[0], final_image.shape[1], 1))


        # print final_image.shape

        # final_image_message = image

        # try:
        #     final_image_message = self.bridge.cv2_to_imgmsg(final_image, "mono8")
        #     # final_image_message = self.bridge.cv2_to_imgmsg(final_image, "bgr8" )
        # except CvBridgeError, e:
        #     print e

        # resize it again and remove third dimension
        # final_image.resize(final_image.shape[0], final_image.shape[1])

        #### Create CompressedImage to publish ####
        final_image_message = CompressedImage()
        final_image_message.header.stamp = rospy.Time.now()
        final_image_message.format = "jpeg"
        final_image_message.data = np.array(cv2.imencode('.jpg', final_image)[1]).tostring()

        self.line_image_pub.publish(final_image_message)

        # # show images using gui
        # cv2.imshow('original', roi)
        # # cv2.imshow('gray_roi', gray_roi)
        # cv2.imshow('final image', final_image)

        ### TODO need to resize image to original 720p


        # line_pointcloud = self.get_pointcloud2_from_line_image(final_image)

        # self.line_pub.publish(line_pointcloud)

    ## end image_callback()

def reconfigure_callback(config, level):

    # TODO fix this ugly hack by refactoring entire file into a class
    global global_threshold
    global global_threshold_factor
    global adaptive_threshold_block_size
    global adaptive_threshold_C
    global blur_size
    global canny_threshold
    global max_erode_iterations
    global bandpass_low_cutoff
    global bandpass_high_cutoff
    global hue_low
    global hue_high
    global saturation_low
    global saturation_high
    global value_low
    global value_high
    global backprojection_threshold
    global training_file_name

    # TODO check if the keys exist in the config dictionary or else error
    # TODO also check if invalid values

    global_threshold = config['global_threshold']
    global_threshold_factor = config['global_threshold_factor']
    adaptive_threshold_block_size = config['adaptive_threshold_block_size']
    adaptive_threshold_C = config['adaptive_threshold_C']
    blur_size = config['blur_size']
    canny_threshold = config['canny_threshold']
    max_erode_iterations = config['max_erode_iterations']
    bandpass_low_cutoff = config['bandpass_low_cutoff']
    bandpass_high_cutoff = config['bandpass_high_cutoff']
    hue_low = config['hue_low']
    hue_high = config['hue_high']
    saturation_low = config['saturation_low']
    saturation_high = config['saturation_high']
    value_low = config['value_low']
    value_high = config['value_high']
    backprojection_threshold = config['backprojection_threshold']
    training_file_name = config['training_file_name']
    return config

def main(args):
    # TODO make this file into a class
    # create a line_detection object
    ld = line_detection()

    # start the line_detector node and start listening
    rospy.init_node('backprojectgrass_skeletonize')
    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, reconfigure_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
