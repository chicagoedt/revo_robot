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
adaptive_threshold_block_size = 101
adaptive_threshold_C = 30
blur_size = 49
# canny_threshold = 100
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

file_number = 1
backprojection_threshold = 50

use_cam = False

## initialize webcam
cam = VideoCapture()


def nothing(x):
    pass

# converts pixels from camera space to 3-d space
## TODO take all these constants from ROS parameters
def publish_line_data(camera_image, horizon = 0) :

    # get indices of nonzero elements (pixels that are lines)
    pixels = np.transpose(camera_image.nonzero())
    #pixels contains the line obstacle pixels that need to be converted to PCL


    #height = height of the camera from the ground in meters
    height = 0.7
    orien = 65 # 90 - 25
    #orien = orientation of the camera (pitch angle from vertical)
    
    ##TODO find dimensions dynamically

    image_width = camera_image.shape[0]
    image_height = camera_image.shape[1]  
    horAngl = 120.0 #field of view
    verAngl = 120.0 * 9.0 / 16.0 #field of view

    #reference horizon that will be used in calculations.
    refHor = image_height - horizon

    #differnce in angle that each pixel corresponds to
    dAV = verAngl / image_height
    
    #create the cloud
    # cloud = sensor_msgs.msg.PointCloud(rospy.Time.now(), pixels, 0)


    # cloud = sensor_msgs.msg.PointCloud2()
    #create the header
    # cloud.header.stamp = rospy.Time.now()
    # cloud.header.frame_id = "sensor_frame"

    # cloud.channels.resize(1)
    #sets the size of the cloud. This was need in c++ so i think it will be needed in python
    # cloud.resize(len(pixels))
    # cloud.set_points_size(len(pixels))

    cloud_points = np.array([[0 for i in range(0,3)] for j in range(len(pixels))], dtype = float)
    # print "length of cloud points is ", len(cloud.points)
    # print pixels[0][0]
    #if there is no horizon argument passed into the function it will default to this method
    if horizon == 0 :
        for i in range(0,len(pixels)) :
            temp = height * math.tan((orien - (verAngl / 2) + (image_height - pixels[i][1])*dAV) * math.pi / 180.0)
            cloud_points[i][0] = (float(pixels[i][0] - (image_width / 2)) / (image_width / 2)) * temp * math.tan(horAngl * math.pi / 180.0)
            cloud_points[i][1] = temp
            cloud_points[i][2] = 0.0


    else :
        #reference horizon that will be used in calculations
        refHor = image_height - horizon

        #fills the cloud with the line data
        for i in range(0,len(pixels)) :
            temp = height * math.tan( (90 - (refHor - abs(pixels[i][1] - image_height)) * dAV) * math.pi / 180.0)
            cloud_points[i][0] = (float(pixels[i][0] - (image_width / 2)) / (image_width / 2)) * temp * math.tan(horAngl * math.pi / 180.0)
            cloud_points[i][1] = temp
            cloud_points[i][2] = 0.0


    # cloud_header = std_msgs.header.Header()

    print cloud_points

    cloud = sensor_msgs.msg.PointCloud2()
    cloud = sensor_msgs.point_cloud2.create_cloud_xyz32(cloud.header, cloud_points)

    cloud.header.frame_id = "base_camera"

    linePub.publish(cloud)

    return



def render_image():

    start_time = time.time()


    if use_cam:
        success = True
        if not cam.isOpened():
            success = cam.open(0)
        if success: 
            ok, img = cam.read()
    else:
        file_name = 'parking_test' + str(file_number) + '.jpg'
        img = cv2.imread(file_name,1) # read in image (from file)

        ##TODO check if file doesnt exist

    # our region of interest is only in bottom half of image
    horizon = img.shape[0]/2
    roi = img[horizon:, :]

    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) # convert BGR image (by default using imshow) to GRAY


    # roi = gray_image


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

    backprojection_training = cv2.imread('training_for_backprojection_1.png')
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
    cv2.imshow('backprojection_result', after_backprojection)

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
    print "normalized brightness: ", normalized_brightness
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
    cv2.imshow('before adaptive threshold', equ)
    thresh = cv2.adaptiveThreshold(equ, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, adaptive_threshold_block_size, adaptive_threshold_C - 30)
    cv2.imshow("after adaptive threshold", thresh)
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

    print "time elapsed: ", (end_time - start_time)    

    
    final_image = skel
    
    # show images using gui
    cv2.imshow('original', roi)
    # cv2.imshow('gray_roi', gray_roi)
    cv2.imshow('final image', final_image)

    ## TODO extract line_data from final_image
    return final_image

## end render_image()

## start main procedure

# initialize ROS stuff
## TODO find out what name the topic should have
linePub = rospy.Publisher('line_data',sensor_msgs.msg.PointCloud2)
rospy.init_node('line_detection_GUI', anonymous=True)


# create GUI elements (windows and trackbars)
# cv2.namedWindow('gray_roi', cv2.WINDOW_AUTOSIZE) # create a window for gray image
cv2.namedWindow('final', cv2.WINDOW_AUTOSIZE) # create a window for canny image
# cv2.namedWindow('laplacian', cv2.WINDOW_AUTOSIZE) # create a window for laplacian image
# cv2.namedWindow('afterBandpass', cv2.WINDOW_AUTOSIZE) # create a window for gray image
# cv2.createTrackbar('canny_threshold_trackbar', 'final', canny_threshold, canny_threshold*3, nothing) # create a trackbar for canny threshold
# cv2.createTrackbar('bandpass_low_cutoff_trackbar', 'final', bandpass_low_cutoff, 100, nothing) # create a trackbar for low bandpass 
# cv2.createTrackbar('bandpass_high_cutoff_trackbar', 'final', bandpass_high_cutoff, 1000, nothing) # create a trackbar for high bandpass
cv2.createTrackbar('adaptive_threshold_block_size_trackbar', 'final', adaptive_threshold_block_size, 300, nothing) # create a trackbar for adaptive threshold block size
# cv2.createTrackbar('adaptive_threshold_C_trackbar', 'final', adaptive_threshold_C, 100, nothing) # create a trackbar for adaptive threshold C value (just an offset)
cv2.createTrackbar('blur_size_trackbar', 'final', blur_size, 101, nothing) # create a trackbar for blur size
cv2.createTrackbar('global_threshold_trackbar', 'final', global_threshold, 255, nothing) # create a trackbar for global threshold

cv2.createTrackbar('hue_low_trackbar', 'final', hue_low, 179, nothing) # create a trackbar for hue_low (hsv threshold)
cv2.createTrackbar('hue_high_trackbar', 'final', hue_high, 179, nothing) # create a trackbar for hue_high (hsv threshold)
# cv2.createTrackbar('saturation_low_trackbar', 'final', saturation_low, 255, nothing) # create a trackbar for saturation_low (hsv threshold)
# cv2.createTrackbar('saturation_high_trackbar', 'final', saturation_high, 255, nothing) # create a trackbar for saturation_high (hsv threshold)
# cv2.createTrackbar('value_low_trackbar', 'final', value_low, 255, nothing) # create a trackbar for value_low (hsv threshold)
# cv2.createTrackbar('value_high_trackbar', 'final', value_high, 255, nothing) # create a trackbar for value_high (hsv threshold)
cv2.createTrackbar('file_number_trackbar', 'final', file_number, 7, nothing) # create a trackbar for file_number threshold
cv2.createTrackbar('backprojection_threshold_trackbar', 'final', backprojection_threshold, 255, nothing) # create a trackbar for backprojection threshold



# render the image for the first time
line_data = render_image()
publish_line_data(line_data)

# wait for user key (such as arrow keys) and render image again if trackbars change; quit upon ESC hit.
while 1:
    k = cv2.waitKey(0) & 0xFF
    if k == 27:         # wait for ESC key to exit
        break

    # # if canny threshold (trackbar) has changed, render image again
    # if (canny_threshold != cv2.getTrackbarPos('canny_threshold_trackbar', 'final')):
    #     canny_threshold = cv2.getTrackbarPos('canny_threshold_trackbar', 'final')
        
    #     # make sure value is greater than zero
    #     canny_threshold = canny_threshold + 1 if canny_threshold == 0 else canny_threshold

    #     render_image()

    # else if adap. block size (trackbar) has changed, render image again
    if (adaptive_threshold_block_size != cv2.getTrackbarPos('adaptive_threshold_block_size_trackbar', 'final')):
        adaptive_threshold_block_size = cv2.getTrackbarPos('adaptive_threshold_block_size_trackbar', 'final')

        # make sure value is odd and more than 2
        adaptive_threshold_block_size = adaptive_threshold_block_size + 1 if adaptive_threshold_block_size % 2 == 0 else adaptive_threshold_block_size
        adaptive_threshold_block_size = adaptive_threshold_block_size + 2 if adaptive_threshold_block_size == 1 else adaptive_threshold_block_size

        line_data = render_image()
        publish_line_data(line_data)

    # # else if adap. block size (trackbar) has changed, render image again
    # if (adaptive_threshold_C != cv2.getTrackbarPos('adaptive_threshold_C_trackbar', 'final')):
    #     adaptive_threshold_C = cv2.getTrackbarPos('adaptive_threshold_C_trackbar', 'final')
    #     render_image()


    # else if blur size (trackbar) has changed, render image again
    if (blur_size != cv2.getTrackbarPos('blur_size_trackbar', 'final')):
        blur_size = cv2.getTrackbarPos('blur_size_trackbar', 'final')

        # make sure value is odd and more than 2
        blur_size = blur_size + 1 if blur_size % 2 == 0 else blur_size
        blur_size = blur_size + 2 if blur_size == 1 else blur_size

        line_data = render_image()
        publish_line_data(line_data)

    # else if global threshold (trackbar) has changed, render image again
    if (global_threshold != cv2.getTrackbarPos('global_threshold_trackbar', 'final')):
        global_threshold = cv2.getTrackbarPos('global_threshold_trackbar', 'final')
        
        line_data = render_image()
        publish_line_data(line_data)
    # no need for global_threshold value check (all values are valid)



    # if (bandpass_low_cutoff != cv2.getTrackbarPos('bandpass_low_cutoff_trackbar', 'final')):
    #     bandpass_low_cutoff = cv2.getTrackbarPos('bandpass_low_cutoff_trackbar', 'final')
    #     render_image()

    # if (bandpass_high_cutoff != cv2.getTrackbarPos('bandpass_high_cutoff_trackbar', 'final')):
    #     bandpass_high_cutoff = cv2.getTrackbarPos('bandpass_high_cutoff_trackbar', 'final')
        
    #     # set high value to be at least low value
    #     bandpass_high_cutoff = bandpass_low_cutoff if bandpass_high_cutoff < bandpass_low_cutoff else bandpass_high_cutoff
    #     render_image()

    # else if hue_low (trackbar) has changed, render image again
    if (hue_low != cv2.getTrackbarPos('hue_low_trackbar', 'final')):
        hue_low = cv2.getTrackbarPos('hue_low_trackbar', 'final')
        
        line_data = render_image()
        publish_line_data(line_data)
    # else if hue_high (trackbar) has changed, render image again
    if (hue_high != cv2.getTrackbarPos('hue_high_trackbar', 'final')):
        hue_high = cv2.getTrackbarPos('hue_high_trackbar', 'final')
        
        line_data = render_image()
        publish_line_data(line_data)
    # # else if saturation_low (trackbar) has changed, render image again
    # if (saturation_low != cv2.getTrackbarPos('saturation_low_trackbar', 'final')):
    #     saturation_low = cv2.getTrackbarPos('saturation_low_trackbar', 'final')
    #     render_image()
    # # else if saturation_high (trackbar) has changed, render image again
    # if (saturation_high != cv2.getTrackbarPos('saturation_high_trackbar', 'final')):
    #     saturation_high = cv2.getTrackbarPos('saturation_high_trackbar', 'final')
    #     render_image()
    # # else if value_low (trackbar) has changed, render image again
    # if (value_low != cv2.getTrackbarPos('value_low_trackbar', 'final')):
    #     value_low = cv2.getTrackbarPos('value_low_trackbar', 'final')
    #     render_image()
    # # else if value_high (trackbar) has changed, render image again
    # if (value_high != cv2.getTrackbarPos('value_high_trackbar', 'final')):
    #     value_high = cv2.getTrackbarPos('value_high_trackbar', 'final')
    #     render_image()

    # else if backprojection_threshold (trackbar) has changed, render image again
    if (backprojection_threshold != cv2.getTrackbarPos('backprojection_threshold_trackbar', 'final')):
        backprojection_threshold = cv2.getTrackbarPos('backprojection_threshold_trackbar', 'final')
        
        line_data = render_image()
        publish_line_data(line_data)



    # else if file_number (trackbar) has changed, render image again
    if (file_number != cv2.getTrackbarPos('file_number_trackbar', 'final')):
        file_number = cv2.getTrackbarPos('file_number_trackbar', 'final')

        file_number = 1 if file_number==0 else file_number

        line_data = render_image()
        publish_line_data(line_data)


# destroy all objects
cv2.destroyAllWindows()
