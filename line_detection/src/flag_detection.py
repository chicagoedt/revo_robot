#!/usr/bin/env python
import sys
import numpy as np
import cv2
import rospy
from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
from line_detection.cfg import LineDetectionConfig

###############################################################################
## Chicago Engineering Design Team
## Flag Detection using Python OpenCV for autonomous robot Scipio
##    (IGVC competition).
## @author Mohammad Badwan
## @email mbadwa3@uic.edu
#######################################################


class FlagDetection(LaneDetection):
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000

    squares   = []
    #cnt_blue = []
    #cnt_red  = []
    xx_pos    = 0
    #i        = 0
    #cnt_max  = []
    ##########################

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)


    def color_track(self,img,l_b,u_b): # ********************************************************************************************************************************* # 
        ######################
    #    img = cv2.flip(img, 1)
        ######################
        hsv = cv2.GaussianBlur(img, (5,5), 0)
        hsv = cv2.cvtColor(hsv, cv2.COLOR_BGR2HSV) 
        ######################
        lower_bounder = np.array(l_b, np.uint8)         
        upper_bounder = np.array(u_b, np.uint8)
        ######################
        mask     = cv2.inRange(hsv, lower_bounder, upper_bounder)    
        dilation = np.ones((1, 1), "uint8")
        mask     = cv2.dilate(mask, dilation)
        res      = cv2.bitwise_and(img,img, mask= mask)    
        ######################
        return res


    def angle_cos(self,p0, p1, p2): # ********************************************************************************************************************************* # 
        ######################
        d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
        ######################
        return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

    def center_mass(self, img): # ********************************************************************************************************************************* # 
        ######################
    #    global cnt_max
        blur = cv2.GaussianBlur(img, (5, 5), 0)   
        ######################                                   
        for gray in cv2.split(blur):                 
            for thrs in xrange(0, 255, 26): 
                ##############         
                if thrs == 0:
                    bin = cv2.Canny(gray, 0, 50, apertureSize=5)       
                    bin = cv2.dilate(bin, None)                        
                else:
                    retval, bin = cv2.threshold(gray, thrs, 255, cv2.THRESH_BINARY)
                contours, hierarchy = cv2.findContours(bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) 
                ##############
                for cnt in contours:
                    cnt_moment = cnt
                    cnt_len = cv2.arcLength(cnt, True)
                    cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)                                     
                    if len(cnt) == 4 and cv2.contourArea(cnt) > 500: # and cv2.isContourConvex(cnt):   
    #                if cv2.contourArea(cnt) > 1000 and cv2.isContourConvex(cnt):                                                                 
                        cnt = cnt.reshape(-1, 2)
                        max_cos = np.max([self.angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)]) 
                        if max_cos < 0.1:   
                            self.squares.append(cnt)      
                            moments = cv2.moments(cnt_moment) 
                            if moments['m00']!=0:
    #                            cnt_max.append(cnt_moment)
                                cx     = int(moments['m10']/moments['m00']) 
                                self.xx_pos = cx
                                cy     = int(moments['m01']/moments['m00'])
                                cv2.circle(img,(cx,cy),5,(0,255,0),-1) 
                ############## 
        return img


    def max_cnt(cnt): # ********************************************************************************************************************************* #
        ######################
        cnt_moment = cnt
        cnt_len = cv2.arcLength(cnt, True)
        cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)  
        ######################                                   
        if len(cnt) == 4 and cv2.contourArea(cnt) > 1000 and cv2.isContourConvex(cnt):                                                                                     
            cnt = cnt.reshape(-1, 2)
            max_cos = np.max([self.angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)]) 
            if max_cos < 0.1:       
                moments = cv2.moments(cnt_moment) 
                if moments['m00']!=0:
                    cx     = int(moments['m10']/moments['m00']) 
        ######################
        return cx 

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)
        roi = LaneDetection.get_roi(self, cv2_image)
        #######################
        frame = roi
        global cnt_max
        #######################
        frame_b  = self.color_track(frame, [100, 150, 100], [130, 255, 255])
        frame_r1 = self.color_track(frame, [  0, 200, 150], [  5, 255, 255])
        frame_r2 = self.color_track(frame, [169, 200, 150], [179, 255, 255])
        frame_r  = cv2.bitwise_or(frame_r1, frame_r2)
        #######################  
        frame_rr    = self.center_mass(frame_r)
        squares_red = self.squares
        red_x       = self.xx_pos
    #    cnt_max_r   = cnt_max
    #    cv2.drawContours(frame_rr, squares_red, 0, (0, 0, 255), 0 )
        self.squares     = []
    #    cnt_max     = []
        #######################
        frame_bb     = self.center_mass(frame_b)
        squares_blue = self.squares
        blue_x       = self.xx_pos
    #    cnt_max_b    = cnt_max
    #    cv2.drawContours(frame_bb, squares_blue, 0, (0, 0, 255), 0 )
        self.squares      = []
    #    cnt_max      = []
        #######################
    #    cnt_max_b.sort()
    #    cnt_max_r.sort()
    #    if cnt_max_b:
    #        cmbm = max(cnt_max_b)
    #        cmb = max_cnt(cmbm)
    #    if cnt_max_r:
    #        cmrm = max(cnt_max_r)
    #        cmr = max_cnt(cmrm)
        #######################
    #    if cnt_max_b and cnt_max_r:
    #        if   cmb > cmr:
    #            cv2.drawContours(frame_rr, squares_red, 0, (0, 0, 255), 0 )
    #            cv2.drawContours(frame_bb, squares_blue, 0, (0, 0, 255), 0 )
    #        elif cmb == cmr:
    #            cv2.drawContours(frame_rr, squares_red, 0, (0, 255, 0), 0 )
    #            cv2.drawContours(frame_bb, squares_blue, 0, (0, 255, 0), 0 )
    #        elif cmb < cmr:
    #            cv2.drawContours(frame_rr, squares_red, 0, (255, 0, 0), 0 )
    #            cv2.drawContours(frame_bb, squares_blue, 0, (255, 0, 0), 0 )
        if   blue_x > red_x:
            cv2.drawContours(frame_rr, squares_red, 0, (0, 0, 255), 0 )
            cv2.drawContours(frame_bb, squares_blue, 0, (0, 0, 255), 0 )
        elif blue_x == red_x:
            cv2.drawContours(frame_rr, squares_red, 0, (0, 255, 0), 0 )
            cv2.drawContours(frame_bb, squares_blue, 0, (0, 255, 0), 0 )
        elif blue_x < red_x:
            cv2.drawContours(frame_rr, squares_red, 0, (255, 0, 0), 0 )
            cv2.drawContours(frame_bb, squares_blue, 0, (255, 0, 0), 0 )
        #######################
        color = cv2.bitwise_or(frame_rr, frame_bb)
        both  = np.hstack((frame,color))
        
        final_image = color

        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )
        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "flag_detection"
    namespace = rospy.get_namespace()

    # create a FlagDetection object
    fd = FlagDetection(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("flag_detection", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, fd.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
