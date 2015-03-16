import numpy as np
import cv2

cap = cv2.VideoCapture(0)                                                                               # cature video from cam
#fourcc = cv2.cv.CV_FOURCC(*'XVID')                                                                     # video codec
#out = cv2.VideoWriter('/home/batata/Documents/trash/draft/output33.avi', fourcc,20.0, (1280,480))      # output file

# see squares.py example in opencv documentationv*********

def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

def shape_track(img):
    img = cv2.GaussianBlur(img, (5, 5), 0)       # Blurs an image using a Gaussian filter. GaussianBlur( src, dst, Size( i, i ), 0, 0 )
    squares = []                                 # empty list
    for gray in cv2.split(img):                  # splitting to diffrent channels
        for thrs in xrange(0, 255, 26):          
            if thrs == 0:
                bin = cv2.Canny(gray, 0, 50, apertureSize=5)       # canny filter
                bin = cv2.dilate(bin, None)                        # dilation = cv2.dilate(img,kernel,iterations_value)
            else:
                retval, bin = cv2.threshold(gray, thrs, 255, cv2.THRESH_BINARY)
            contours, hierarchy = cv2.findContours(bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) # in begining( bin, ) removed
            for cnt in contours:
                cnt_len = cv2.arcLength(cnt, True)
                cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)                                     # approximating the contour more to straight line
                if len(cnt) == 4 and cv2.contourArea(cnt) > 1000 and cv2.isContourConvex(cnt):      # cv2.contourArea() this decide the minimum area to detect 
                                                                                                    # cv2.isContourConvex()
                    cnt = cnt.reshape(-1, 2)
                    max_cos = np.max([angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)]) # function def angle_cos() is called
                    if max_cos < 0.1:                                                                          # remember, cos(0)=1, cos(90)=0, cos(89.247)=0.1 ## changed from max_cos < 0.1 to < 1
                        squares.append(cnt)                                                                    # append cnt to list square                       
    return squares                                                                                             # return list squares

def color_track(img):

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # convert from BGR to HSV
    lower_red1 = np.array([0,120,120])         # red rang1 note that red-color has 2 ranges
    upper_red1 = np.array([12,255,255])   

    lower_red2 = np.array([160,100,100])       # red range2
    upper_red2 = np.array([179,255,255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)    # using range1
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)    # using range2

    mask = cv2.bitwise_or(mask1,mask2)                  # using bitwise_or
    res = cv2.bitwise_and(img,img, mask= mask)          # using bitwise_and
    return res

while(True):                                            # while loop only exits when user press 'q'

    ret, frame = cap.read()
    color = color_track(frame)                              # call function for color

    squares = shape_track(frame)                            # call function for shape
    cv2.drawContours(frame, squares, -1, (0, 255, 0), 3 )   # draw square in fram
    #cv2.imshow('frame',frame)                              # show frame after editing
    
    squares = shape_track(color)                            # call square function on frame that only has our color 
    cv2.drawContours(color, squares, -1, (0, 255, 0), 3 )   # draw square on the one-color frame
    #cv2.imshow('res',color)                                # show frame
    both = np.hstack((frame,color))                 
    cv2.imshow('join, press ''q'' to exit', both)           # shows both frame side by side
    #out.write(both)                                        # writting the frame into output file
    if cv2.waitKey(1) & 0xFF == ord('q'):                   # press 'q' while in frame to stop
        break

cap.release()                                               # When everything done, release the capture  
#out.release()                                               # When everything done, release the capture 
cv2.destroyAllWindows()                                     # the end
