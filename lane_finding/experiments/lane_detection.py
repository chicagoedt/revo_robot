import cv2
import numpy as np
import sys

cap = cv2.VideoCapture(sys.argv[1])

def abs_sobel_thresh( img, orient='x', sobel_kernel=3, thresh=(0,255) ):
    sobelx = cv2.Sobel(img, cv2.CV_8U, 1, 0, ksize=sobel_kernel)
    abs_sobelx = np.absolute(sobelx)
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    binary_output = np.zeros_like(scaled_sobel)
    binary_output[(abs_sobelx >= thresh[0]) & (abs_sobelx <= thresh[1])] = 255

    return binary_output

def mag_thresh( img, sobel_kernel=5, thresh=(0, 255) ):
    sobelx = cv2.Sobel(img, cv2.CV_8U, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(img, cv2.CV_8U, 0, 1, ksize=sobel_kernel)
    gradmag = np.sqrt(sobelx**2 + sobely**2)
    scaled_magnitude = np.uint8(255*gradmag/np.max(gradmag))
    binary_output = np.zeros_like(scaled_magnitude)
    binary_output[(scaled_magnitude >= thresh[0]) & (scaled_magnitude <= thresh[1])] = 255

    return binary_output

def dir_thresh( img, sobel_kernel=3, thresh=(0,np.pi/2) ):
    sobelx = cv2.Sobel(img, cv2.CV_8U, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(img, cv2.CV_8U, 0, 1, ksize=sobel_kernel)
    absgraddir = np.absolute(np.arctan2(np.absolute(sobely), np.absolute(sobelx)))
    binary_output = np.zeros_like(absgraddir)
    binary_output[(absgraddir >= thresh[0]) & (absgraddir <= thresh[1])] = 255
    binary_output = binary_output.astype(np.uint8)

    return binary_output

def houghOverlay ( img, rho=1, theta=np.pi/180, threshold=1, minLineLength=10, maxLineGap=1 ):
    lines = cv2.HoughLinesP( img, rho, theta, threshold, np.array([]), minLineLength, maxLineGap )
    hough = np.copy(frame)*0
    if lines is not None:
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(hough,(x1,y1),(x2,y2),(0,0,255),10)
    color_edges = np.dstack( (img, img, img) )
    hough_overlay = cv2.addWeighted(color_edges, 0.8, hough, 1, 0)

    return hough_overlay

#TODO
def getSlope(lines):
    slope = []
    for line in lines:
        rise = line[3] - line[1]
        run = line[2] - line[0]
        if rise is int and run is int:
            slope.append( rise / run )
    return np.mean(slope)

#TODO: Get more precise transform matrix.
def getPerspectiveTransform():
    return [[  1.33649160e+00   2.50032697e-01  -1.27593229e+02]
            [  1.50587524e-01   1.94616896e+00  -3.16767176e+02]
            [  2.18559541e-04   4.31596967e-04   1.00000000e+00]]



counter = 0
while True:
    ret, full_frame = cap.read()
    height, width = full_frame.shape[:2]
    reframe = cv2.resize(full_frame, (width/3, height/3))
    frame = cv2.GaussianBlur(reframe, ksize=(5,5), sigmaX=10)

    intensity = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)

    sobelXInt = abs_sobel_thresh(intensity, thresh=(20,60))
    sobelXSat = abs_sobel_thresh(saturation, thresh=(30,60))
    magInt = mag_thresh(intensity, thresh=(10,100))
    magSat = mag_thresh(saturation, thresh=(10,50))
    dirInt = dir_thresh(intensity, sobel_kernel=7, thresh=(0.7,1.3))
    dirSat = dir_thresh(saturation, sobel_kernel=7, thresh=(0.7,1.3))

    hough = houghOverlay(sobelXInt, threshold=20 )

    cv2.imshow( 'Original', frame )
    #cv2.imshow( 'Raw Canny', cv2.Canny(frame, 30, 120))
    #cv2.imshow( 'Intensity', sobelXInt )
    #cv2.imshow( 'Saturation', sobelXSat )
    cv2.imshow( 'Hough', hough )

    if cap.get(1) == cap.get(7): # Enums are broken, 1 is frame position, 7 is frame count
        cap.set(1, 0)


    if 0xFF & cv2.waitKey(5) == 27:
        break

cap.release()
cv2.destroyAllWindows()
