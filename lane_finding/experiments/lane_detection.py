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

def mag_thresh( img, sobel_kernel=3, thresh=(0, 255) ):
    sobelx = cv2.Sobel(img, cv2.CV_8U, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(img, cv2.CV_8U, 0, 1, ksize=sobel_kernel)
    gradmag = np.sqrt(sobelx**2 + sobely**2)
    scale_factor = np.max(gradmag)/255
    gradmag = (gradmag/scale_factor).astype(np.uint8)
    binary_output = np.zeros_like(gradmag)
    binary_output[(gradmag >= thresh[0]) & (gradmag <= thresh[1])] = 255

    return binary_output

def dir_thresh( img, sobel_kernel=3, thresh=(0,np.pi/2) ):
    sobelx = cv2.Sobel(img, cv2.CV_8U, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(img, cv2.CV_8U, 0, 1, ksize=sobel_kernel)
    absgraddir = np.absolute(np.arctan2(np.absolute(sobely), np.absolute(sobelx)))
    binary_output = np.zeros_like(absgraddir)
    binary_output[(absgraddir >= thresh[0]) & (absgraddir <= thresh[1])] = 255
    binary_output = binary_output.astype(np.uint8)

    return binary_output

while True:
    ret, full_frame = cap.read()
    height, width = full_frame.shape[:2]
    reframe = cv2.resize(full_frame, (width/3, height/3))
    frame = cv2.GaussianBlur(reframe, ksize=(3,3), sigmaX=10)

    intensity = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue, saturation, intensity = cv2.split(hsv)

    sobelXSat = abs_sobel_thresh(saturation, thresh=(30,60))
    sobelXInt = abs_sobel_thresh(intensity, thresh=(20,60))
    magSat = mag_thresh(saturation, thresh=(60,80))
    magInt = mag_thresh(intensity, thresh=(60,80))
    dirSat = dir_thresh(saturation, sobel_kernel=7, thresh=(0.7,1.3))
    dirInt = dir_thresh(intensity, sobel_kernel=7, thresh=(0.7,1.3))

    cv2.imshow( 'Original', frame )
    cv2.imshow( 'Intensity', magInt )
    cv2.imshow( 'Saturation', magSat )

    if 0xFF & cv2.waitKey(5) == 27:
        break

cap.release()
cv2.destroyAllWindows()
