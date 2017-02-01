import cv2
import numpy as np
import sys

cap = cv2.VideoCapture(sys.argv[1])

def abs_sobel_thresh( img, orient='x', sobel_kernel=3, thresh=(40,70) ):
    sobelx = cv2.Sobel(img, cv2.CV_8U, 1, 0, ksize=sobel_kernel)
    abs_sobelx = np.absolute(sobelx)
    binary_output = np.zeros_like(abs_sobelx)
    binary_output[(abs_sobelx >= thresh[0]) & (abs_sobelx <= thresh[1])] = 255

    return binary_output

while True:
    ret, full_frame = cap.read()
    height, width = full_frame.shape[:2]
    reframe = cv2.resize(full_frame, (width/3, height/3))
    frame = cv2.GaussianBlur(reframe, ksize=(3,3), sigmaX=10)

    intensity = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue, saturation, intensity = cv2.split(hsv)

    cv2.imshow( 'Original', frame )
    cv2.imshow( 'Intensity', abs_sobel_thresh(intensity, thresh=(30,60)) )
    cv2.imshow( 'Saturation', abs_sobel_thresh(saturation, thresh=(30,60)) )

    if 0xFF & cv2.waitKey(5) == 27:
        break

cap.release()
cv2.destroyAllWindows()
