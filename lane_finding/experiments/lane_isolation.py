import cv2
import numpy as np
import sys

cap = cv2.VideoCapture(sys.argv[1])




counter = 0
while True:
    ret, full_frame = cap.read()
    height, width = full_frame.shape[:2]
    reframe = cv2.resize(full_frame, (width/3, height/3))
    frame = cv2.GaussianBlur(reframe, ksize=(3,3), sigmaX=10)

    intensity = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)

    cv2.imshow( 'Original', frame )
    cv2.imshow( 'sat_thresh', sat_thresh)
    cv2.imshow( 'Saturation', saturation)

    if cap.get(1) == cap.get(7): # Enums are broken, 1 is frame position, 7 is frame count
        cap.set(1, 0)


    if 0xFF & cv2.waitKey(5) == 27:
        break

cap.release()
cv2.destroyAllWindows()
