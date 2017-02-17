import cv2
import numpy as np
import sys

cap = cv2.VideoCapture(sys.argv[1])
counter = 0

pts1 = np.float32([[330,375],[460,370],[310,610],[450,610]])
pts2 = np.float32([[330,375],[460,375],[330,689],[460,689]])
M = cv2.getPerspectiveTransform(pts1,pts2)

while True:
    ret, frame = cap.read()
    height, width = frame.shape[:2]
    dst = cv2.warpPerspective(frame, M, (width, height))
    cv2.imshow('original',frame)
    cv2.imshow('warp', dst)

    if cap.get(1) == cap.get(7):
        cap.set(1,0)

    if 0xFF & cv2.waitKey(5) == 27:
        break
