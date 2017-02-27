import cv2
import numpy as np
import sys

cap = cv2.VideoCapture(sys.argv[1])
counter = 0
# (x,y)
pt1 = ( 150, 580 ) # upper left
pt2 = ( 345, 585 ) # upper right
pt3 = ( 133, 747 ) # lower left
pt4 = ( 345, 745 ) # lower right

pts1 = np.float32([pt1,pt2,pt3,pt4])
pts2 = np.float32([[150,580],[300,580],[150,730],[300,730]])

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
