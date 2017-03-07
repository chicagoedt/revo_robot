#!/usr/bin/env python
import sys
import cv2

cap = cv2.VideoCapture(sys.argv[1])
height, width = img.shape[:2]
fourcc = cv2.cv.CV_FOURCC(*'XVID')
video_out = cv2.VideoWriter('output.avi', fourcc, 20.0, (width,height))








    try:
        out.write(cv_image)
        cv2.imwrite('frame.jpg',cv_image)


