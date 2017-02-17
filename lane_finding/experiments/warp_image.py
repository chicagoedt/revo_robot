import cv2
import numpy as np
import sys

full_img = cv2.imread(sys.argv[1])
height, width = full_img.shape[:2]
img = cv2.resize(full_img, (width/4, height/4))

# (x,y)
pt1 = ( 150, 580 ) # upper left
pt2 = ( 345, 585 ) # upper right
pt3 = ( 133, 747 ) # lower left
pt4 = ( 345, 745 ) # lower right

radius = 2
color = (0,0,255)
thickness = 2

cv2.circle(img, pt1, radius, color, thickness)
cv2.circle(img, pt2, radius, color, thickness)
cv2.circle(img, pt3, radius, color, thickness)
cv2.circle(img, pt4, radius, color, thickness)

pts1 = np.float32([pt1,pt2,pt3,pt4])
pts2 = np.float32([[150,580],[300,580],[150,730],[300,730]])
M = cv2.getPerspectiveTransform(pts1,pts2)
print M
dst = cv2.warpPerspective(img,M,(width/4, height/4))
cv2.imshow('img', img)
cv2.imshow('dst', dst)
cv2.waitKey(0)
