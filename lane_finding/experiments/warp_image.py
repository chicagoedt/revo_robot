import cv2
import numpy as np
import sys

full_img = cv2.imread(sys.argv[1])
height, width = full_img.shape[:2]
img = cv2.resize(full_img, (width/4, height/4))
rows,cols,ch = img.shape
pts1 = np.float32([[330,375],[460,370],[310,610],[450,610]])
pts2 = np.float32([[330,375],[460,375],[330,689],[460,689]])
M = cv2.getPerspectiveTransform(pts1,pts2)
print M
dst = cv2.warpPerspective(img,M,(width/4, height/4))
cv2.imshow('img', img)
cv2.imshow('dst', dst)
cv2.waitKey(0)
