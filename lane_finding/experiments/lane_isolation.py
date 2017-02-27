import cv2
import numpy as np
import sys

def thresh(img, tightThresh, wideThresh):
    pixels_to_check = set()
    rows = img.shape[0]
    cols = img.shape[1]
    thresh_img = img.copy() * 0 + 100

    for x in xrange(rows):
        for y in xrange(cols):
            px = (x,y)
            val = img.item(px)
            if val > tightThresh[0] and val < tightThresh[1]:
                pixels_to_check.add(px)
                thresh_img.itemset(px, 255)
    while len(pixels_to_check) > 0:
        p = pixels_to_check.pop()
        adjacent_pixels = []
        for x in [ p[0]-1, p[0], p[0]+1]:
            for y in [ p[1]-1, p[1], p[1]+1]:
                if x >= 0 and y >=0 and x < rows and y < cols:
                    adjacent_pixels.append( (x,y) )
        for a in adjacent_pixels:
            if thresh_img.item(a) == 100:
                val = img.item(a)
                if val > wideThresh[0] and val < wideThresh[1]:
                    pixels_to_check.add(a)
                    thresh_img.itemset(a, 128)
                else:
                    thresh_img.itemset(a, 0)
    thresh_img[thresh_img == 100] = 0

    return thresh_img


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

    sat_thresh = thresh( saturation, (30,80), (0,120) )

    cv2.imshow( 'Original', frame )
    cv2.imshow( 'Canny', cv2.Canny(saturation, 100, 255))
    cv2.imshow( 'sat_thresh', sat_thresh)
    cv2.imshow( 'Saturation', saturation)

    if cap.get(1) == cap.get(7): # Enums are broken, 1 is frame position, 7 is frame count
        cap.set(1, 0)


    if 0xFF & cv2.waitKey(5) == 27:
        break

cap.release()
cv2.destroyAllWindows()
