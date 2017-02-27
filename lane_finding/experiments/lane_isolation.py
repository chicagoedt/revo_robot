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

def isWeird(contour):
    convexHull = cv2.convexHull(contour)
    clength = cv2.arcLength(contour, True)
    hlength = cv2.arcLength(convexHull, True)
    if clength / hlength > 2.5:
        return True
    else:
        return False

def getBadCountours(img, contours):
    print "Hit 'd' if contour is bad, otherwise hit any other key."
    for c in contours:
        img = img * 0
        cv2.drawContours(img, [c], -1, 255, -1)
        cv2.imshow('Check', img)
        kp = cv2.waitKey(0)
        if 0xFF & kp == ord('d'):
            print "YOU WILL BE DELETED"
        elif 0xFF & kp == 27:
            break


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

    canny = cv2.Canny(saturation, 128, 255)
    sat_thresh = thresh( saturation, (30,80), (0,100) )

    athresh = cv2.adaptiveThreshold(saturation, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,33,12)
    contours, hierarchy = cv2.findContours(athresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    bigContours = []
    normalContours = []
    for c in contours:
        if cv2.contourArea(c) > 200:
            bigContours.append(c)
    for c in bigContours:
        if not isWeird(c):
            normalContours.append(c)

    con = athresh.copy() * 0
    cv2.drawContours(con, normalContours, -1, 255, -1)

    cv2.imshow( 'Original', frame )
    cv2.imshow( 'Saturation', saturation)
    cv2.imshow( 'Contoured', con )

    if cap.get(1) == cap.get(7): # Enums are broken, 1 is frame position, 7 is frame count
        cap.set(1, 0)

    press = cv2.waitKey(5)
    if 0xFF & press == 27:
        break
    if 0xFF & press == 32:
        tmpimg = frame.copy() * 0
        getBadCountours(tmpimg, normalContours)
    press = None

cap.release()
cv2.destroyAllWindows()
