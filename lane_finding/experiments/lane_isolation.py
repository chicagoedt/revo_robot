import cv2
import numpy as np
import sys
import string
import random

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

def normalize(contour):
    lowest_x = 1e9
    lowest_y = 1e9
    for px in contour[0]:
        if px[0] < lowest_x:
            lowest_x = px[0]
        if px[1] < lowest_y:
            lowest_y = px[1]
    for px in contour[0]:
        px[0] = px[0] - lowest_x
        px[1] = px[1] - lowest_y
    return contour

def getID(size=6, chars=string.ascii_lowercase + string.digits):
    return ''.join(random.choice(chars) for _ in range(size))

def getBadCountours(img, contours):
    eyedee = getID()
    print eyedee
    cv2.imwrite("data/" + eyedee + ".jpg", img)
    bad = open("data/" + eyedee + "_bad", 'w')
    good = open("data/" + eyedee + "_good",'w')
    print "Hit 'd' if contour is bad, 'k' if contour is good, otherwise hit any other key."
    for c in contours:
        img = img * 0
        cv2.drawContours(img, [c], -1, 255, -1)
        cv2.imshow('Check', img)
        kp = cv2.waitKey(0)
        if 0xFF & kp == ord('d'):
            bad.write(str(normalize(c)) + '\n')
        elif 0xFF & kp == ord('k'):
            good.write(str(normalize(c)) + '\n')
        elif 0xFF & kp == 27:
            break
    cv2.destroyWindow('Check')


cap = cv2.VideoCapture(sys.argv[1])
counter = 0

while True:
    ret, full_frame = cap.read()
    height, width = full_frame.shape[:2]
    cropped_frame = full_frame[0:height - 200, 0:width]
    height, width = cropped_frame.shape[:2]
    reframe = cv2.resize(cropped_frame, (width/2, height/2))
    frame = cv2.GaussianBlur(reframe, ksize=(3,3), sigmaX=10)

    intensity = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)

    canny = cv2.Canny(saturation, 128, 255)
    sat_thresh = thresh( saturation, (30,80), (0,100) )

    athresh = cv2.adaptiveThreshold(saturation, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,33,12)
    contours, hierarchy = cv2.findContours(athresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    bigContours = []
    for c in contours:
        if cv2.contourArea(c) > 200:
            bigContours.append(c)

    con = athresh.copy() * 0
    cv2.drawContours(con, bigContours, -1, 255, -1)

    cv2.imshow( 'Original', frame )
    cv2.imshow( 'Saturation', saturation)
    cv2.imshow( 'Contoured', con )

    if cap.get(1) == cap.get(7): # Enums are broken, 1 is frame position, 7 is frame count
        cap.set(1, 0)

    press = cv2.waitKey(5)
    if 0xFF & press == 27:
        break
    if 0xFF & press == 32:
        tmpimg = con.copy()
        getBadCountours(tmpimg, bigContours)
    press = None

cap.release()
cv2.destroyAllWindows()
