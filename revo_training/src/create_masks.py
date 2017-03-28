import cv2
import sys, random, string, os

img_dir = 'data/classification/training/lanes/'
images = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser(img_dir)) for f in fn]

random.shuffle(images)

def getID(size=6, chars=string.ascii_lowercase + string.digits):
   return ''.join(random.choice(chars) for _ in range(size))


def getContours(img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    bigContours = []
    for c in contours:
        if cv2.contourArea(c) > 1000:
            bigContours.append(c)
    con = img.copy() * 0
    cv2.drawContours(con, bigContours, -1, 255, -1)
    return con

C = 2

for i in images:
    img = cv2.imread(i)
    cv2.imshow('Original', img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    blur = cv2.GaussianBlur(saturation,(5,5),0)

    ret,othresh = cv2.threshold(blur,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    ocon = getContours(othresh)

    while True:
        if random.random() < 0.2:
            save_dir = 'data/segmentation/validation/'
        else:
            save_dir = 'data/segmentation/training/'
        ID = getID()
        athresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 35, C)
        acon = getContours(athresh)
        cv2.imshow('Otsu Threshed', ocon)
        cv2.imshow('Adaptive Threshed', acon)

        press = 0xFF & cv2.waitKey(0)
        if press == 27:
            sys.exit()
        elif press == 32:
            break
        elif press == ord('a'):
            cv2.imwrite(save_dir + 'images/' + ID + '.png' , img)
            cv2.imwrite(save_dir + 'masks/' + ID + '.png', acon)
            print ID + " written to " + save_dir + " using adaptive thresholded mask."
            break
        elif press == ord('o'):
            cv2.imwrite(save_dir + 'images/' + ID + '.png', img)
            cv2.imwrite(save_dir + 'masks/' + ID + '.png', ocon)
            print ID + " written to " + save_dir + " using Ostu's Binarization for thresholding."
            break
        elif press == 82 and C > 0:
            C -= 1
            print C
        elif press == 84:
            C += 1
            print C

