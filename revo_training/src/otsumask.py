import cv2
import sys, random, string, os

directory = 'data/segmentation/validation/'
img_dir = directory + 'images/imgs/'
images = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser(img_dir)) for f in fn]

random.shuffle(images)

for i in images:
    name = i.rsplit('/', 1)[-1]
    img = cv2.imread(i)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    blur = cv2.GaussianBlur(saturation, (5,5), 0)
    ret, othresh = cv2.threshold(blur,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

    cv2.imwrite(directory + 'otsu/otsu/' + name, othresh)
    cv2.imwrite(directory + 'hsv/hsv/' + name, hsv)
