import cv2
import sys, random, string, os

img_dir = 'data/to_mask/lanes'
images = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser(img_dir)) for f in fn]

random.shuffle(images)
drawing = False

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

def mouseCallback(event, x, y, flags, param, image, window):
    global drawing
    if event == cv2.EVENT_MOUSEMOVE:
        img = image.copy()
        cv2.circle(img, (x, y), 20, (127,127,127), 1)
        cv2.circle(img, (x, y), 10, (127,127,127), 1)
        cv2.imshow(window, img)
        if drawing:
            cv2.circle(image, (x, y), 10, (255,255,255), -1)
            cv2.imshow(window, img)
            if window == 'Original':
                cv2.circle(draw, (x, y), 10, (255,255,255), -1)
                cv2.imshow('Draw', draw)
    elif event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        cv2.circle(image, (x, y), 10, (255,255,255), -1)
        cv2.imshow(window, image)
        if window == 'Original':
            cv2.circle(draw, (x, y), 10, (255,255,255), -1)
            cv2.imshow('Draw', draw)
    elif event == cv2.EVENT_RBUTTONDOWN:
        cv2.circle(image, (x, y), 20, (0,0,0), -1)
        cv2.imshow(window, image)
        if window == 'Original':
            cv2.circle(draw, (x, y), 20, (0,0,0), -1)
            cv2.imshow('Draw', draw)
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False

def callback_A(event, x, y, flags, param):
    mouseCallback(event, x, y, flags, param, acon, 'Adaptive')

def callback_O(event, x, y, flags, param):
    mouseCallback(event, x, y, flags, param, ocon, 'Otsu')

def callback_D(event, x, y, flags, param):
    mouseCallback(event, x, y, flags, param, img, 'Original')

C = 2
cv2.namedWindow('Original')
cv2.namedWindow('Draw')
cv2.setMouseCallback('Original', callback_D)
cv2.namedWindow('Adaptive')
cv2.setMouseCallback('Adaptive', callback_A)
cv2.namedWindow('Otsu')
cv2.setMouseCallback('Otsu', callback_O)
for i in images:
    img = cv2.imread(i)
    cv2.imshow('Original', img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    blur = cv2.GaussianBlur(saturation,(5,5),0)

    ret,othresh = cv2.threshold(blur,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    ocon = getContours(othresh)

    while True:
        draw = img.copy() * 0
        cv2.imshow('Draw', draw)
        if random.random() < 0.2:
            save_dir = 'data/segmentation_lines/validation/'
        else:
            save_dir = 'data/segmentation_lines/training/'
        ID = getID()
        athresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 35, C)
        acon = getContours(athresh)
        cv2.imshow('Otsu', ocon)
        cv2.imshow('Adaptive', acon)

        press = 0xFF & cv2.waitKey(0)
        if press == 27:
            sys.exit()
        elif press == 32:
            break
        elif press == ord('a'):
            cv2.imwrite(save_dir + 'images/imgs/' + ID + '.png' , img)
            cv2.imwrite(save_dir + 'masks/msks/' + ID + '.png', acon)
            print ID + " written to " + save_dir + " using adaptive thresholded mask."
            break
        elif press == ord('o'):
            cv2.imwrite(save_dir + 'images/imgs/' + ID + '.png', img)
            cv2.imwrite(save_dir + 'masks/msks/' + ID + '.png', ocon)
            print ID + " written to " + save_dir + " using Otsu's Binarization for thresholding."
            break
        elif press == ord('d'):
            cv2.imwrite(save_dir + 'images/imgs/' + ID + '.png', img)
            cv2.imwrite(save_dir + 'masks/msks/' + ID + '.png', draw)
            print ID + " written to " + save_dir + " using hand-drawn ground truth."
            break
        elif press == 82:
            C -= 1
            print C
        elif press == 84:
            C += 1
            print C

