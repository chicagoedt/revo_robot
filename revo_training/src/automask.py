import cv2
import sys, random, string, os

img_dir = 'data/classification/validation/terrain/'
images = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser(img_dir)) for f in fn]

random.shuffle(images)

def getID(size=6, chars=string.ascii_lowercase + string.digits):
    return ''.join(random.choice(chars) for _ in range(size))

for i in images:
    img = cv2.imread(i)
    mask = img.copy() * 0
    if random.random() < 0.2:
        save_dir = 'data/segmentation/validation/'
    else:
        save_dir = 'data/segmentation/training/'
    ID = getID()
    cv2.imwrite(save_dir + 'images/imgs/' + ID + '.png', img)
    cv2.imwrite(save_dir + 'masks/msks/' + ID + '.png', mask)
