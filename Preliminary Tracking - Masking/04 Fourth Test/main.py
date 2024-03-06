import cv2 as cv
import numpy as np
import time

from functions import *

#define how many sky images will be processed
num_imgs = 4

#read sky images with file names "sky_n" where n > 0 to num_imgs
images = readImgs(num_imgs)

#create array of hsv values for all images
blues = [None] * num_imgs
for i in range(num_imgs):
    print(i)
    blues[i] = getBlueRange(images[i])

#defining arrays to store both min and max hsv values
h = [[0 for x in range(num_imgs)] for y in range(2)]
s = [[0 for x in range(num_imgs)] for y in range(2)]
v = [[0 for x in range(num_imgs)] for y in range(2)]

for i in range(num_imgs):
    #storing mins
    h[0][i] = blues[i][0][0]
    s[0][i] = blues[i][0][1]
    v[0][i] = blues[i][0][2]

    #storing maxes
    h[1][i] = blues[i][1][0]
    s[1][i] = blues[i][1][1]
    v[1][i] = blues[i][1][2]

#defining range to mask
lower_blue = np.array([min(h[0]), min(s[0]), min(v[0])])
upper_blue = np.array([max(h[1]), max(s[1]), max(v[1])])

#open video
cap = cv.VideoCapture("C:\\Users\\Teighin Nordholt\\Desktop\\Tech Dev Reading Break\\04 Fourth Test\\1.mp4")

while(cap.isOpened()):
    ret, frame = cap.read()

    frame = cv.resize(frame, (960,540), interpolation = cv.INTER_AREA)

    #ret is false if read correctly
    if not ret:
        print("Can't receive frame. Exiting ...")
        break

    blueMask = maskBlue(frame, lower_blue, upper_blue)

    top = getMaskHighest(blueMask)

    cv.circle(frame, top, 5, (0, 0, 255), 2)

    #print mask
    #cv.imshow("Output", frame)
    cv.imshow("Blue Mask", blueMask)
    cv.imshow("Line Mask", drawLines(blueMask))

    if cv.waitKey(1) == ord('q'):
        break

    time.sleep(0.01)

cap.release
cv.destroyAllWindows()