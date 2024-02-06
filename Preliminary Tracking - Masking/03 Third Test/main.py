import cv2 as cv
import numpy as np
import time


from maskfunctions import getMaskHighest, maskBlue, maskOrange

cap = cv.VideoCapture("C:\\Users\\Teighin Nordholt\\Documents\GitHub\\techdev-2024\\Preliminary Tracking - Masking\\03 Third Test\\1.mp4")

l = 0

while(cap.isOpened()):
    ret, frame = cap.read()

    frame = cv.resize(frame, (960,540), interpolation = cv.INTER_AREA)

    #ret is false if read correctly
    if not ret:
        print("Can't receive frame. Exiting ...")
        break

    blueMask = maskBlue(frame)
    orangeMask = maskOrange(frame)

    top = getMaskHighest(blueMask)

    cv.circle(frame, top, 5, (0, 0, 255), 2)

    #print mask
    cv.imshow("Output", frame)
    #cv.imshow("Blue Mask", blueMask)
    #cv.imshow("Orange Mask", orangeMask)

    if cv.waitKey(1) == ord('q'):
        break

    time.sleep(0.01)

    if(l==0):
        time.sleep(5)
        l+=1

cap.release
cv.destroyAllWindows()