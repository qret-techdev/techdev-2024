import cv2 as cv
import numpy as np
from myfunctions import findHighest
import time

time0 = time.time()
time1 = time.time()
counter = 1

#defining video
cap = cv.VideoCapture("C:\\Users\\Teighin Nordholt\\Desktop\\Tech Dev Reading Break\\02 Second Test\\1.mp4")

while(cap.isOpened()):
    ret, frame = cap.read()

    frame = cv.resize(frame, (960,540), interpolation = cv.INTER_AREA)

    #ret is false if read correctly
    if not ret:
        print("Can't receive frame. Exiting ...")
        break

    top, mask = findHighest(frame)

    cv.circle(frame, top, 5, (0, 0, 255), 2)

    #print mask
    cv.imshow("Output", frame)
    cv.imshow("Masked", mask)

    if cv.waitKey(1) == ord('q'):
        break

    time.sleep(0.01)
    
    

cap.release
cv.destroyAllWindows()