import cv2 as cv
import numpy as np
from myfunctions import *
import time

time0 = time.time()
time1 = time.time()
counter = 1

#defining video
cap = cv.VideoCapture("1.mp4")

#throw away first frame
_, u = cap.read()
prev_frame = cv.resize(u, (960,540), interpolation = cv.INTER_AREA)

loc_grad = (480, 270)

start = time.time()
while(cap.isOpened()):
    ret, frame = cap.read()
    #ret is false if read correctly
    if not ret:
        print("Can't receive frame. Exiting ...")
        break

    frame = cv.resize(frame, (960,540), interpolation = cv.INTER_AREA)

    x = subtract_prev_frame(frame, prev_frame, 69, 0)
    prev_frame = frame
    frame_draw = np.copy(frame)
    
    loc = findMax(x)
    o_loc, _ = findHighest(frame)

    #print(loc)

    cv.circle(frame_draw, loc, 5, (0, 0, 255), 2)
    cv.circle(frame_draw, o_loc, 7, (0, 255, 0), 2)

    #print mask
    cv.imshow("Output", frame_draw)
    #cv.imshow("Subtracted", x)
    #cv.imshow("Masked", mask)


    if cv.waitKey(1) == ord('q'):
        break

    #time.sleep(0.01)
print(time.time() - start)
    

cap.release
cv.destroyAllWindows()