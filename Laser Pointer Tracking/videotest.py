import cv2 as cv
import numpy as np
from myfunctions import *


#defining video
cap = cv.VideoCapture(0)

#throw away first frame
_, u = cap.read()
prev_frame = cv.resize(u, (640,640), interpolation = cv.INTER_AREA)

while(1):

    #reading frame
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame. Exiting ...")
        break

    #resizing frame and converting
    frame = cv.resize(frame, (640,640), interpolation = cv.INTER_AREA)

    #perform subtraction
    x = subtract_prev_frame(frame, prev_frame, 69, 0)
    prev_frame = frame
    frame_draw = np.copy(frame)
    
    #finding max
    loc = findMax(x)
 
    #finding max in out coords
    loc_rel = loc - np.array((320,320))
    loc_rel[1] = -loc_rel[1]

    if cv.waitKey(1) == ord('q'):
        break

    #for vertical test: pass loc_rel[1] (vert position) to pid
    #return result of pid to arduino over serial


cap.release
cv.destroyAllWindows()