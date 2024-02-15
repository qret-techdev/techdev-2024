#libraries for image subtraction
import cv2 as cv
import numpy as np
from myfunctions import *

#pid and time libraries
from simple_pid import PID
import time

#serial library to communicate with arduino
import serial

#empty array for moving average filter
past_10 = [0,0,0,0,0,0,0,0,0,0]

#defining video
cap = cv.VideoCapture(0)

#throw away first frame for image subtraction
_, u = cap.read()
prev_frame = cv.resize(u, (640,640), interpolation = cv.INTER_AREA)

#defining pid system, first three are pid constants
pid = PID(1, 1, 1, setpoint=0)

#opening serial port with arduino
#ser = serial.Serial('COM10', 115200)
#time.sleep(2)

#defining motor speed
speed = 30
prev_time = time.time()

while(1):

    #image subtraction - reading frame
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame. Exiting ...")
        break

    #image subtraction - resizing frame and converting
    frame = cv.resize(frame, (640,640), interpolation = cv.INTER_AREA)

    #image subtraction - perform subtraction
    x = subtract_prev_frame(frame, prev_frame, 69, 0)
    prev_frame = frame
    frame_draw = np.copy(frame)

    #image subtraction - finding max
    loc = findMax(x)
 
    #converting to our coords with 0,0 at center
    loc_rel = loc - np.array((320,320))
    cv.circle(frame, loc_rel, 5, (0, 0, 255), 2)
    cv.imshow('frame', frame)
    loc_rel[1] = -loc_rel[1]
    #print(loc_rel)

    #exit if press q
    if cv.waitKey(1) == ord('q'):
        break

    #removing first item of moving average array and adding new reading
    past_10.pop(0)
    past_10.append(loc_rel)

    #computing new average
    mov_avg = sum(past_10)/10

    #PID - getting acceleration from pid with average vertical height
    accel = pid(mov_avg[1])

    #PID - calculating new motor speed
    speed += accel * (time.time() - prev_time)
    prev_time = time.time()

    print(speed)

    #serial - sending speed to arduino
    #ser.write(f'{speed:.2f}'.encode())
    
cap.release
cv.destroyAllWindows()