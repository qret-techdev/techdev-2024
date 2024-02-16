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
past_5 = [0,0,0,0,0]
past_10 = [0,0,0,0,0,0,0,0,0,0]

#defining video
cap = cv.VideoCapture(0)

#throw away first frame for image subtraction
_, u = cap.read()
prev_frame = cv.resize(u, (640,640), interpolation = cv.INTER_AREA)

#defining pid system, first three are pid constants
pid = PID(0.005, 0, 0.003, setpoint=0)

#opening serial port with arduino
ser = serial.Serial('COM9', 115200)
time.sleep(2)

#defining motor speed
speed = 0
prev_time = time.time()
max_speed = 60 #should be 50ish
max_accel = 150 #should be 60ish

#defining vertical range to consider zero
zero_range = 100

while(1):

    #image subtraction - reading frame
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame. Exiting ...")
        break

    #image subtraction - resizing frame and converting
    frame = cv.resize(frame, (640,640), interpolation = cv.INTER_AREA)
    frame = cv.rotate(frame, cv.ROTATE_90_COUNTERCLOCKWISE)

    #image subtraction - perform subtraction
    x = subtract_prev_frame(frame, prev_frame, 69, 0)
    prev_frame = frame
    frame_draw = np.copy(frame)

    #image subtraction - finding max
    loc = findMax(x)
 
    #converting to our coords with 0,0 at center
    loc_rel = loc - np.array((320,320))
    cv.circle(frame, loc, 5, (0, 255, 0), 2)
    cv.imshow('frame', frame)
    loc_rel[1] = -loc_rel[1]
    #print(loc_rel)

    if(loc_rel[1] > -zero_range and loc_rel[1] < zero_range): #this kinda works lol, not well
        loc_rel[1] = 0

    #exit if press q
    if cv.waitKey(1) == ord('q'):
        break

    #removing first item of moving average array and adding new reading
    past_10.pop(0)
    past_10.append(loc_rel)

    past_5.pop(0)
    past_5.append(loc_rel)

    #computing new average
    #mov_avg = sum(past_10)/10
    mov_avg = sum(past_5)/5
    #print(mov_avg)

    #PID - getting acceleration from pid with average vertical height
    accel = pid(-1*mov_avg[1])

    #confining acceleration
    if(accel > max_accel):
        accel = max_accel
    elif(accel < -max_accel):
        accel = -max_accel

    #PID - calculating new motor speed
    speed += accel * (time.time() - prev_time)
    prev_time = time.time()

    #confining speed
    if(speed > max_speed):
        speed = max_speed
    elif(speed < -max_speed):
        speed = -max_speed

    print(speed)

    #serial - sending speed to arduino
    ser.write(f'{speed:.2f}\n'.encode()) #\n is absolutely necessary!!!

    if cv.waitKey(1) == ord('q'):
        speed = 0
        accel = 0
    
cap.release
cv.destroyAllWindows()