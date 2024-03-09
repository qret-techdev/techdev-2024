"""FOR NOW: ignoring video capture, not sure what it will be handled by. mock functions get_rock_x/y are pseudo and designed around 
   returning the x and y number of pixels from the center of the rocket"""

#imporing libraries
import cv2 as cv #getting video, monitoring key presses while in video window
import numpy as np #some math, array management
from simple_pid import PID #control system pid
import time #measure time, calculate speed from accel and delta t
import serial #communication with arduino
from funcs import *

#empty array for moving average filter
mov_avg_x = np.zeros(5) #THOUGHTS ON SIZE?
mov_avg_y = np.zeros(5)

#AXES: y represents moving the camera 'up and down', x is rotating the entire setup
#defining pid system, first three are pid constants
pidx = PID(0.012, 0.0009, 0.02, setpoint=0) #has been tuned a little bit
pidy = PID(0.012, 0.0009, 0.02, setpoint=0) #copy of the x consts, completely untested

#opening serial port with arduino
#ser = serial.Serial('COM9', 115200) #might have to change com number, ex 'COM11'... best to keep a high baud rate, make sure it matches w/ arduino
time.sleep(1)

#starting video
cap = cv.VideoCapture(0) #number is webcam number, might have to change around
time.sleep(1)

#throw away first frame for image subtraction
_, u = cap.read()
prev_frame = cv.resize(u, (640,640), interpolation = cv.INTER_AREA)

#defining motor speed
speedy = 0
speedx = 0
accel_x = 0
accel_y = 0
delta_t = 0

#defining tripwire for giving initial vertical motor speed - should only happen once!
trip_init_guess = 0
speedy_init_guess = 30 #initial guess for y motor speed - only given once when changing to automatic mode for the first time

#defining max motor speeds NOT IMPLEMENTED
max_speed = 90 #should be 50ish
max_accel = 180 #should be 60ish

prev_time = time.time() #used to find time step

#defining state variable: 0 indicating manual mode with no pid, 1 automatic tracking with ml and pid
sys_state = 0

while(1):

    #read key press
    key = cv.waitKey(1)

    if key == ord('q'): #exit if q is pressed
        break

    if key == ord(' '): #toggle state if space is pressed
        sys_state = (sys_state+1)%2

    if key == ord('r'): #reset motor speeds if r is pressed
        speedx = 0
        speedy = 0

    #image subtraction - reading frame
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame. Exiting ...")
        break

    #image subtraction - resizing frame, converting, and rotating
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

    #reading rocket position
    #rock_x = get_rock_x(frame)
    #rock_y = get_rock_y(frame)
    rock_x = -loc_rel[0]
    rock_y = -loc_rel[1]

    #rolling the moving average arrray to get rid of first value
    mov_avg_x = np.roll(mov_avg_x, -1)
    mov_avg_y = np.roll(mov_avg_y, -1)

    #replacing oldest value (moved to end with roll) with the newest
    mov_avg_x[-1] = rock_x
    mov_avg_y[-1] = rock_y

    #averaging the array
    rock_x_filt = (sum(mov_avg_x))/5
    rock_y_filt = (sum(mov_avg_y))/5

    print(rock_x_filt, rock_y_filt)

    if(sys_state==0): #keyboard control when in manual mode
        accel_x = 0
        accel_y = 0

        if key == ord('w'): #THESE WILL DEPEND ON ORIENTATION OF AXES, UNTESTED
            speedy += 5
        elif key == ord('s'):
            speedy -= 5
        elif key == ord('a'):
            speedx += 5
        elif key == ord('d'):
            speedx -= 5

        delta_t = time.time()-prev_time
        prev_time = time.time()

    elif(sys_state==1): #doing pid and changing vel if in automatic

        #giving intial guess if first time going to automatic state
        if(trip_init_guess==0):
            speedx = 0
            speedy = speedy_init_guess
            trip_init_guess += 1

        #getting motor accelerations
        accel_x = pidx(rock_x_filt)
        accel_y = pidy(rock_y_filt)

        #updating speeds
        delta_t = time.time()-prev_time
        speedx += accel_x * delta_t
        speedy += accel_y * delta_t
        prev_time = time.time()

    #serial - sending speeds to arduino
    #ser.write(f'{speedx:.2f}\n'.encode()) #\n is absolutely necessary!!!
    #ser.write(f'{speedy:.2f}\n'.encode()) #ON ARDUINO SIDE NEEDS TO HAVE SPACE BETWEEN, HASN'T BEEN TESTED
    #ser.flushInput()
    #ser.flushOutput()

    #print(f'\nSpeedx: {speedx:.2f} | Speedy: {speedy:.2f} | Accelx: {accel_x:.2f} | Accely: {accel_y:.2f} | Time Delta {delta_t:.2f}')
