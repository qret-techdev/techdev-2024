
#imporing libraries
import numpy as np
import cv2 #getting video, monitoring key presses while in video window
import time #measure time, calculate speed from accel and delta t
import serial #communication with arduino


#opening serial port with arduino
ser = serial.Serial('COM4', 115200) #might have to change com number, ex 'COM11'... best to keep a high baud rate, make sure it matches w/ arduino
time.sleep(1)

mov_avg_x = np.zeros(5) #THOUGHTS ON SIZE?
mov_avg_y = np.zeros(5)


#AXES: y represents moving the camera 'up and down', x is rotating the entire setup
#defining pid system, first three are pid constants

#defining motor speed
speedy = 0
speedx = 0
accel_x = 0
accel_y = 0
delta_t = 0


prev_time = time.time() #used to find time step

#defining state variable: 0 indicating manual mode with no pid, 1 automatic tracking with ml and pid
sys_state = 0
  


while 1:
      
    speedx = int(input('xspeed'))
    speedy = int(input('yspeed'))



    # serial - sending speeds to arduino
    ser.write(f'{speedx:.2f}\n'.encode()) #\n is absolutely necessary!!!
    ser.write(f'{speedy:.2f}\n'.encode()) #ON ARDUINO SIDE NEEDS TO HAVE SPACE BETWEEN, HASN'T BEEN TESTED
    ser.flushInput()
    ser.flushOutput()

    print(f'\n State: {sys_state} Speedx: {speedx:.2f} | Speedy: {speedy:.2f} | Accelx: {accel_x:.2f} | Accely: {accel_y:.2f} | Time Delta {delta_t:.2f}')
