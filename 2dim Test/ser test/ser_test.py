import time
import serial
import numpy as np

#opening serial port with arduino
ser = serial.Serial('COM10', 115200) #might have to change com number, ex 'COM11'... best to keep a high baud rate, make sure it matches w/ arduino
time.sleep(1)

xs = 1*np.array([2, 4, 8, 15, 20, 25, 30, 40, 50, 60, 65, 75, 75, 80, 85, 90, 90, 90, 80, 70, 60, 50, 45, 40, 30, 15, 5, 0])
ys = np.ones(len(xs))

for i in range(len(xs)):
    print(xs[i], ys[i])
    #serial - sending speeds to arduino
    print(f'{xs[i]:.2f} {ys[i]:.2f}\n')
    ser.write(f'{xs[i]:.2f} {ys[i]:.2f}\n'.encode()) #\n is absolutely necessary!!!
    #ser.write(f'\n'.encode()) #ON ARDUINO SIDE NEEDS TO HAVE SPACE BETWEEN, HASN'T BEEN TESTED
    ser.flushInput()
    ser.flushOutput()
    time.sleep(0.25)