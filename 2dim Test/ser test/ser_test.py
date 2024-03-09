import time
import serial

#opening serial port with arduino
ser = serial.Serial('COM9', 115200) #might have to change com number, ex 'COM11'... best to keep a high baud rate, make sure it matches w/ arduino
time.sleep(1)

xs = [45, 90, 180, 45, 90, 180, 360, 10]
ys = [45, 90, 180, 180, 90, 10, 360, 10]

for i in range(len(xs)):
    #serial - sending speeds to arduino
    ser.write(f'{xs[i]:.2f}\n'.encode()) #\n is absolutely necessary!!!
    ser.write(f'{ys[i]:.2f}\n'.encode()) #ON ARDUINO SIDE NEEDS TO HAVE SPACE BETWEEN, HASN'T BEEN TESTED
    ser.flushInput()
    ser.flushOutput()