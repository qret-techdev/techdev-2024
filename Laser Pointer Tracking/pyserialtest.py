import serial
import time

ser = serial.Serial('COM9', 115200)

speeds = [-180, 180]

temp = 1

while(1):
    if(temp == 1):
        message = speeds[0]
        temp = 0
    else:
        message = speeds[1]
        temp = 1

    ser.write(f"{message:.2f}\n".encode())

    print(message)

    time.sleep(1)