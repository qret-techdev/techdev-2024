import serial
import time

ser = serial.Serial('COM10', 115200)

speed1 = 45

while(1):
    ser.write(f"{speed1:.2f}".encode())