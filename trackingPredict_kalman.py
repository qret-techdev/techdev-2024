#imporing libraries
import numpy as np
from collections import defaultdict
import cv2 #getting video, monitoring key presses while in video window
from simple_pid import PID #control system pid
import time #measure time, calculate speed from accel and delta t
import serial #communication with arduino

from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors

AVG_NUMBER = 3
DEVICE_NUMBER = 1
Y_FRAME_SIZE = 640
X_FRAME_SIZE = 480
CUDA = 1
SERIAL = 0

if(SERIAL):
  ser = serial.Serial('COM5', 115200) #might have to change com number, ex 'COM11'... best to keep a high baud rate, make sure it matches w/ arduino