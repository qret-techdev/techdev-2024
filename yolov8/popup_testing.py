import PySimpleGUI as sg
from popup_window import *

state = 1
loc = (5, 6)
vel = (7, 8)
accel = (9, 10)
# Define the window layout
window = make_window(state, loc, vel, accel)

cap = cv2.VideoCapture(0)

while(1):
    ret, frame = cap.read()
    


