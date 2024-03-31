import PySimpleGUI as sg
import cv2
import numpy as np

def make_window(state, loc, vel, accel):
    layout = [
        [sg.Text("OpenCV Test", size=(60, 1), justification="center")],
        [sg.Image(filename="", key="-IMAGE-")],
        [sg.Button("State", size=(10, 1))],
        [sg.Text(f"State: {state}", size = (60,1), justification="left", key="-STATE-")],
        [sg.Text(f"x: {loc[0]:.2f} y: {loc[1]:.2f}", size=(60, 1), justification="center", key="-XY-")],
        [sg.Text(f"Velocity x: {vel[0]:.2f} Velocity y: {vel[1]:.2f}", size=(60, 1), justification="center", key="-V-") ],
        [sg.Text(f"Accel X: {accel[0]:.2f} Accel Y: {accel[1]:.2f}", size=(60, 1), justification="center", key="-A-")],
        [sg.Button("Exit", size=(10, 1))],
    ]

    # Create the window and show it without the plot
    window = sg.Window("Sunflower Tracking System", layout, location=(800, 400))
    return window

state = 0
def update_window(window, frame, state, loc, vel, accel):
    sg.theme("LightGreen")
    
    event, values = window.read(timeout=20)
    if event == "State":
        state = (state+1)%2
    if event == "Exit" or event == sg.WIN_CLOSED:
        window.close()


    imgbytes = cv2.imencode(".png", frame)[1].tobytes()
    window
    window["-IMAGE-"].update(data=imgbytes)
    window["-STATE-"].update(value=f"State: {state}")
    window["-XY-"].update(value=f"x: {loc[0]:.2f} y: {loc[1]:.2f}")
    window["-V-"].update(value=f"Velocity x: {vel[0]:.2f} Velocity y: {vel[1]:.2f}")
    window["-A-"].update(value=f"Accel X: {accel[0]:.2f} Accel Y: {accel[1]:.2f}")
    return state