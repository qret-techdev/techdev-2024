import datetime
import cv2
import pandas as pd
import os

def initialize_video_writer(cap, width, height, fps):
    os.makedirs('output_video', exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"output_video/output_{timestamp}.mp4"
    return cv2.VideoWriter(filename, fourcc, fps, (width, height))

def get_pid_params(filename, param):
    df = pd.read_excel(filename, engine='openpyxl')
    Kp = float(df.loc[0, param])
    Ki = float(df.loc[1, param])
    Kd = float(df.loc[2, param])
    return Kp, Ki, Kd
