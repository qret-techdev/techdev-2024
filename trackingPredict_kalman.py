#imporing libraries
import numpy as np
from collections import defaultdict
import cv2 #getting video, monitoring key presses while in video window
from simple_pid import PID #control system pid
import time #measure time, calculate speed from accel and delta t
import serial #communication with arduino
import kalman_filter as kf

from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors

import datetime

AVG_NUMBER = 1
DEVICE_NUMBER = 0
Y_FRAME_SIZE = 640
X_FRAME_SIZE = 480
SERIAL = 0
KALMAN = 1
WEIGHTS = "weights/best.pt"

if(SERIAL):
  ser = serial.Serial('COM5', 115200) #might have to change com number, ex 'COM11'... best to keep a high baud rate, make sure it matches w/ arduino

if(KALMAN):
  n_trackables = 1
  ekf = kf.ExampleEKF(n_trackables)

def initialize_mov_avg(size):
    """Parameters:
      - size: The size of the moving average arrays.
      Returns:
      - Two zero-initialized numpy arrays of the given size."""
    return np.zeros(size), np.zeros(size)

def initialize_pid():
    """Returns:
      - Two PID controllers with predefined parameters and setpoints."""
    return PID(0.047, 0.0011, 0.10, setpoint=0), PID(0.0444, 0, 0, setpoint=0)

def initialize_motor_variables():
    """Returns:
      - speed: A list with initial speed values.
      - accel: A list with initial acceleration values.
      - loc_x_y_filt: A list with initial filtered location values.
      - delta_t: Initial time delta.
      - prev_time: Initial previous time.   """
    return [0, 0], [0, 0], [0, 0], [0, 0], 0, 0

def initialize_tracking_variables():
    """Returns:
      - t_delay: Initial time delay.
      - prevx: Initial previous x-coordinate.
      - prevy: Initial previous y-coordinate.
      - velx: Initial velocity in x direction.
      - vely: Initial velocity in y direction.
      - trip_init_guess: Initial trip guess.
      - motor_speedy_init_guess: Initial motor speed guess."""
    return 0.4, 0, 0, 0, 0, 0, 0

def initialize_video_writer(cap, width, height, fps):
    """Parameters:
      - cap: Video capture object.
      - width: Width of the video frame.
      - height: Height of the video frame.
      - fps: Frames per second for the output video.
      Returns:
      - VideoWriter object for saving videos."""
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"output_video/output_{timestamp}.mp4"
    return cv2.VideoWriter(filename, fourcc, fps, (width, height))

def update_speed_and_time(speed, accel, delta_t, prev_time):
    """Parameters:
      - speed: Current speed values.
      - accel: Acceleration values.
      - delta_t: Time delta.
      - prev_time: Previous timestamp.
      Returns:
      - Updated speed values.
      - Updated time delta.
      - Updated previous timestamp."""
    delta_t = time.time() - prev_time
    speed[0] += accel[0] * delta_t
    speed[1] += accel[1] * delta_t
    prev_time = time.time()
    return speed, delta_t, prev_time

def send_speed_to_arduino(speed):
    ser.write(f'{speed[0]:.2f}\n'.encode())
    ser.write(f'{speed[1]:.2f}\n'.encode())
    ser.flushInput()
    ser.flushOutput()

def process_center(loc, mov_avg_x, mov_avg_y):
    """Process the center location for moving average filtering.
      Parameters:
      - loc: Current location.
      - mov_avg_x: Moving average array for x coordinates.
      - mov_avg_y: Moving average array for y coordinates.
      Returns:
      - Filtered x and y coordinates."""
    loc_rel = loc - np.array((X_FRAME_SIZE/2, Y_FRAME_SIZE/2))
    mov_avg_x = np.roll(mov_avg_x, -1)
    mov_avg_y = np.roll(mov_avg_y, -1)
    
    mov_avg_x[-1] = loc_rel[0]
    mov_avg_y[-1] = -loc_rel[1]

    rock_x_filt = (sum(mov_avg_x)) / AVG_NUMBER
    rock_y_filt = (sum(mov_avg_y)) / AVG_NUMBER

    print(rock_x_filt, rock_y_filt)
    return (rock_x_filt, rock_y_filt)

def kalman_filter(loc_x_y_unfilt):
    observations = np.array(loc_x_y_unfilt).reshape(n_trackables, 2, 1)
    ekf.predict()
    ekf.update(observations)
    return ekf.m[:, :, 0].flatten()

def process_frame(frame, model, track_history, names, mov_avg_x, mov_avg_y, confidence_threshold=0.6):
    """Process a single frame for object detection and tracking.
      Parameters:
      - frame: The current video frame to process.
      - model: The YOLO model used for object detection.
      - track_history: A dictionary maintaining track history for each detected object.
      - names: Class names for detected objects.
      - confidence_threshold: Minimum confidence level required to process a detection.
      Returns:
      - frame: The processed frame with annotations.
      - loc_filt: Filtered location coordinates."""
    loc = (0, 0)
    loc_filt = (0, 0)
    results = model.track(frame, persist=True)
    boxes = results[0].boxes.xyxy
    confs = results[0].boxes.conf

    if results[0].boxes.id is not None:
        clss = results[0].boxes.cls.tolist()
        track_ids = results[0].boxes.id.int().tolist()
        annotator = Annotator(frame, line_width=2)

        for box, cls, track_id, conf in zip(boxes, clss, track_ids, confs):
            if conf >= confidence_threshold:
                annotator.box_label(box, color=colors(int(cls), True), label=f"{names[int(cls)]} {conf:.2f}")
                track = track_history[track_id]
                loc = (((box[0] + box[2]) / 2).cpu().numpy(), ((box[1] + box[3]) / 2).cpu().numpy())
                loc_filt = process_center(loc, mov_avg_x, mov_avg_y)
                track.append((int(loc[0]), int(loc[1])))
                if len(track) > 30:
                    track.pop(0)
                points = np.array(track, dtype=np.int32).reshape((-1, 1, 2))
                cv2.circle(frame, track[-1], 7, colors(int(cls), True), -1)
                cv2.polylines(frame, [points], isClosed=False, color=colors(int(cls), True), thickness=2)

    return frame, loc_filt

def main():
    mov_avg_x, mov_avg_y = initialize_mov_avg(AVG_NUMBER)
    pidx, pidy = initialize_pid()
    speed, accel, loc_x_y_unfilt, loc_x_y_filt, delta_t, prev_time = initialize_motor_variables()
    t_delay, prevx, prevy, velx, vely, trip_init_guess, motor_speedy_init_guess = initialize_tracking_variables()
    max_speed = 90
    max_accel = 180
    sys_state = 0
    track_history = defaultdict(lambda: [])
    model = YOLO(WEIGHTS)
    names = model.model.names
    cap = cv2.VideoCapture(DEVICE_NUMBER)
    w, h, fps = (int(cap.get(x)) for x in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS))
    result = initialize_video_writer(cap, X_FRAME_SIZE, Y_FRAME_SIZE, fps)

    while True:
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        if key == ord(' '):
            sys_state = (sys_state + 1) % 2
        if key == ord('r'):
            speed = [0, 0]

        success, frame = cap.read()
        if not success:
            break
        
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        loc_x_y_unfilt = process_frame(frame, model, track_history, names, mov_avg_x, mov_avg_y)[1]
        if KALMAN:
            loc_x_y_filt = kalman_filter(loc_x_y_unfilt)
        else:
            loc_x_y_filt = loc_x_y_unfilt
        print(loc_x_y_filt[0], loc_x_y_filt[1])
        cv2.imshow("Webcam", frame)
        result.write(frame)

        if sys_state == 0:
            accel = [0, 0]
            if key == ord('w'):
                speed[1] += 5
            elif key == ord('s'):
                speed[0] -= 5
            elif key == ord('a'):
                speed[1] -= 5
            elif key == ord('d'):
                speed[0] += 5
            delta_t = time.time() - prev_time
            prev_time = time.time()

        elif sys_state == 1:
            if trip_init_guess == 0:
                speed[1] = motor_speedy_init_guess
                trip_init_guess += 1
            accel[0] = pidx(loc_x_y_filt[0] + t_delay * (loc_x_y_filt[0] - prevx) / delta_t)
            accel[1] = -pidy(loc_x_y_filt[1] + t_delay * (loc_x_y_filt[1] - prevy) / delta_t)
            prevx, prevy = loc_x_y_filt[0], loc_x_y_filt[1]
            speed, delta_t, prev_time = update_speed_and_time(speed, accel, delta_t, prev_time)

        if SERIAL:
            send_speed_to_arduino(speed)
        print(f'\n State: {sys_state} | motor_speedx: {speed[0]:.2f} | motor_speedy: {speed[1]:.2f} | Accelx: {accel[0]:.2f} | Accely: {accel[1]:.2f} | Time Delta {delta_t:.2f}')

    if SERIAL:
        send_speed_to_arduino([0, 0])

    cap.release()
    result.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()