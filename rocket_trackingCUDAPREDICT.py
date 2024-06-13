#imporing libraries
import numpy as np
from collections import defaultdict
import cv2 #getting video, monitoring key presses while in video window
from simple_pid import PID #control system pid
import time #measure time, calculate speed from accel and delta t
import serial #communication with arduino

from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors

<<<<<<< HEAD
import datetime
=======
"""FOR NOW: ignoring video capture, not sure what it will be handled by. mock functions get_rock_x/y are pseudo and designed around 
  returning the x and y number of pixels from the center of the rocket"""

ser = serial.Serial('COM3', 115200) #might have to change com number, ex 'COM11'... best to keep a high baud rate, make sure it matches w/ arduino
>>>>>>> 95d3c9ef (Slowed down control to make it slightly more bearable)

AVG_NUMBER = 3
DEVICE_NUMBER = 0
Y_FRAME_SIZE = 640
X_FRAME_SIZE = 480
CUDA = 1
SERIAL = 0

if(SERIAL):
  ser = serial.Serial('COM5', 115200) #might have to change com number, ex 'COM11'... best to keep a high baud rate, make sure it matches w/ arduino

def process_center(loc, mov_avg_x, mov_avg_y):
  loc_rel = loc - np.array((X_FRAME_SIZE/2, Y_FRAME_SIZE/2))
        
  #rolling the moving average arrray to get rid of first value
  mov_avg_x = np.roll(mov_avg_x, -1)
  mov_avg_y = np.roll(mov_avg_y, -1)

  #replacing oldest value (moved to end with roll) with the newest
  mov_avg_x[-1] = loc_rel[0]
  mov_avg_y[-1] = -loc_rel[1]

  #averaging the array
  rock_x_filt = (sum(mov_avg_x))/AVG_NUMBER
  rock_y_filt = (sum(mov_avg_y))/AVG_NUMBER

  print(rock_x_filt, rock_y_filt)
  return (rock_x_filt, rock_y_filt)


def process_frame(frame, model, track_history, names, mov_avg_x, mov_avg_y, confidence_threshold=0.2):
    """
    Process a single frame for object detection and tracking.

    Parameters:
    - frame: The current video frame to process.
    - model: The YOLO model used for object detection.
    - track_history: A dictionary maintaining track history for each detected object.
    - names: Class names for detected objects.
    - confidence_threshold: Minimum confidence level required to process a detection.

    Returns:
    - frame: The processed frame with annotations.
    - loc_filt: Filtered location coordinates.
    """
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
  """
  Main function to initialize model and video capture, and process each frame.
  """
  #empty array for moving average filter
  mov_avg_x = np.zeros(AVG_NUMBER) #THOUGHTS ON SIZE?
  mov_avg_y = np.zeros(AVG_NUMBER)


  #AXES: y represents moving the camera 'up and down', x is rotating the entire setup
  #defining pid system, first three are pid constants
  pidx = PID(0.047, 0.0011, 0.10, setpoint=0)
  pidy = PID(0.015, 0.00002, 0.0125, setpoint=0)
  #kd=0.15 seems to be te upper bound. 0.09 seems good

  #defining motor speed
  loc_x_y_filt = [0, 0]
  speed = [0, 0]
  accel = [0, 0]
  delta_t = 0
  delta_t = 0

  #defining variables to predict location
  t_delay = 0.35
  prevx = 0
  prevy = 0
  prevvelx = 0
  prevvey = 0 #we should make vectors for these at some point lol

  #defining tripwire for giving initial vertical motor speed - should only happen once!
  trip_init_guess = 0
  motor_speedy_init_guess = 0 #initial guess for y motor speed - only given once when changing to automatic mode for the first time

  #defining max motor speeds NOT IMPLEMENTED
  max_speed = 90 #should be 50ish
  max_accel = 180 #should be 60ish

  prev_time = time.time() #used to find time step

  #defining state variable: 0 indicating manual mode with no pid, 1 automatic tracking with ml and pid
  sys_state = 0
  track_history = defaultdict(lambda: [])
  model = YOLO("best.pt")
  names = model.model.names

  cap = cv2.VideoCapture(DEVICE_NUMBER)
  # cap.set(cv2.CAP_PROP_FRAME_WIDTH, X_FRAME_SIZE)
  # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Y_FRAME_SIZE)
  

  # Define the codec and create VideoWriter object
  fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for mp4
  w, h, fps = (int(cap.get(x)) for x in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS))

  # Generate a unique filename using the current timestamp
  timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
  filename = f"output_video/output_{timestamp}.mp4"

  result = cv2.VideoWriter(filename,            # File name, 
                      fourcc,                       # codec,
                      fps,                          # fps, 
                      (X_FRAME_SIZE, Y_FRAME_SIZE)) # frame size

  while 1:
      
    key = cv2.waitKey(1)

    if key == ord('q'): #exit if q is pressed
      break

    if key == ord(' '): #toggle state if space is pressed
      sys_state = (sys_state+1)%2

    if key == ord('r'): #reset motor speeds if r is pressed
      speed[0] = 0
      speed[1] = 0    
            
    success, frame = cap.read()
    if not success:
      break
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    processed_frame, loc_x_y_filt = process_frame(frame, model, track_history, names, mov_avg_x, mov_avg_y)
    print(f"\nX: {10*loc_x_y_filt[0]:.2f} | Y: {10*loc_x_y_filt[1]:.2f} | Time: {1000*delta_t:.2f}")
    result.write(processed_frame) 
    cv2.imshow("Webcam", processed_frame)
    
    if(sys_state==0): #keyboard control when in manual mode
      accel[0] = 0
      accel[1] = 0

      if key == ord('w'):
<<<<<<< HEAD
        speed[1] += 5
      elif key == ord('s'):
        speed[0] -= 5
=======
        motor_speedy += 2
      elif key == ord('s'):
        motor_speedy -= 2
>>>>>>> 95d3c9ef (Slowed down control to make it slightly more bearable)
      elif key == ord('a'):
        speed[1] -= 5
      elif key == ord('d'):
        speed[0] += 5

  
      delta_t = time.time()-prev_time
      prev_time = time.time()

    elif(sys_state==1): #doing pid and changing vel if in automatic
     
      #giving intial guess if first time going to automatic state
      if(trip_init_guess==0):
        #speed[1] = 0
        speed[1] = motor_speedy_init_guess
        trip_init_guess += 1

      #getting motor accelerations using predicted location
<<<<<<< HEAD
<<<<<<< HEAD
      accel[0] = pidx(loc_x_y_filt[0] + t_delay*(loc_x_y_filt[0]-prevx)/delta_t)
      accel[1] = -pidy(loc_x_y_filt[1] + t_delay*(loc_x_y_filt[1]-prevy)/delta_t)
=======
      motor_accelx = pidx(loc_x_y_filt[0] + t_delay*(loc_x_y_filt[0]-prevx)/delta_t)
      motor_accely = -pidy(loc_x_y_filt[1] + t_delay*(loc_x_y_filt[1]-prevy)/delta_t)
<<<<<<< HEAD
>>>>>>> b91b372d (fixed pidy calculation)
=======
>>>>>>> 95d3c9ef (Slowed down control to make it slightly more bearable)
=======
      # x = x0 + vt + 1/2at^2
      velx = (loc_x_y_filt[0]-prevx)/delta_t
      vely = (loc_x_y_filt[1]-prevy)/delta_t

      accelx = (velx - prevvelx)/delta_t
      accely = (vely - prevvely)/delta_t

      motor_accelx = pidx(loc_x_y_filt[0] + t_delay*velx + 0.5*accelx*(t_delay**2))
      motor_accely = -pidy(loc_x_y_filt[1] + t_delay*vely + 0.5*accely*(t_delay**2))
>>>>>>> 5d38260c (added second order prediction)

      #updating previous locations
      prevx = loc_x_y_filt[0]
      prevy = loc_x_y_filt[1]

      prevvelx = velx
      prevvely = vely

      #updating speeds
      delta_t = time.time()-prev_time
      speed[0] += accel[0] * delta_t
      speed[1] += accel[1] * delta_t
      prev_time = time.time()

    speed[0]=0
    # serial - sending speeds to arduino
    if(SERIAL):
      ser.write(f'{speed[0]:.2f}\n'.encode()) #\n is absolutely necessary!!!
      #ser.write(f'{0}\n'.encode()) #tis didn't let te x work
      ser.write(f'{speed[1]:.2f}\n'.encode()) #ON ARDUINO SIDE NEEDS TO HAVE SPACE BETWEEN, HAS BEEN TESTED
      ser.flushInput()
      ser.flushOutput()

    print(f'\n State: {sys_state} | motor_speedx: {speed[0]:.2f} | motor_speedy: {speed[1]:.2f} | Accelx: {accel[0]:.2f} | Accely: {accel[1]:.2f} | Time Delta {delta_t:.2f}')

  #setting motors to zero wen we sut off
  speed[0]=0
  speed[1]=0
  
  # serial - sending speeds to arduino
  if(SERIAL):
    ser.write(f'{speed[0]:.2f}\n'.encode()) #\n is absolutely necessary!!!
    ser.write(f'{speed[1]:.2f}\n'.encode()) #ON ARDUINO SIDE NEEDS TO HAVE SPACE BETWEEN, HAS BEEN TESTED
    ser.flushInput()
    ser.flushOutput()  
    
  cap.release()
  result.release()
  cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
