#imporing libraries
import numpy as np
from collections import defaultdict
import cv2 #getting video, monitoring key presses while in video window
from simple_pid import PID #control system pid
import time #measure time, calculate speed from accel and delta t
import serial #communication with arduino

from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors

"""FOR NOW: ignoring video capture, not sure what it will be handled by. mock functions get_rock_x/y are pseudo and designed around 
   returning the x and y number of pixels from the center of the rocket"""


def process_frame(frame, model, track_history, names):
  """
  Process a single frame for object detection and tracking.

  Parameters:
  - frame: The current video frame to process.
  - model: The YOLO model used for object detection.
  - track_history: A dictionary maintaining track history for each detected object.
  - names: Class names for detected objects.

  Returns:
  - frame: The processed frame with annotations.
  """
  loc = (0, 0)
  results = model.track(frame, persist=True)
  boxes = results[0].boxes.xyxy.cpu()

  if results[0].boxes.id is not None:
    clss = results[0].boxes.cls.cpu().tolist()
    track_ids = results[0].boxes.id.int().cpu().tolist()

    annotator = Annotator(frame, line_width=2)

    for box, cls, track_id in zip(boxes, clss, track_ids):
      annotator.box_label(box, color=colors(int(cls), True), label=names[int(cls)])

      track = track_history[track_id]
      loc = (((box[0] + box[2]) / 2).numpy(), ((box[1] + box[3]) / 2).numpy())
      track.append((int(loc[0]), int(loc[1])))
      if len(track) > 30:
        track.pop(0)

      points = np.array(track, dtype=np.int32).reshape((-1, 1, 2))
      cv2.circle(frame, track[-1], 7, colors(int(cls), True), -1)
      cv2.polylines(frame, [points], isClosed=False, color=colors(int(cls), True), thickness=2)
            
  return frame, loc

def main():
  """
  Main function to initialize model and video capture, and process each frame.
  """
  
  #empty array for moving average filter
  mov_avg_x = np.zeros(5) #THOUGHTS ON SIZE?
  mov_avg_y = np.zeros(5)

  #AXES: y represents moving the camera 'up and down', x is rotating the entire setup
  #defining pid system, first three are pid constants
  pidx = PID(0.012, 0.0009, 0.02, setpoint=0) #has been tuned a little bit
  pidy = PID(0.012, 0.0009, 0.02, setpoint=0) #copy of the x consts, completely untested

  #defining motor speed
  speedy = 0
  speedx = 0
  accel_x = 0
  accel_y = 0
  delta_t = 0

  #defining tripwire for giving initial vertical motor speed - should only happen once!
  trip_init_guess = 0
  speedy_init_guess = 30 #initial guess for y motor speed - only given once when changing to automatic mode for the first time

  #defining max motor speeds NOT IMPLEMENTED
  max_speed = 90 #should be 50ish
  max_accel = 180 #should be 60ish

  prev_time = time.time() #used to find time step

  #defining state variable: 0 indicating manual mode with no pid, 1 automatic tracking with ml and pid
  sys_state = 0
  track_history = defaultdict(lambda: [])
  model = YOLO("best.pt")
  names = model.model.names

  cap = cv2.VideoCapture(0)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
  assert cap.isOpened(), "Error reading video file"

  while cap.isOpened():
      
    key = cv2.waitKey(1)

    if key == ord('q'): #exit if q is pressed
      break

    if key == ord(' '): #toggle state if space is pressed
      sys_state = (sys_state+1)%2

    if key == ord('r'): #reset motor speeds if r is pressed
      speedx = 0
      speedy = 0    
            
    success, frame = cap.read()
    if not success:
      break
    # frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    processed_frame, loc_x_y = process_frame(frame, model, track_history, names)
    cv2.imshow("Webcam", processed_frame)
    loc_rel = loc_x_y - np.array((320, 240))
    rock_x = -loc_rel[0]
    rock_y = -loc_rel[1]
        
    #rolling the moving average arrray to get rid of first value
    mov_avg_x = np.roll(mov_avg_x, -1)
    mov_avg_y = np.roll(mov_avg_y, -1)

    #replacing oldest value (moved to end with roll) with the newest
    mov_avg_x[-1] = rock_x
    mov_avg_y[-1] = rock_y

    #averaging the array
    rock_x_filt = (sum(mov_avg_x))/5
    rock_y_filt = (sum(mov_avg_y))/5

    print(rock_x_filt, rock_y_filt)
        
    if(sys_state==0): #keyboard control when in manual mode
      accel_x = 0
      accel_y = 0

      if key == ord('w'): #THESE WILL DEPEND ON ORIENTATION OF AXES, UNTESTED
        speedy += 5
      elif key == ord('s'):
        speedy -= 5
      elif key == ord('a'):
        speedx += 5
      elif key == ord('d'):
        speedx -= 5

      delta_t = time.time()-prev_time
      prev_time = time.time()

    elif(sys_state==1): #doing pid and changing vel if in automatic
     
      #giving intial guess if first time going to automatic state
      if(trip_init_guess==0):
        speedx = 0
        speedy = speedy_init_guess
        trip_init_guess += 1

      #getting motor accelerations
      accel_x = pidx(rock_x_filt)
      accel_y = pidy(rock_y_filt)
      #updating speeds
      delta_t = time.time()-prev_time
      speedx += accel_x * delta_t
      speedy += accel_y * delta_t
      prev_time = time.time()

    #serial - sending speeds to arduino
    #ser.write(f'{speedx:.2f}\n'.encode()) #\n is absolutely necessary!!!
    #ser.write(f'{speedy:.2f}\n'.encode()) #ON ARDUINO SIDE NEEDS TO HAVE SPACE BETWEEN, HASN'T BEEN TESTED
    #ser.flushInput()
    #ser.flushOutput()

    #print(f'\nSpeedx: {speedx:.2f} | Speedy: {speedy:.2f} | Accelx: {accel_x:.2f} | Accely: {accel_y:.2f} | Time Delta {delta_t:.2f}')
    #read key press

  cap.release()
  cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
