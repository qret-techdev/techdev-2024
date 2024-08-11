import sys
import cv2
from utils import get_params, initialize_video_writer
from tracker import SingleObjectTracker
from serial_comm import SerialCommunication

def main():
  # Load parameters from the CSV file
  
  tracker = SingleObjectTracker()
  
  serial_comm = SerialCommunication(tracker.config['port'], tracker.config['baudrate']) if tracker.config['serial_en'] == 1 else None

  cap = cv2.VideoCapture(tracker.config['device'])
  result = initialize_video_writer(cap, tracker.config['frame_width'], tracker.config['frame_height'], 30)
  
  launch_mode = True
  velocity_x, velocity_y = (0, 0)
  
  if tracker.is_debugging_enabled("TRACKING"):
    x_tunings, y_tunings = tracker.get_pid_tunings()
    print("X PID: ", x_tunings)
    print("Y PID: ", y_tunings) 

  while True:
    key = cv2.waitKey(1)
    if key in [ord(' ')]:
      tracker.reset_tracker()
      tracker.switch_state()
        
    if key == ord('q'):
        break

    if key == ord('r'):
      tracker.reset_tracker()

    if key == ord('t'):
      tracker.load_params()
      if tracker.is_debugging_enabled("TRACKING"):
        x_tunings, y_tunings = tracker.get_pid_tunings()
        print("X PID: ", x_tunings)
        print("Y PID: ", y_tunings) 
      
    if key == ord('w'):  # Increase speed_y
      tracker.vel_y += 1
    elif key == ord('s'):  # Decrease speed_y
      tracker.vel_y -= 1
    elif key == ord('a'):  # Decrease speed_x
      tracker.vel_x -= 1
    elif key == ord('d'):  # Increase speed_x
      tracker.vel_x += 1
    
    success, frame = cap.read()
    if not success:
      break
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    padded_frame = tracker.add_padding(frame)
    tracker.process_frame(padded_frame)   
    
    fps = 1/tracker.get_fps() if tracker.get_fps() else 0

    cv2.imshow("Webcam", padded_frame)
    result.write(padded_frame)

    velocity_x, velocity_y = tracker.get_speed()

    if(tracker.system_state == 'Manual'):
      launch_mode = True
      
    if(tracker.system_state == 'Boost'):
      
        if tracker.is_debugging_enabled("TRACKING"):
          if(launch_mode):
            print("~~~Launch Booster~~~")
            print("Press or hold W for boost")
            launch_mode = False
        sys.stdout.write(f"Boost Y: {velocity_y}\r")
        sys.stdout.flush()
        if key == ord('\r'):  # Launch
          tracker.switch_state()
          
    elif tracker.system_state == 'Launch':
      
      if tracker.time_not_tracking > tracker.config['stop_tracking']:
        print(f'not tracking for {tracker.time_not_tracking:.2f}s')
        tracker.reset_tracker()
        tracker.switch_state()
        print("Lost Object")
        if serial_comm:
          serial_comm.send_speed_to_arduino((0, 0))
        tracker.config['serial_en'] = False
    
    if tracker.system_state != 'Boost':
      if serial_comm:
        serial_comm.send_speed_to_arduino((velocity_x, velocity_y))

    if tracker.is_debugging_enabled("TRACKING") and tracker.system_state != 'Boost':
      print("State:", tracker.system_state)
      x_center, y_center = tracker.get_center()
      print(f'Filtered Center: ({x_center:.2f}, {y_center:.2f})')
    if tracker.is_debugging_enabled("FPS") and tracker.system_state != 'Boost':
      print(f'FPS: {fps:.2f}')
    
    if tracker.is_debugging_enabled("COMMUNICATION"): 
      
      if tracker.system_state == 'Launch':
        print(f'Speed sent to Arduino: {velocity_x:.2f} {velocity_y:.2f}')
      elif tracker.system_state == 'Manual':
        print(f'Speed sent to Arduino (Manual): {velocity_x:.2f} {velocity_y:.2f}')

  tracker.cleanup(serial_comm, cap, result)

if __name__ == "__main__":
  main()
  