import sys
import cv2
from utils import get_params, initialize_video_writer
from tracker import SingleObjectTracker

def handle_keypress(tracker, key):
    """
    Handles keypress events.
    """
    if key == ord(' '):
        tracker.reset_tracker()
        tracker.switch_state()
    elif key == ord('q'):
        return False  # Signal to quit
    elif key == ord('r'):
        tracker.reset_tracker()
    elif key == ord('t'):
        tracker.load_params()
        if tracker.is_debugging_enabled("TRACKING"):
            print_pid_tunings(tracker)
    elif key in [ord('w'), ord('s'), ord('a'), ord('d')]:
        handle_speed_change(tracker, key)
    elif key == ord('\r'): 
<<<<<<< HEAD
<<<<<<< HEAD
        if tracker.system_state == 'Boost' and serial_comm:
            velocity_x, velocity_y = tracker.get_speed()
            print(f"Sending Boost Y: {velocity_y}")
            serial_comm.send_speed_to_arduino((velocity_x, velocity_y))
=======
        if tracker.system_state == 'Boost': #and serial_comm:
            velocity_x, velocity_y = tracker.get_speed()
            print(f"Sending Boost Y: {velocity_y}")
            # serial_comm.send_speed_to_arduino((velocity_x, velocity_y))
>>>>>>> 5ebab150 (Refactor and optimize tracker codebase)
=======
        if tracker.system_state == 'Boost' and tracker.serial_comm:
            velocity_x, velocity_y = tracker.get_speed()
            print(f"Sending Boost Y: {velocity_y}")
            tracker.serial_com.send_speed_to_arduino((velocity_x, velocity_y))
>>>>>>> bf0c73e6 (Cleaned up main)
            tracker.switch_state()
    
    return True

def print_pid_tunings(tracker):
    """
    Prints PID tunings for debugging.
    """
    x_tunings, y_tunings = tracker.get_pid_tunings()
    print("X PID: ", x_tunings)
    print("Y PID: ", y_tunings)

def handle_speed_change(tracker, key):
    """
    Adjusts the speed based on keypress.
    """
    if key == ord('w'):
        tracker.increment_y(1)
    elif key == ord('s'):
        tracker.increment_y(-1)
    elif key == ord('a'):
        tracker.increment_x(-1)
    elif key == ord('d'):
        tracker.increment_x(1)

def main():
    tracker = SingleObjectTracker()

    cap = cv2.VideoCapture(tracker.config['device_number'])
    result = initialize_video_writer(cap, tracker.config['frame_width'], tracker.config['frame_height'], 30)
    launch_mode = True

    while True:
        key = cv2.waitKey(1)
        if not handle_keypress(tracker, key):
            break

        success, frame = cap.read()
        if not success:
            break

        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        padded_frame = cv2.copyMakeBorder(src=frame, top=0, bottom=0, left=80, right=80, borderType=cv2.BORDER_CONSTANT, value=[0, 0, 0])
        tracker.process_frame(padded_frame)
        cv2.imshow("Webcam", padded_frame)
        result.write(padded_frame)

        velocity_x, velocity_y = tracker.get_speed()

        if tracker.system_state == 'Manual':
            launch_mode = True
        elif tracker.system_state == 'Boost':
            if launch_mode and tracker.is_debugging_enabled("TRACKING"):
                print("~~~Launch Booster~~~")
                launch_mode = False
            sys.stdout.write(f"Boost Y: {velocity_y}\r")
            sys.stdout.flush()

    tracker.cleanup(cap, result)

if __name__ == "__main__":
    main()
