import time
import cv2
from config import DEVICE_NUMBER, PARAMFILE, X_FRAME_SIZE, Y_FRAME_SIZE, SERIAL, KALMAN, SERIAL_PORT, SERIAL_BAUDRATE, PARAMFILE, PIDX, PIDY, CONFIDENCE
from utils import get_pid_params, initialize_video_writer
from tracker import ObjectTracker
from kalman_filter import KalmanFilterManager
from serial_comm import SerialCommunication

def update_speed_and_time(speed, accel, delta_t):
    speed[0] += accel[0] * delta_t
    speed[1] += accel[1] * delta_t
    return speed

def main():
    tracker = ObjectTracker()
    tracker.pidx.tunings = get_pid_params(PARAMFILE, PIDX)
    tracker.pidy.tunings = get_pid_params(PARAMFILE, PIDY)
    kf_manager =  KalmanFilterManager() if KALMAN else None
    serial_comm = SerialCommunication(SERIAL_PORT, SERIAL_BAUDRATE) if SERIAL else None

    cap = cv2.VideoCapture(DEVICE_NUMBER)
    w, h, fps = (int(cap.get(x)) for x in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS))
    result = initialize_video_writer(cap, X_FRAME_SIZE, Y_FRAME_SIZE, fps)

    # Define padding parameters (example: 50 pixels padding on each side)
    top, bottom, left, right = 0, 0, 0 ,0#80, 80
    padding_color = [0, 0, 0]  # Padding color (black)

    while True:
        key = cv2.waitKey(1)
        if key in [ord('q'), ord(' ')]:
            if key == ord('q'):
                break
            tracker.pidx.reset()
            tracker.pidy.reset()
            tracker.speed = [0, 0]
            tracker.sys_state = (tracker.sys_state + 1) % 2
        if key == ord('r'):
            tracker.speed = [0, 0]
        if key == ord('t'):
            tracker.pidx.tunings = get_pid_params(PARAMFILE, PIDX)
            tracker.pidy.tunings = get_pid_params(PARAMFILE, PIDY)
            print(f'x-tuning: {tracker.pidx.tunings}')
            print(f'y-tuning: {tracker.pidy.tunings}')
    
        success, frame = cap.read()
        if not success:
            break

        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        # Add padding to the image
        padded_frame = cv2.copyMakeBorder(frame, top, bottom, left, right, cv2.BORDER_CONSTANT, value=padding_color)
        
        loc_x_y_unfilt = tracker.process_frame(padded_frame, CONFIDENCE)[1]
        print(loc_x_y_unfilt[0], loc_x_y_unfilt[1])
        tracker.loc_x_y_filt = kf_manager.apply_filter(loc_x_y_unfilt) if KALMAN else loc_x_y_unfilt
        print(tracker.loc_x_y_filt[0], tracker.loc_x_y_filt[1])
        cv2.imshow("Webcam", padded_frame)
        result.write(padded_frame)

        if tracker.sys_state == 0:
            tracker.frame_count, tracker.box_count = (0,0)
            tracker.count = False
            tracker.accel = [0, 0]
            if key == ord('w'):
                tracker.speed[1] += 3
            elif key == ord('s'):
                tracker.speed[1] -= 3
            elif key == ord('a'):
                tracker.speed[0] -= 3
            elif key == ord('d'):
                tracker.speed[0] += 3
            tracker.delta_t = time.time() - tracker.prev_time
            tracker.prev_time = time.time()
            tracker.accel[0] = tracker.pidx(tracker.loc_x_y_filt[0] + tracker.t_delay * (tracker.loc_x_y_filt[0] - tracker.prevx) / tracker.delta_t)
            tracker.accel[1] = -tracker.pidy(tracker.loc_x_y_filt[1] + tracker.t_delay * (tracker.loc_x_y_filt[1] - tracker.prevy) / tracker.delta_t)
            tracker.prevx, tracker.prevy = tracker.loc_x_y_filt[0], tracker.loc_x_y_filt[1]
        elif tracker.sys_state == 1:
            tracker.count = True
            if tracker.trip_init_guess:
                tracker.speed[0] = tracker.motor_speedx_init_guess
                tracker.speed[1] = tracker.motor_speedy_init_guess
                tracker.trip_init_guess = 0
            tracker.accel[0] = tracker.pidx(tracker.loc_x_y_filt[0] + tracker.t_delay * (tracker.loc_x_y_filt[0] - tracker.prevx) / tracker.delta_t)
            tracker.accel[1] = -tracker.pidy(tracker.loc_x_y_filt[1] + tracker.t_delay * (tracker.loc_x_y_filt[1] - tracker.prevy) / tracker.delta_t)
            tracker.prevx, tracker.prevy = tracker.loc_x_y_filt[0], tracker.loc_x_y_filt[1]
            tracker.delta_t = time.time() - tracker.prev_time
            tracker.prev_time = time.time()
            tracker.speed = update_speed_and_time(tracker.speed, tracker.accel, tracker.delta_t)
            tracker.speed[0]=0
        if SERIAL:
            serial_comm.send_speed_to_arduino(tracker.speed)
        print(f'\n State: {tracker.sys_state} | motor_speedx: {tracker.speed[0]:.2f} | motor_speedy: {tracker.speed[1]:.2f} | Accelx: {tracker.accel[0]:.2f} | Accely: {tracker.accel[1]:.2f} | Time Delta {tracker.delta_t:.2f}')
    if(tracker.frame_count != 0):
        print(f'\nPercentage of Rocket Tracked = %{100*(tracker.box_count/tracker.frame_count):.2f} \nTotal Frames = {tracker.frame_count} \nBoundboxes = {tracker.box_count}')
    if SERIAL:
        serial_comm.send_speed_to_arduino([0, 0])

    cap.release()
    result.release()
    cv2.destroyAllWindows()
    if SERIAL:
        serial_comm.close()

if __name__ == "__main__":
    main()
