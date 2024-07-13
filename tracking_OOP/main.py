import time
import cv2
from config import DEVICE_NUMBER, PARAMFILE, X_FRAME_SIZE, Y_FRAME_SIZE, SERIAL, KALMAN, SERIAL_PORT, SERIAL_BAUDRATE, PARAMFILE, PIDX, PIDY
from utils import get_pid_params, initialize_video_writer
from tracker import ObjectTracker
from kalman_filter import KalmanFilterManager
from serial_comm import SerialCommunication

def update_speed_and_time(speed, accel, delta_t, prev_time):
    delta_t = time.time() - prev_time
    speed[0] += accel[0] * delta_t
    speed[1] += accel[1] * delta_t
    prev_time = time.time()
    return speed, delta_t, prev_time

def main():
    tracker = ObjectTracker()
    print(tracker.pidx.tunings)
    print(tracker.pidy.tunings)
    kf_manager =  KalmanFilterManager() if KALMAN else None
    serial_comm = SerialCommunication(SERIAL_PORT, SERIAL_BAUDRATE) if SERIAL else None

    cap = cv2.VideoCapture(DEVICE_NUMBER)
    w, h, fps = (int(cap.get(x)) for x in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS))
    result = initialize_video_writer(cap, X_FRAME_SIZE, Y_FRAME_SIZE, fps)

    while True:
        key = cv2.waitKey(1)
        if key in [ord('q'), ord(' ')]:
            if key == ord('q'):
                break
            tracker.sys_state = (tracker.sys_state + 1) % 2
        if key == ord('r'):
            tracker.speed = [0, 0]
        if key == ord('t'):
            tracker.pidy.tunings = get_pid_params(PARAMFILE, PIDX)
            tracker.pidx.tunings = get_pid_params(PARAMFILE, PIDY)
            print(tracker.pidx.tunings)
            print(tracker.pidy.tunings)
    
        success, frame = cap.read()
        if not success:
            break

        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        loc_x_y_unfilt = tracker.process_frame(frame)[1]
        tracker.loc_x_y_filt = kf_manager.apply_filter(loc_x_y_unfilt) if KALMAN else loc_x_y_unfilt
        print(tracker.loc_x_y_filt[0] * 2, tracker.loc_x_y_filt[1] * 2)
        cv2.imshow("Webcam", frame)
        result.write(frame)

        if tracker.sys_state == 0:
            tracker.accel = [0, 0]
            if key == ord('w'):
                tracker.speed[1] += 5
            elif key == ord('s'):
                tracker.speed[1] -= 5
            elif key == ord('a'):
                tracker.speed[0] -= 5
            elif key == ord('d'):
                tracker.speed[0] += 5
            tracker.delta_t = time.time() - tracker.prev_time
            tracker.prev_time = time.time()
        elif tracker.sys_state == 1:
            if tracker.trip_init_guess:
                tracker.speed[0] = tracker.motor_speedx_init_guess
                tracker.speed[1] = tracker.motor_speedy_init_guess
                tracker.trip_init_guess = 0
            tracker.accel[0] = tracker.pidx(tracker.loc_x_y_filt[0] + tracker.t_delay * (tracker.loc_x_y_filt[0] - tracker.prevx) / tracker.delta_t)
            tracker.accel[1] = -tracker.pidy(tracker.loc_x_y_filt[1] + tracker.t_delay * (tracker.loc_x_y_filt[1] - tracker.prevy) / tracker.delta_t)
            tracker.prevx, tracker.prevy = tracker.loc_x_y_filt[0], tracker.loc_x_y_filt[1]
            tracker.speed, tracker.delta_t, tracker.prev_time = update_speed_and_time(tracker.speed, tracker.accel, tracker.delta_t, tracker.prev_time)

        if SERIAL:
            serial_comm.send_speed_to_arduino(tracker.speed)
        print(f'\n State: {tracker.sys_state} | motor_speedx: {tracker.speed[0]:.2f} | motor_speedy: {tracker.speed[1]:.2f} | Accelx: {tracker.accel[0]:.2f} | Accely: {tracker.accel[1]:.2f} | Time Delta {tracker.delta_t:.2f}')

    if SERIAL:
        serial_comm.send_speed_to_arduino([0, 0])

    cap.release()
    result.release()
    cv2.destroyAllWindows()
    if SERIAL:
        serial_comm.close()

if __name__ == "__main__":
    main()
