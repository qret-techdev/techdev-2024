import time
import numpy as np
import cv2
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors
from simple_pid import PID
from kalman_filter import KalmanFilterManager
from serial_comm import SerialCommunication
from utils import get_params

class SingleObjectTracker:
    def __init__(self):
        self.load_params()
        self.initialize_variables()

    def load_params(self, paramfile='params.csv'):
        self.config = {
            "avg_number": int(get_params(paramfile, 'avg_number')),
            "device_number": int(get_params(paramfile, 'device_number')),
            "frame_height": int(get_params(paramfile, 'frame_height')),
            "frame_width": int(get_params(paramfile, 'frame_width')),
            "confidence": float(get_params(paramfile, 'confidence')),
            "kalman": int(get_params(paramfile, 'kalman')),
            "weights": str(get_params(paramfile, 'weights')),
            "t_delay": float(get_params(paramfile, 't_delay')),
            "tracking_debug": bool(int(get_params(paramfile, 'tracking_debug'))),
            "communication_debug": bool(int(get_params(paramfile, 'communication_debug'))),
            "fps_debug": bool(int(get_params(paramfile, 'fps_debug'))),
            "color": tuple(map(int, get_params(paramfile, 'color'))),  # Assumes color is already a tuple
            "mask_x": float(get_params(paramfile, 'mask_x')),
            "mask_y": float(get_params(paramfile, 'mask_y')),
            "stop_tracking": float(get_params(paramfile, 'stop_tracking')),
            "serial_baudrate": int(get_params(paramfile, 'serial_baudrate')),
            "serial_port": str(get_params(paramfile, 'serial_port')),
            "serial_en": int(get_params(paramfile, 'serial_en')),
            "integer": int(get_params(paramfile, 'integer'))
        }

    def initialize_variables(self):
        self.vel_x = self.vel_y = 0.0
        self.prev_time_center = 0.0
        self.prev_time_speed = 0.0
        self.center_x = self.center_y = 0.0
        self.mov_avg_vel_x = np.zeros(self.config['avg_number'])
        self.mov_avg_vel_y = np.zeros(self.config['avg_number'])
        self.model = YOLO(self.config['weights'])
        self.serial_comm = SerialCommunication(self.config['serial_port'], self.config['serial_baudrate']) if self.config['serial_en'] else None
        self.kf_manager = KalmanFilterManager()
        self.pid_x = PID(*get_params('params.csv', 'pidx'))
        self.pid_y = PID(*get_params('params.csv', 'pidy'))
        self.system_state = 'Manual'
        self.time_not_tracking = 0.0
        self.max_speed = 90
        self.max_accel = 180
        self.prev_center_x = self.prev_center_y = 0
        self.local_vel_x = self.local_vel_y = 0.0
        self.local_accel_x = self.local_accel_y = 0.0

    def is_debugging_enabled(self, category):
        return self.config.get(f"{category.lower()}_debug", False)

    def get_speed(self):
        return self.vel_x, self.vel_y
        
    def get_pid_tunings(self):
        return self.pid_x.tunings, self.pid_y.tunings

    def reset_tracker(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.time_not_tracking = 0
        self.vel_x = self.vel_y = 0.0
        self.mov_avg_vel_x.fill(0)
        self.mov_avg_vel_y.fill(0)
        self.prev_center_x = self.prev_center_y = 0
        self.local_vel_x = self.local_vel_y = 0.0
        self.local_accel_x = self.local_accel_y = 0.0

    def switch_state(self):
        if self.system_state == 'Manual':
            self.system_state = 'Boost'
        elif self.system_state == 'Boost':
            self.system_state = 'Launch'
        elif self.system_state == 'Launch':
            self.system_state = 'Manual'
    
    def increment_x(self, increment):
        self.vel_x += increment
        
    def increment_y(self, increment):
        self.vel_y += increment
            
    def update_speed_and_time(self, center_x, center_y, estimated_vel_x, estimated_vel_y):
        current_time = time.perf_counter()
        delta_time = current_time - self.prev_time_speed
        self.prev_time_speed = current_time
        if self.system_state == 'Launch':
            accel_x = self.pid_x(center_x + self.config["t_delay"] * estimated_vel_x)
            accel_y = -self.pid_y(center_y + self.config["t_delay"] * estimated_vel_y)
            speed_x = (accel_x / delta_time if not self.config['integer'] else int(accel_x / delta_time))
            speed_y = (accel_y / delta_time if not self.config['integer'] else int(accel_y / delta_time))
        else:
            speed_x = self.vel_x
            speed_y = self.vel_y
            
        if self.system_state != 'Boost' and self.serial_comm:
            self.serial_comm.send_speed_to_arduino((speed_x, speed_y))
        
        if self.system_state != 'Boost':
            if self.is_debugging_enabled("TRACKING"): 
                print("State:", self.system_state)
                print(f'Filtered Center: ({center_x:.2f}, {center_y:.2f})')
            if self.is_debugging_enabled("FPS"):
                print(f'FPS: {1/delta_time:.2f}')
            if self.is_debugging_enabled("COMMUNICATION"):
                print(f'Speed sent to Arduino: {speed_x:.2f}, {speed_y:.2f}')

    def rolling_average(self, vel_x, vel_y):
        weights = np.linspace(1, 0, self.config['avg_number'])
        weights /= weights.sum()
        self.mov_avg_vel_x = np.roll(self.mov_avg_vel_x, -1)
        self.mov_avg_vel_y = np.roll(self.mov_avg_vel_y, -1)
        self.mov_avg_vel_x[-1] = vel_x
        self.mov_avg_vel_y[-1] = vel_y
        return np.dot(self.mov_avg_vel_x, weights), np.dot(self.mov_avg_vel_y, weights)

    def process_center(self, center_x_unfiltered, center_y_unfiltered):
        center_offset = (center_x_unfiltered, center_y_unfiltered) - np.array([self.config['frame_width'] / 2, self.config['frame_height'] / 2])
        center_x, center_y = self.kf_manager.apply_filter(center_offset) if self.config['kalman'] else center_offset

        # Capture previous velocities for acceleration calculation
        # previous_vel_x, previous_vel_y = self.local_vel_x, self.local_vel_y
        
        current_time = time.perf_counter()
        delta_time = current_time - self.prev_time_center
        self.prev_time_center = current_time
            
        # Calculate new velocity
        self.local_vel_x, self.local_vel_y = self.rolling_average((center_x - self.prev_center_x) / delta_time, (center_y - self.prev_center_y) / delta_time)
            
        # Calculate acceleration based on change in velocity
        # self.local_accel_x = (self.local_vel_x - previous_vel_x) / delta_time
        # self.local_accel_y = (self.local_vel_y - previous_vel_y) / delta_time
                  
        self.prev_center_x, self.prev_center_y = center_x, center_y
        # Update system speed and time
        self.update_speed_and_time(center_x, center_y, self.local_vel_x, self.local_vel_y)
        
    def predict_center(self):
        # Estimate position
        current_time = time.perf_counter()
        delta_time = current_time - self.prev_time_center
        self.prev_time_center = current_time
        
        predicted_x = self.prev_center_x + self.local_vel_x * delta_time #+ 0.5 * self.local_accel_x * (delta_time ** 2)
        predicted_y = self.prev_center_y + self.local_vel_y * delta_time #+ 0.5 * self.local_accel_y * (delta_time ** 2)
        
        center_x, center_y = self.kf_manager.apply_filter((predicted_x, predicted_y)) if self.config['kalman'] else (predicted_x, predicted_y)
        if (center_x, center_y > (400.0, 400.0)):
            # Update previous center position
            center_x, center_y = self.prev_center_x, self.prev_center_y
        else:
            self.prev_center_x, self.prev_center_y = center_x, center_y
        
        # Update system speed and time
        self.update_speed_and_time(center_x, center_y, self.local_vel_x, self.local_vel_y)

    def cleanup(self, cap, result):
        if self.serial_comm:
            self.serial_comm.send_speed_to_arduino([0, 0])
            self.serial_comm.close()
        cap.release()
        result.release()
        cv2.destroyAllWindows()

    def process_frame(self, frame):
        results = self.model.predict(source=frame, verbose=False)
        boxes = results[0].boxes.xyxy
        confs = results[0].boxes.conf
        
        if confs is not None and len(confs) > 0:  # Check if confs is not empty
            index = np.argmax(confs.cpu().numpy())
            annotator = Annotator(frame, line_width=2)
            box = boxes[index]
            conf = confs[index]
            
            center_x, center_y = ((int(box[0] + box[2]) / 2), (int(box[1] + box[3]) / 2))
            if conf >= self.config['confidence']:
                annotator.box_label(box=box, color=(128, 128, 128), label=f"{conf:.2f}")
                self.process_center(center_x, center_y)
            else:
                self.predict_center()
        else:
            # Handle the case where no detections were made
            self.predict_center()
             