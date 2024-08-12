import time
import numpy as np
import cv2
import onnxruntime as ort
from simple_pid import PID
from kalman_filter import KalmanFilterManager
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
        self.vel_x = self.vel_y = self.accel_x = self.accel_y = 0.0
        self.center_x = self.center_y = 0.0
        self.prev_time = self.delta_time = 0.0
        self.mov_avg_vel_x = np.zeros(self.config['avg_number'])
        self.mov_avg_vel_y = np.zeros(self.config['avg_number'])
        self.ort_session = ort.InferenceSession(self.config['weights'])
        self.system_state = 'Manual'
        self.tracking_state = 0 
        self.time_not_tracking = 0.0
        self.max_speed = 90
        self.max_accel = 180
        self.prev_center_x = self.prev_center_y = 0
        self.kf_manager = KalmanFilterManager()
        self.config['color'] = tuple(map(int, self.config['color']))
        self.pid_x = PID(*get_params('params.csv', 'pidx'))
        self.pid_y = PID(*get_params('params.csv', 'pidy'))
        self.local_vel_x = self.local_vel_y = 0.0
        self.local_accel_x = self.local_accel_y = 0.0
        self.local_prev_time = None

    def is_debugging_enabled(self, category):
        return self.config.get(f"{category.lower()}_debug", False)

    def add_padding(self, frame, top=0, bottom=0, left=80, right=80, color=[0, 0, 0]):
        return cv2.copyMakeBorder(frame, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)

    def get_speed(self):
        return self.vel_x, self.vel_y
        
    def get_fps(self):
        return 1 / self.delta_time
    
    def get_pid_tunings(self):
        return self.pid_x.tunings, self.pid_y.tunings
    
    def get_center(self):
        return self.center_x, self.center_y

    def reset_tracker(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.time_not_tracking = self.vel_x = self.vel_y = 0
        self.center_x = self.center_y = self.accel_x = self.accel_y = 0
        self.mov_avg_vel_x.fill(0)
        self.mov_avg_vel_y.fill(0)
        self.prev_center_x = self.prev_center_y = 0
        self.local_vel_x = self.local_vel_y = 0.0
        self.local_accel_x = self.local_accel_y = 0.0
        self.local_prev_time = None

    def switch_state(self):
        if self.system_state == 'Manual':
            self.system_state = 'Boost'
        elif self.system_state == 'Boost':
            self.system_state = 'Launch'
        elif self.system_state == 'Launch':
            self.system_state = 'Manual'
    
    def change_speed_x(self, increment):
        self.vel_x += increment
    
    def change_speed_y(self, increment):
        self.vel_y += increment
            
    def update_speed_and_time(self):
        current_time = time.time() 
        self.delta_time = current_time - self.prev_time if self.prev_time != 0 else 0.1
        self.prev_time = current_time

        if self.tracking_state == 0: 
            self.time_not_tracking += self.delta_time

        if self.system_state == 'Launch':
            self.accel_x = self.pid_x(self.center_x + self.config["t_delay"] * (self.center_x - self.prev_center_x) / self.delta_time)
            self.accel_y = -self.pid_y(self.center_y + self.config["t_delay"] * (self.center_y - self.prev_center_y) / self.delta_time)
            self.change_speed_x(self.accel_x * self.delta_time if not self.config['integer'] else int(self.accel_x * self.delta_time))
            self.change_speed_y(self.accel_y * self.delta_time if not self.config['integer'] else int(self.accel_y * self.delta_time))

        self.prev_center_x, self.prev_center_y = self.center_x, self.center_y
        
        if self.system_state != 'Boost':
            if self.is_debugging_enabled("TRACKING"): 
                print("State:", self.system_state)
                print(f'Filtered Center: ({self.center_x:.2f}, {self.center_y:.2f})')
            if self.is_debugging_enabled("FPS"):
                print(f'FPS: {self.get_fps():.2f}')
            if self.is_debugging_enabled("COMMUNICATION"):
                print(f'Speed sent to Arduino: {self.vel_x:.2f}, {self.vel_y:.2f}')

    def rolling_average(self, velocity):
        weights = np.linspace(1, 0, self.config['avg_number'])
        weights /= weights.sum()
        self.mov_avg_vel_x = np.roll(self.mov_avg_vel_x, -1)
        self.mov_avg_vel_y = np.roll(self.mov_avg_vel_y, -1)
        self.mov_avg_vel_x[-1] = velocity[0]
        self.mov_avg_vel_y[-1] = velocity[1]
        return np.dot(self.mov_avg_vel_x, weights), np.dot(self.mov_avg_vel_y, weights)

    def process_center(self, center_x_unfiltered, center_y_unfiltered):
        current_time = time.time()
        delta_time = current_time - self.local_prev_time if self.local_prev_time else 0.1
        self.local_prev_time = current_time
        
        if self.tracking_state:
            center_offset = (center_x_unfiltered, center_y_unfiltered) - np.array([self.config['frame_width'] / 2, self.config['frame_height'] / 2])
            self.center_x, self.center_y = self.kf_manager.apply_filter(center_offset) if self.config['kalman'] else center_offset

            self.local_vel_x, self.local_vel_y = self.rolling_average(((self.center_x - self.prev_center_x) / delta_time, (self.center_y - self.prev_center_y) / delta_time))
            self.local_accel_x = (self.local_vel_x - self.local_vel_x) / delta_time
            self.local_accel_y = (self.local_vel_y - self.local_vel_y) / delta_time
        else:
            # Use previous velocity and acceleration to estimate position if detection is lost
            x = self.prev_center_x + self.local_vel_x * delta_time + 0.5 * self.local_accel_x * (delta_time ** 2)
            y = self.prev_center_y + self.local_vel_y * delta_time + 0.5 * self.local_accel_y * (delta_time ** 2)
            self.center_x, self.center_y = self.kf_manager.apply_filter((x, y)) if self.config['kalman'] else (x, y)
        
        self.update_speed_and_time()

    def cleanup(self, serial_comm, cap, result):
        if serial_comm:
            serial_comm.send_speed_to_arduino([0, 0])
            serial_comm.close()
        cap.release()
        result.release()
        cv2.destroyAllWindows()

    def annotate_frame(self, frame, box, confidence):
        x1, y1, x2, y2 = map(int, box[:4])
        label = f"{confidence:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), self.config['color'], 2)
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.config['color'], 2)
        
    def preprocess(self, frame):
        img = cv2.resize(frame, (640, 640))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.transpose(2, 0, 1)
        return img[np.newaxis, :, :, :].astype(np.float32) / 255.0

    def postprocess(self, outputs):
        output_array = outputs[0]
        if output_array.ndim == 2:
            output_array = output_array.transpose(1, 0)
        elif output_array.ndim == 4:
            output_array = output_array[0].transpose(1, 2, 0).reshape(-1, output_array.shape[1])
        
        boxes = []
        for i in range(output_array.shape[0]):
            if len(output_array[i, 4]) > 0:
                idx = np.argmax(output_array[i, 4])
                confidence = output_array[i, 4][idx]
                x_center, y_center, width, height = output_array[i, :4, idx]
                
                if confidence >= self.config['confidence']:
                    x1, y1, x2, y2 = x_center - (0.5 * width), y_center - (0.5 * height), x_center + (0.5 * width), y_center + (0.5 * height)
                    boxes.append([x1, y1, x2, y2, confidence])
        
        if len(boxes) > 0:
            boxes = np.array(boxes, dtype=float)
            self.tracking_state = 1
        else:
            boxes = np.zeros((0, 5), dtype=float)
            self.tracking_state = 0

        self.process_center(x_center, y_center)

        return boxes

    def process_frame(self, frame):
        input_tensor = self.preprocess(frame)
        outputs = self.ort_session.run(None, {'images': input_tensor})
        boxes = self.postprocess(outputs)

        if len(boxes) > 0:
            self.annotate_frame(frame, boxes[0], boxes[0][4])

        return frame
