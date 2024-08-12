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
            "weights": get_params(paramfile, 'weights'),
            "t_delay": float(get_params(paramfile, 't_delay')),
            "tracking_debug": int(get_params(paramfile, 'tracking_debug')),
            "communication_debug": int(get_params(paramfile, 'communication_debug')),
            "fps_debug": int(get_params(paramfile, 'fps_debug')),
            "color": np.array(get_params(paramfile, 'color')),
            "mask_x": float(get_params(paramfile, 'mask_x')),
            "mask_y": float(get_params(paramfile, 'mask_y')),
            "stop_tracking": float(get_params(paramfile, 'stop_tracking')),
            "baudrate": int(get_params(paramfile, 'serial_baudrate')),
            "port": str(get_params(paramfile, 'serial_port')),
            'device': int(get_params(paramfile, 'device_number')),
            "serial_en": int(get_params(paramfile, 'serial_en')),
            "integer": int(get_params(paramfile, 'integer'))
        }

    def initialize_variables(self):
        self.vel_x = 0
        self.vel_y = 0
        self.accel_x = 0
        self.accel_y = 0
        self.center_x, self.center_y = (0, 0)
        self.prev_time = 0
        self.delta_time = 0
        self.mov_avg_vel_x = np.zeros(self.config['avg_number'])
        self.mov_avg_vel_y = np.zeros(self.config['avg_number'])
        self.ort_session = ort.InferenceSession(self.config['weights'])
        self.system_state = 'Manual'
        self.tracking_state = 0 
        self.prev_tracking_state = 0
        self.time_not_tracking = 0
        self.prev_tracking_time = 0
        self.max_speed = 90
        self.max_accel = 180
        self.prev_center_x = 0
        self.prev_center_y = 0
        self.counter = 1
        self.frame_counter = 0
        self.rocket_counter = 0
        self.kf_manager = KalmanFilterManager()
        color_r, color_g, color_b = self.config['color']
        self.config['color'] = (int(color_r), int(color_g), int(color_b))
        self.pid_x = PID()
        self.pid_y = PID()
        self.pid_x.tunings = get_params('params.csv', 'pidx')
        self.pid_y.tunings = get_params('params.csv', 'pidy')
        
        # Initialize local variables for process_center function
        self.local_prev_time = None
        self.local_vel_x = 0.0
        self.local_vel_y = 0.0
        self.local_accel_x = 0.0
        self.local_accel_y = 0.0
        
    def is_debugging_enabled(self, category):
        """
        Checks if a specific debugging category is enabled.
        
        Args:
            category (str): The name of the debugging category to check.
        
        Returns:
            bool: True if the category is enabled, False otherwise.
        """
        debug_key = f"{category.lower()}_debug"
        return self.config.get(debug_key, False)

    def add_padding(self, frame, top=0, bottom=0, left=80, right=80, color=[0, 0, 0]):
        """
        Adds padding to the frame.
        
        Args:
            frame: The input frame to which padding will be added.
            top: The padding in pixels to add on the top side.
            bottom: The padding in pixels to add on the bottom side.
            left: The padding in pixels to add on the left side.
            right: The padding in pixels to add on the right side.
            color: The padding color (default is black).
        
        Returns:
            padded_frame: The frame with padding added.
        """
        padded_frame = cv2.copyMakeBorder(frame, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
        return padded_frame

    def get_speed(self):
        return self.vel_x, self.vel_y
        
    def get_fps(self):
        return 1/self.delta_time
    
    def get_pid_tunings(self):
        return self.pid_x.tunings, self.pid_y.tunings
    
    def get_center(self):
        return self.center_x, self.center_y

    def reset_tracker(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.time_not_tracking = 0
        self.vel_x = 0
        self.vel_y = 0
        self.center_x = 0
        self.center_y = 0
        self.accel_x = 0
        self.accel_y = 0
        self.mov_avg_vel_x = np.zeros(self.config['avg_number'])
        self.mov_avg_vel_y = np.zeros(self.config['avg_number'])
        self.prev_center_x = 0
        self.prev_center_y = 0
        self.local_prev_time = None
        self.local_vel_x = 0.0
        self.local_vel_y = 0.0
        self.local_accel_x = 0.0
        self.local_accel_y = 0.0

    def switch_state(self):
        if self.system_state == 'Manual':
            self.system_state = 'Boost'
        elif self.system_state == 'Boost':
            self.system_state = 'Launch'
        elif self.system_state == 'Launch':
            self.system_state = 'Manual'
            
    def update_speed_and_time(self):
        current_time = time.time() 
        if self.prev_time != 0:
            self.delta_time = current_time - self.prev_time
        else:
            self.delta_time = 0.1  # Small initial delta_time to prevent division by zero

        if self.tracking_state == 0:  # When not tracking
            if self.prev_tracking_time == 0:  # If this is the first time we're not tracking
                self.prev_tracking_time = current_time  # Initialize it to current time
            self.time_not_tracking = current_time - self.prev_tracking_time
        else:  # When tracking
            self.time_not_tracking = 0
            self.prev_tracking_time = 0
        self.prev_tracking_time = current_time 
         
        # Only update speed if system_state is 'Launch'
        if self.system_state == 'Launch':
            self.accel_x = self.pid_x(self.center_x + self.config["t_delay"] * (self.center_x - self.prev_center_x) / self.delta_time)
            self.accel_y = -self.pid_y(self.center_y + self.config["t_delay"] * (self.center_y - self.prev_center_y) / self.delta_time)
            self.vel_x += self.accel_x * self.delta_time if not self.config['integer'] else int(self.accel_x * self.delta_time)
            self.vel_y += self.accel_y * self.delta_time if not self.config['integer'] else int(self.accel_y * self.delta_time)

        self.prev_center_x, self.prev_center_y = self.center_x, self.center_y
        self.prev_time = current_time
        
        if self.is_debugging_enabled("TRACKING") and self.system_state != 'Boost':
            print("State:", self.system_state)
            print(f'Filtered Center: ({self.center_x:.2f}, {self.center_y:.2f})')
        if self.is_debugging_enabled("FPS") and self.system_state != 'Boost':
            fps = self.get_fps() if self.get_fps() else 0
            print(f'FPS: {fps:.2f}')
        
        if self.is_debugging_enabled("COMMUNICATION"): 
        
            if self.system_state == 'Manual':
                print(f'Speed sent to Arduino (Manual): {self.vel_x:.2f} {self.vel_y:.2f}')
            elif self.system_state == 'Launch':
                if(not self.config['integer']):
                    print(f'Speed sent to Arduino: {self.vel_x:.2f} {self.vel_y:.2f}')
                else:
                    print(f'Speed sent to Arduino: {int(self.vel_x)} {int(self.vel_y)}')
                    
    def rolling_average(self, velocity: tuple):
        weights = np.linspace(1, 0, self.config['avg_number'])
        weights /= weights.sum()  # Normalize weights
        
        self.mov_avg_vel_x = np.roll(self.mov_avg_vel_x, -1)
        self.mov_avg_vel_y = np.roll(self.mov_avg_vel_y, -1)
        self.mov_avg_vel_x[-1] = velocity[0]
        self.mov_avg_vel_y[-1] = velocity[1]
        
        weighted_avg_vel_x = np.dot(self.mov_avg_vel_x, weights)
        weighted_avg_vel_y = np.dot(self.mov_avg_vel_y, weights)
        
        return weighted_avg_vel_x, weighted_avg_vel_y

    def process_center(self, center_x_unfiltered, center_y_unfiltered):
        # Local variables are now handled as class attributes initialized in initialize_variables()
        local_prev_time = self.local_prev_time
        local_vel_x = self.local_vel_x
        local_vel_y = self.local_vel_y
        local_accel_x = self.local_accel_x
        local_accel_y = self.local_accel_y
        
        # Compute delta time
        current_time = time.time()  # Use high-resolution timer for accuracy
        if local_prev_time is not None:
            delta_time = current_time - local_prev_time
        else:
            delta_time = 0.1  # Initial delta_time to prevent division by zero or large jumps
        
        if self.tracking_state:
            (x, y) = (center_x_unfiltered, center_y_unfiltered) - np.array([(self.config['frame_width'])/2, (self.config['frame_height'])/2])
            x_filtered, y_filtered = self.kf_manager.apply_filter((x, y)) if self.config['kalman'] else (x, y)
            
            self.center_x = x_filtered if (abs(x_filtered - self.prev_center_x) > self.config['mask_x']) else self.prev_center_x
            self.center_y = y_filtered if (abs(y_filtered - self.prev_center_y) > self.config['mask_y']) else self.prev_center_y
            
            # Calculate velocity in pixels per second
            if len(self.mov_avg_vel_x) > 1:
                local_vel_x, local_vel_y = self.rolling_average((
                    (self.center_x - self.prev_center_x) / delta_time, 
                    (self.center_y - self.prev_center_y) / delta_time
                ))
            else:
                local_vel_x = (self.center_x - self.prev_center_x) / delta_time
                local_vel_y = (self.center_y - self.prev_center_y) / delta_time
            
            # Calculate acceleration in pixels per second squared
            local_accel_x = (local_vel_x - self.local_vel_x) / delta_time
            local_accel_y = (local_vel_y - self.local_vel_y) / delta_time
            
        else:
            # Use previous velocity and acceleration to estimate position if detection is lost
            x = self.prev_center_x + local_vel_x * delta_time + 0.5 * local_accel_x * (delta_time ** 2)
            y = self.prev_center_y + local_vel_y * delta_time + 0.5 * local_accel_y * (delta_time ** 2)
            self.center_x, self.center_y = self.kf_manager.apply_filter((x, y)) if self.config['kalman'] else (x, y)
        
        # Update previous velocity, acceleration, and time for the next iteration
        self.local_vel_x, self.local_vel_y = local_vel_x, local_vel_y
        self.local_accel_x, self.local_accel_y = local_accel_x, local_accel_y
        self.local_prev_time = current_time

        self.update_speed_and_time()

    def cleanup(self, serial_comm, cap, result):
        if serial_comm:
            serial_comm.send_speed_to_arduino([0, 0])
            serial_comm.close()
        cap.release()
        result.release()
        cv2.destroyAllWindows()

    def annotate_frame(self, frame, box, confidence):
        # Annotate the frame with the bounding box and confidence score
        x1, y1, x2, y2 = map(int, box[:4])
        label = f"{confidence:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), self.config['color'], 2)
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.config['color'], 2)
        
    def preprocess(self, frame):
        # Preprocess the frame before feeding it to the model
        img = cv2.resize(frame, (640, 640))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.transpose(2, 0, 1)  # HWC to CHW
        img = img[np.newaxis, :, :, :].astype(np.float32) / 255.0  # Add batch dimension and normalize
        return img

    def postprocess(self, outputs):
        output_array = outputs[0]

        # Handle different possible output shapes
        if output_array.ndim == 2:
            output_array = output_array.transpose(1, 0)
        elif output_array.ndim == 4:
            # If the output is in the shape [N, C, H, W] (e.g., for YOLO models)
            output_array = output_array[0].transpose(1, 2, 0).reshape(-1, output_array.shape[1])
        
        boxes = []

        # Iterate through each detection
        for i in range(output_array.shape[0]):
            if len(output_array[i, 4]) > 0:
                
                idx = np.argmax(output_array[i, 4])
                confidence = output_array[i, 4][idx]
                x_center = output_array[i, 0][idx]
                y_center = output_array[i, 1][idx]
                width = output_array[i, 2][idx]
                height = output_array[i, 3][idx]
                
                if confidence >= self.config['confidence']:
                    # Calculate the box coordinates
                    x1 = x_center - (0.5 * width)
                    y1 = y_center - (0.5 * height)
                    x2 = x_center + (0.5 * width)
                    y2 = y_center + (0.5 * height)

                    # Store the box with confidence
                    box = [x1, y1, x2, y2, confidence]
                    boxes.append(box)
        
        # Ensure all elements are consistent before conversion
        if len(boxes) > 0:
            boxes = np.array(boxes, dtype=float)
            self.tracking_state = 1
        else:
            boxes = np.zeros((0, 5), dtype=float)
            self.tracking_state = 0

        # Use self.config to reference frame size
        # print("unfilter center:", ((x_center, y_center) - np.array([self.config['frame_width'] / 2, self.config['frame_height'] / 2])))
        
        self.process_center(x_center, y_center)
        
        return boxes

    def process_frame(self, frame):
        # Preprocess the frame
        input_tensor = self.preprocess(frame)

        # Run inference
        outputs = self.ort_session.run(None, {'images': input_tensor})

        # Post-process the outputs
        boxes = self.postprocess(outputs)

        if self.counter:
            self.frame_counter += 1

        # Only proceed if there are detections
        if len(boxes) > 0:
            # Choose the box with the highest confidence
            box = boxes[0]
            confidence = box[4]

            # Annotate the frame
            self.annotate_frame(frame, box, confidence)

            if self.counter:
                self.rocket_counter += 1

        return frame
