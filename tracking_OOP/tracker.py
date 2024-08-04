import numpy as np
import cv2
from collections import defaultdict
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors
from simple_pid import PID
from config import AVG_NUMBER, X_FRAME_SIZE, Y_FRAME_SIZE, WEIGHTS, PARAMFILE, PIDX, PIDY, T_DELAY
from utils import get_pid_params

class ObjectTracker:
    def __init__(self):
        self.mov_avg_x = np.zeros(AVG_NUMBER)
        self.mov_avg_y = np.zeros(AVG_NUMBER)
        Kp, Ki, Kd = get_pid_params(PARAMFILE, PIDX)
        self.pidx = PID(Kp, Ki, Kd, setpoint=0)
        Kp, Ki, Kd = get_pid_params(PARAMFILE, PIDY)
        self.pidy = PID(Kp, Ki, Kd, setpoint=0)
        self.speed = [0, 0]
        self.accel = [0, 0]
        self.loc_x_y_filt = [0, 0]
        self.prev_time = 0
        self.delta_t = 0
        self.track_history = defaultdict(lambda: [])
        self.model = YOLO(WEIGHTS)
        self.sys_state = 0
        self.max_speed = 90
        self.max_accel = 180
        self.t_delay = T_DELAY
        self.prevx = 0
        self.prevy = 0
        self.velx = 0
        self.vely = 0
        self.trip_init_guess = 0
        self.motor_speedy_init_guess = 0
        self.motor_speedx_init_guess = 0
        self.count = False
        self.box_count = 0
        self.frame_count = 0
            
    def process_center(self, loc: tuple):
        loc_rel = loc - np.array((X_FRAME_SIZE/2, Y_FRAME_SIZE/2))
        self.mov_avg_x = np.roll(self.mov_avg_x, -1)
        self.mov_avg_y = np.roll(self.mov_avg_y, -1)
        self.mov_avg_x[-1] = loc_rel[0]
        self.mov_avg_y[-1] = -loc_rel[1]
        return sum(self.mov_avg_x) / AVG_NUMBER, sum(self.mov_avg_y) / AVG_NUMBER

    def process_frame(self, frame, confidence_threshold=0.6):
        loc = (0, 0)
        loc_filt = (0, 0)
        results = self.model.track(frame, persist=True)
        boxes = results[0].boxes.xyxy
        confs = results[0].boxes.conf
        if self.count:
            self.frame_count += 1
        if results[0].boxes.id is not None:
            index=np.argmax(confs.cpu().numpy())
            clss = results[0].boxes.cls.tolist()
            track_ids = results[0].boxes.id.int().tolist()
            annotator = Annotator(frame, line_width=2)
            box = boxes[index]
            cls=clss[index]
            track_id = track_ids[index]
            conf = confs[index]
            if self.count:
                self.box_count += 1
            if conf >= confidence_threshold:
                annotator.box_label(box, color=colors(int(cls), True), label=f"{conf:.2f}")
                loc = (((box[0] + box[2]) / 2).cpu().numpy(), ((box[1] + box[3]) / 2).cpu().numpy())
                loc_filt = self.process_center(loc)
                self.track_history[track_id].append((int(loc[0]), int(loc[1])))
                if len(self.track_history[track_id]) > 30:
                    self.track_history[track_id].pop(0)
                points = np.array(self.track_history[track_id], dtype=np.int32).reshape((-1, 1, 2))
                cv2.circle(frame, self.track_history[track_id][-1], 7, colors(int(cls), True), -1)
                cv2.polylines(frame, [points], isClosed=False, color=colors(int(cls), True), thickness=2)
        return frame, loc_filt
