import cv2
import numpy as np
from ultralytics import YOLO

from ultralytics.utils.checks import check_imshow
from ultralytics.utils.plotting import Annotator, colors

from collections import defaultdict

track_history = defaultdict(lambda: [])
model = YOLO("best.pt")
names = model.model.names

video_path = "/path/to/video/file.mp4"
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

assert cap.isOpened(), "Error reading video file"

# w, h, fps = (int(cap.get(x)) for x in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS))
# result = cv2.VideoWriter("object_tracking.avi",
#                        cv2.VideoWriter_fourcc(*'mp4v'),
#                        fps,
#                        (w, h))


while cap.isOpened():
    success, frame = cap.read()
    if success:
        results = model.track(frame, persist=True)
        boxes = results[0].boxes.xyxy.cuda()

        if results[0].boxes.id is not None:

            # Extract prediction results
            clss = results[0].boxes.cls.cuda().tolist()
            track_ids = results[0].boxes.id.int().cuda().tolist()
            confs = results[0].boxes.conf.float().cuda().tolist()

            # Annotator Init
            annotator = Annotator(frame, line_width=2)

            for box, cls, track_id in zip(boxes, clss, track_ids):
                annotator.box_label(box, color=colors(int(cls), True), label=names[int(cls)])

                # Store tracking history
                track = track_history[track_id]
                x_tensor, y_tensor = ((box[0] + box[2]) / 2), ((box[1] + box[3]) / 2)
                x_center, y_center = x_tensor.numpy(), y_tensor.numpy()
                print("\nID: ", track_id, "\nX Center:", x_center, "\nY_center", y_center)
                track.append((int(x_tensor), int(y_tensor)))
                if len(track) > 30:
                    track.pop(0)

                # Plot tracks
                points = np.array(track, dtype=np.int32).reshape((-1, 1, 2))
                cv2.circle(frame, (track[-1]), 7, colors(int(cls), True), -1)
                cv2.polylines(frame, [points], isClosed=False, color=colors(int(cls), True), thickness=2)
                
        # for outputing to video
        # result.write(frame) 
        cv2.imshow("Webcam", frame)   
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        break

# result.release()
cap.release()
cv2.destroyAllWindows()