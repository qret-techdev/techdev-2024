#imporing libraries
import numpy as np
from collections import defaultdict
import cv2 #getting video, monitoring key presses while in video window
from simple_pid import PID #control system pid
from simdkalman import KalmanFilter

import time #measure time, calculate speed from accel and delta t
import serial #communication with arduino

import argparse

import cv2.dnn
import numpy as np
import yaml

from ultralytics.utils import ASSETS, yaml_load
from ultralytics.utils.checks import check_yaml


DEVICE_NUMBER = 1
Y_FRAME_SIZE = 640
X_FRAME_SIZE = 640
CONFIDENCE = 0.35
SERIAL = 0

if(SERIAL):
  ser = serial.Serial('COM5', 115200) #might have to change com number, ex 'COM11'... best to keep a high baud rate, make sure it matches w/ arduino
  
def process_center(x1, y1, x2, y2):
  center = (((x1 + x2)/2), ((y1 + x2)/2))
  rock_x_filt, rock_y_filt = center - np.array((X_FRAME_SIZE/2, Y_FRAME_SIZE/2))
  



  return (rock_x_filt, rock_y_filt)



# Load class names and colors
CLASSES = ["rocket"]
colors = np.random.uniform(0, 255, size=(len(CLASSES), 3))


def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h):
    """
    Draws bounding boxes on the input image based on the provided arguments.

    Args:
        img (numpy.ndarray): The input image to draw the bounding box on.
        class_id (int): Class ID of the detected object.
        confidence (float): Confidence score of the detected object.
        x (int): X-coordinate of the top-left corner of the bounding box.
        y (int): Y-coordinate of the top-left corner of the bounding box.
        x_plus_w (int): X-coordinate of the bottom-right corner of the bounding box.
        y_plus_h (int): Y-coordinate of the bottom-right corner of the bounding box.
    """
    label = f"{CLASSES[class_id]} ({confidence:.2f})"
    color = colors[class_id]
    cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)
    cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def process_frame(frame, model):
    """
    Processes a single frame, performs inference, and draws bounding boxes.

    Args:
        frame (numpy.ndarray): The input frame from the video.
        model (cv2.dnn.Net): The loaded ONNX model.

    Returns:
        list: List of dictionaries containing detection information such as class_id, class_name, confidence, etc.
    """
    [height, width, _] = frame.shape

    # Prepare a square image for inference
    length = max((height, width))
    image = np.zeros((length, length, 3), np.uint8)
    image[0:height, 0:width] = frame

    # Calculate scale factor
    scale = length / 640

    # Preprocess the image and prepare blob for model
    blob = cv2.dnn.blobFromImage(image, scalefactor=1 / 255, size=(640, 640), swapRB=True)
    model.setInput(blob)

    # Perform inference
    outputs = model.forward()

    # Prepare output array
    outputs = np.array([cv2.transpose(outputs[0])])
    rows = outputs.shape[1]

    boxes = []
    scores = []
    class_ids = []

    # Iterate through output to collect bounding boxes, confidence scores, and class IDs
    for i in range(rows):
        classes_scores = outputs[0][i][4:]
        (minScore, maxScore, minClassLoc, (x, maxClassIndex)) = cv2.minMaxLoc(classes_scores)
        if maxScore >= CONFIDENCE:
            box = [
                outputs[0][i][0] - (0.5 * outputs[0][i][2]),
                outputs[0][i][1] - (0.5 * outputs[0][i][3]),
                outputs[0][i][2],
                outputs[0][i][3],
            ]
            boxes.append(box)
            scores.append(maxScore)
            class_ids.append(maxClassIndex)

    # Apply NMS (Non-maximum suppression)
    result_boxes = cv2.dnn.NMSBoxes(boxes, scores, 0.25, 0.45, 0.5)

    detections = []

    # Iterate through NMS results to draw bounding boxes and labels
    for i in range(len(result_boxes)):
        index = result_boxes[i]
        box = boxes[index]
        detection = {
            "class_id": class_ids[index],
            "class_name": CLASSES[class_ids[index]],
            "confidence": scores[index],
            "box": box,
            "scale": scale,
        }
        detections.append(detection)
        draw_bounding_box(
            frame,
            class_ids[index],
            scores[index],
            round(box[0] * scale),            # x1 (int): top-left x cord
            round(box[1] * scale),            # y1 (int): top left y cord
            round((box[0] + box[2]) * scale), # x2 (int): bottom-right x cord
            round((box[1] + box[3]) * scale), # y2 (int): bottom-right y cord  
        )
        process_center((box[0] * scale), (box[1] * scale), ((box[0] + box[2])* scale), ((box[1] + box[3]) * scale))
        
    return detections


def main(onnx_model):
    """
    Main function to load ONNX model, capture video from webcam, perform inference, and display the output frames.

    Args:
        onnx_model (str): Path to the ONNX model.
    """
    # Load the ONNX model
    model = cv2.dnn.readNetFromONNX(onnx_model)

    # Capture video from webcam
    cap = cv2.VideoCapture(0)
    prev_time = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Process the frame and draw bounding boxes
        prev_time = time.time()
        process_frame(frame, model)
        delta_t = time.time()-prev_time
        print(f"\nTime: {1000*delta_t:.2f}")

        # Display the frame
        cv2.imshow("Webcam", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="best.onnx", help="Input your ONNX model.")
    args = parser.parse_args()
    main(args.model)
