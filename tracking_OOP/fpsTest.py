import time
import cv2

cap = cv2.VideoCapture(0)

while True:
    startTime = time.time()
    success, frame = cap.read()
    if not success:
        break
    cv2.imshow("Webcam", frame)

    del_t = time.time() - startTime
    print(f'FPS: {1/del_t}')