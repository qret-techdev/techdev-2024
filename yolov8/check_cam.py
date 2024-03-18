import cv2

cap = cv2.VideoCapture(1)  # Start capturing video from the first webcam
cap.set(3, 640)  # Set the width of the video frames
cap.set(4, 640)  # Set the height of the video frames

if not cap.isOpened():  # Check if the video capture has been initialized correctly
    print("Error: Could not open video device.")
else:
    while True:
        ret, img = cap.read()  # Read a frame from the video capture
        if not ret:
            print("Error: Failed to capture image.")
            break  # Exit the loop if an error occurred during frame capture
        
        cv2.imshow('Webcam', img)  # Display the captured frame in a window

        if cv2.waitKey(1) == ord('q'):  # Break the loop if 'q' is pressed
            break

cap.release()  # Release the video capture object
cv2.destroyAllWindows()  # Close all OpenCV windows
