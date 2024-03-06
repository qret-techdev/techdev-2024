import cv2 as cv

video_name = "4"

cap = cv.VideoCapture("C:\\Users\\Teighin Nordholt\\Documents\\GitHub\\techdev-2024\\Image Splitting\\" + video_name + ".mp4")

i = 0
image_index = 1

while(cap.isOpened()):
    ret, frame = cap.read()

    if(i==10):

        cv.imwrite(video_name + '_' + str(image_index) + '.jpg', frame)
        image_index += 1
        i = 1

    i += 1

cap.release