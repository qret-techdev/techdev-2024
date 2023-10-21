import cv2 as cv2
import numpy as np   
import math

while(1):
    src = cv2.imread("C:\\Users\\Teighin Nordholt\\Desktop\\Tech Dev Reading Break\\04 Fourth Test\\sky_5.png", cv2.IMREAD_GRAYSCALE)

    # detect edges using canny edge, for more details you can refer https://indiantechwarrior.com/canny-edge-detection-for-image-processing/
    dst = cv2.Canny(src, 50, 200, None, 3)

    # Copy edges to the images that will display the results in BGR
    dstp = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)

    # Lets apply Standard HoughLine transform to detect lines
    lines = cv2.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)

    # Below we will display the result by drawing lines
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
            pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
            cv2.line(dstp, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)

    cv2.imshow("Source", src)
    cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", dstp)
    cv2.waitKey()