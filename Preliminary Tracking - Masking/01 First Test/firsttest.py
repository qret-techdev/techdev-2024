import cv2 as cv
import numpy as np
from myfunctions import findHighest

#load image
img = cv.imread("C:\\Users\\Teighin Nordholt\\Desktop\\Tech Dev Reading Break\\First Test\\lying_rocket.png", cv.IMREAD_UNCHANGED)

top = findHighest(img)

cv.circle(img, top, 5, (0, 0, 255), 2)

#print mask
cv.imshow("name", img)

#wait for a key to be pressed and delete all windows
cv.waitKey()
cv.destroyAllWindows()