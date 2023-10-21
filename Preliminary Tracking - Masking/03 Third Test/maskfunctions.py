import cv2 as cv
import numpy as np

#input a mask to this function and it will return the coordinates of the highest pixel in the mask
def getMaskHighest(mask):
    
    #finding countours
    cnts = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c = max(cnts, key=cv.contourArea, default = 0)

    #accounting for variable type errors when mask is empty
    if(type(c) == int):
        return [0,0]
    else:
        if(c.any() == None):
            return c
        else:
            #finding max y pos of contours
            top = tuple(c[c[:, :, 1].argmin()][0])
            return top
        
def maskBlue(image):
        
    #convert to hsv format
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV) 

    #define lower and upper bounds of blue to be removed
    #lower_blue = np.array([60, 35, 140])
    upper_blue = np.array([180, 255, 255])
    lower_blue = np.array([100, 50, 200]) #WORKS VERY WELL

    #mask image according to arrays above, and apply slight blurring
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    mask = cv.GaussianBlur(mask, (5, 5), 0)

    #inverse mask
    mask = cv.bitwise_not(mask)

    return mask

def maskOrange(image):

        #convert to hsv format
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV) 

    #define lower and upper bounds of orange to be removed
    lower_orange = np.array([30, 0, 225])
    upper_orange = np.array([60, 40, 255])

    #mask image according to arrays above, and apply slight blurring
    mask = cv.inRange(hsv, lower_orange, upper_orange)
    mask = cv.GaussianBlur(mask, (5, 5), 0)

    return mask