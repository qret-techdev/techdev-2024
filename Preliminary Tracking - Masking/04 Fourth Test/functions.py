import cv2 as cv
import numpy as np
import math

#function to get range of blues
def getBlueRange(img):

    #convert image to hsv
    img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    #get image size
    dimensions = img.shape

    #transfer dimensions to new variables scaled by factor for processing time
    width = 2
    length = int(dimensions[1]/width)
    height = int(dimensions[0]/width)

    #defining min hsv as something big so it's replaced by the smallest we have
    min_h = 10000
    min_s = 10000
    min_v = 10000

    #defining max hsv as something small so it's replaced by the biggest we have
    max_h = 0
    max_s = 0
    max_v = 0

    #dumb ass way to find min and max hsv values
    for i in range(length):
        for j in range(height):

            hsv = img[i+width][j+width]
            
            if(hsv[0] > max_h):
                max_h = hsv[0]
            elif(hsv[0] < min_h):
                min_h = hsv[0]
            elif(hsv[1] > max_s):
                max_s = hsv[1]
            elif(hsv[1] < min_s):
                min_s = hsv[1]
            elif(hsv[2] > max_v):
                max_v = hsv[2]
            elif(hsv[2] < min_v):
                min_v = hsv[2]

    min = [min_h, min_s, min_v]
    max = [max_h, max_s, max_v]
    return([min,max])

#function that returns the coordinates of the highest pixel in the inputted mask
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

#function that returns the inputted image with blue masked out according to the bounds inputted       
def maskBlue(image, lower_blue, upper_blue):
        
    #convert to hsv format
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV) 

    #mask image according to arrays, and apply slight blurring
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    mask = cv.GaussianBlur(mask, (5, 5), 0)

    #inverse mask
    mask = cv.bitwise_not(mask)

    return mask

#function to read all of the images
def readImgs(num_imgs):
    images = [None] * num_imgs

    for i in range(num_imgs):
        images[i] = cv.imread("C:\\Users\\Teighin Nordholt\\Desktop\\Tech Dev Reading Break\\04 Fourth Test\\sky_" + str(i+1) + ".png")

    return images

def drawLines(img):

    #detect edges using canny
    dst = cv.Canny(img, 50, 200, None, 3)

    #copy edges to new image
    dstp = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)

    #apply Standard HoughLine transform to detect lines
    lines = cv.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)

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
            cv.line(dstp, pt1, pt2, (0, 0, 255), 3, cv.LINE_AA)

    return dstp