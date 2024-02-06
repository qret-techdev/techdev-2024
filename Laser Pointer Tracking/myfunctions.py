import cv2 as cv
import numpy as np

def get_adj_frame(frame, fov, d_theta):
    shape = frame.shape
    adjusted_frame = np.zeros(shape)
    adjusted_frame[int(shape[0] - shape[0]*(fov-d_theta)/fov):, :, :] = frame[:int(shape[0]*(fov-d_theta)/fov), :, :]
    return adjusted_frame

def subtract_prev_frame(curr_frame, prev_frame, fov, d_theta):
    #print(np.maximum(curr_frame[0][0] - np.array([168, 128, 98]), np.array([0, 0, 0])))
    return np.maximum(curr_frame - get_adj_frame(prev_frame, fov, d_theta), np.array([0, 0, 0])).astype(np.uint8)
    
def findMax(image):
    #convert to hsv format
    grayImage = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    loc = np.unravel_index(grayImage.argmax(), grayImage.shape)[::-1]
    return loc

def findGradientMax(image, prev_loc, strength=1/4):
    grayImage = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    dims = grayImage.shape

    x_axis = np.linspace(-prev_loc[0]/dims[0], 1 - prev_loc[0]/dims[0], dims[0])[:, None]
    y_axis = np.linspace(-prev_loc[1]/dims[1], 1 - prev_loc[1]/dims[1], dims[1])[None, :]

    subtract_arr = np.sqrt(x_axis ** 2 + y_axis ** 2)
    multiply_arr = np.ones(dims) - strength*subtract_arr

    gradient_image = grayImage*multiply_arr

    loc = np.unravel_index(gradient_image.argmax(), gradient_image.shape)[::-1]
    return loc