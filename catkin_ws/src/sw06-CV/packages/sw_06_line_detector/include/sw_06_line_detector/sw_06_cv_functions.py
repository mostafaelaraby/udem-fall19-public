import cv2
import numpy as np
import rospy
import math
from scipy.ndimage.morphology import binary_dilation
from time import time
measure_time = False


## Software Exercise 6: Choose your category (1 or 2) and replace the cv2 code by your own!

## CATEGORY 1
def inRange(hsv_image, low_range, high_range):
    if measure_time:
        start_time = time()
    # an empty matrix result having the resulting mask
    result = np.zeros(hsv_image.shape[0:-1], dtype=np.uint8)
    # and now the condition used to create the mask
    # the condition is that each pixel is within upper and lower range in each channel huv
    condition = (hsv_image[:, :, 0] >= low_range[0]) & (hsv_image[:, :, 0] <= high_range[0]) \
        & (hsv_image[:, :, 1] >= low_range[1]) & (hsv_image[:, :, 1] <= high_range[1]) \
        & (hsv_image[:, :, 2] >= low_range[2]) & (hsv_image[:, :, 2] <= high_range[2])

    result[condition] = 255
    if measure_time:
        print('My Implementation inrange took ' +
              str(time() - start_time) + 'seconds')
        start_time = time()
        _ = cv2.inRange(hsv_image, low_range, high_range)

        print('Opencv In Range took ' + str(time() - start_time) + 'seconds')
    return result

def bitwise_or(bitwise1, bitwise2):
    if measure_time:
        start_time = time()
    result = bitwise1 | bitwise2
    if measure_time:
        print('My Implementation bitwise_or took ' +
              str(time() - start_time) + 'seconds')
        start_time = time()
        _ = cv2.bitwise_or(bitwise1, bitwise2)
        print('OpenCv bitwise_or took ' + str(time() - start_time) + 'seconds')

    return result

def bitwise_and(bitwise1, bitwise2):
    if measure_time:
        start_time = time()
    result = bitwise1 & bitwise2
    if measure_time:
        print('My Implementation bitwise_and took ' +
              str(time() - start_time) + 'seconds')
        start_time = time()
        _ = cv2.bitwise_and(bitwise1, bitwise2)
        print('Opencv bitwise_or took ' + str(time() - start_time) + 'seconds')

    return result

def getStructuringElement(shape, size):
    if measure_time:
        start_time = time()
    height, width = size
    result = np.zeros(size, dtype=np.uint8)
    if shape == cv2.MORPH_ELLIPSE:
        # we only need to compute the ellipse kernel which will
        # be used later in the dilate part
        center_y = height//2
        center_x = width//2
        if center_y == 0:
            inv_center_y = 0
        else:
            inv_center_y = 1. / (center_y*center_y)
        for i in range(height):
            dy = i - center_y
            j1 = 0
            j2 = 0
            if(abs(dy) <= center_y):
                dx = center_x * \
                    math.sqrt((center_y*center_y - dy*dy) * inv_center_y)
                dx = int(np.round(dx))
                j1 = int(max(center_x - dx, 0))
                j2 = int(min(center_x+dx+1, width))
            for j in range(j1, j2):
                result[i, j] = 1
    else:
        raise NotImplementedError
    
    if measure_time:
        print('My Implementation structuringelement took ' + str(time() - start_time) + 'seconds')
        start_time = time()
        _ = cv2.getStructuringElement(shape, size)
        print('Opencv getStructuringElement took ' + str(time() - start_time) + 'seconds')
        
    return result

def dilate(bitwise, kernel):
    if measure_time:
        start_time = time()
    dilate_output = np.zeros(bitwise.shape, dtype=np.uint8)
    height, width = bitwise.shape
    h_k, w_k = kernel.shape
    step_h = int(math.floor(h_k /2))
    step_w = int(math.floor(w_k/2))
    for i in range(height):
            for j in range(width):
                    # computing start indx and end in the image
                    # on i and j level
                    # these are the pixels we are going to covolve through our kernel
                    # and then we get the maximum
                    # and these conditions are then used to work on poits that doesnt have pixels top left right in case of image edges(end in each row or the start etc..)
                    start_indx_i = i-step_h 
                    start_indx_i_dev = 0
                    end_indx_i_dev = h_k -1
                    if start_indx_i<0:
                            start_indx_i_dev = start_indx_i * -1
                            start_indx_i = 0
                    end_indx_i = i+step_h
                    if end_indx_i>height-1:
                            end_indx_i_dev =h_k -  end_indx_i + height + 1
                            end_indx_i = height-1

                    start_indx_j = j-step_w
                    start_indx_j_dev = 0
                    end_indx_j_dev = w_k -1
                    if start_indx_j<0:
                            start_indx_j_dev = start_indx_j * -1
                            start_indx_j = 0
                    end_indx_j = j+step_w
                    if end_indx_j>width-1:
                            end_indx_j_dev = w_k -  end_indx_j + width + 1
                            end_indx_j = width-1

                    tmp = np.zeros(kernel.shape)
                    tmp[start_indx_i_dev:end_indx_i_dev,start_indx_j_dev:end_indx_j_dev] = bitwise[start_indx_i:end_indx_i,start_indx_j:end_indx_j]
                    dilate_output[i,j] = np.max(np.multiply(kernel, tmp))
    if measure_time:
        print('My Implementation dilate took ' + str(time() - start_time) + 'seconds')
        start_time = time()
        _ = cv2.dilate(bitwise, kernel)
        print('Opencv dilate took ' + str(time() - start_time) + 'seconds')
    
    # binary_dilate in scipy would be faster and better
    return dilate_output



## CATEGORY 2
def Canny(image, threshold1, threshold2, apertureSize=3):
	return cv2.Canny(image, threshold1, threshold2, apertureSize=3)


## CATEGORY 3 (This is a bonus!)
def HoughLinesP(image, rho, theta, threshold, lines, minLineLength, maxLineGap):
	return cv2.HoughLinesP(image, rho, theta, threshold, lines, minLineLength, maxLineGap)