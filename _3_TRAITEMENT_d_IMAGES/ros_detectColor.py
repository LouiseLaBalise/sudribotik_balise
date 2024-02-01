import os
import cv2
import numpy as np


"""
Detect HSV color range on an image.
    frame (numpy.ndArray)   ->      data array of the image.
    hue (tuple)             ->      range of hue values on the hue compass between 0 and 180.
    saturation (tuple)      ->      range of saturation between 0 and 255.
    values (tuple)          ->      range of values between 0 and 255.
    minSurface (int)        ->      number of minimum pixels for a detected area to be take into account.
    maxSurface (int)        ->      number of maximum pixels for a detected area to be take into account.

Return bool success and a list with positions of all area detected.
"""
def colorDetection(frame, hue:tuple, saturation:(50, 255), value:(50, 255),
                   minSurface=0, maxSurface=99999):

    ret = False #return succes

    #List of positions for detected colors
    color_positions = []

    #Convert image from original colorspace bgr to hsv
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #Define hsv limits to see
    lower_limit_hsv = np.array([hue[0], saturation[0], value[0]])
    upper_limit_hsv = np.array([hue[1], saturation[1], value[1]])

    #Create mask to only show hsv limits calculated from bgr
    mask = cv2.inRange(hsv_frame, lower_limit_hsv, upper_limit_hsv)

    #In case of red hue (or if 1st hue value is lower than 2nd)
    if (hue[0]>hue[1]):
        #Mask before 0
        lower_limit_hsv1 = np.array([hue[0], saturation[0], value[0]])
        upper_limit_hsv1 = np.array([180, saturation[1], value[1]])
        sub_mask1 = cv2.inRange(hsv_frame, lower_limit_hsv1, upper_limit_hsv1)
        #Mask after 0
        lower_limit_hsv2 = np.array([0, saturation[0], value[0]])
        upper_limit_hsv2 = np.array([hue[1], saturation[1], value[1]])
        sub_mask2 = cv2.inRange(hsv_frame, lower_limit_hsv2, upper_limit_hsv2)
        #Summ masks
        mask = sub_mask1+sub_mask2

    #Get contours if there are hsv values detected on the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        #Filter on minSurface and maxSurface
        cnt_px = cv2.contourArea(cnt)
        if cnt_px<=minSurface or cnt_px>=maxSurface :
            continue

        x, y, w, h = cv2.boundingRect(cnt) #get rectangles coordinates
        color_positions.append((x, y, x+w, y+h))
        ret = True

    return ret, color_positions