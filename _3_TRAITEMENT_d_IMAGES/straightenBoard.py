import sys
import os
import cv2
import numpy as np
import detectAruco
import argparse

def straightenBoard(filename, path="media/"):

    #Load image
    image = cv2.imread(filename=path+filename)

    #Define ArUco dict
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

    #Get detected corners
    ret, corners, ids = detectAruco.detectAruco(path+filename)

    #In case no tags have been detected
    if not ret :
        print("Can't staighten image.")
        return None
    
    #Select references points (here rectangle)
    src_pts = np.float32([[0, 0],
                          [image.shape[1], 0],
                          [image.shape[1], image.shape[0]],
                          [0, image.shape[0]]])
    
    #Detected corners from detectAruco become destination points
    #(only the first tag for now)
    dst_pts = np.float32(corners[0])

    #Calculate perspective transform
    perspective_transform = cv2.getPerspectiveTransform(src_pts, dst_pts)

    #Apply transformation to image
    warped_image = cv2.warpPerspective(image, perspective_transform, (image.shape[1], image.shape[0]))

    #Show image
    cv2.imshow('Warped Image', warped_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":

    #Create a parser for CLI options
    parser = argparse.ArgumentParser(prog="detectColor.py",
                                     description="Detect specified range of color on an image.") 

    #All arguments available    
    parser.add_argument("path_to_file",                                     #argument name
                        action="store",                                     #mendatory
                        type=str,                                           #must be string
                        help="Path to file")                                #help text
    
    #Get all arguments
    args = parser.parse_args()
    filename = args.path_to_file.split('/')[-1] #get filename

    if len(args.path_to_file.split('/')) >=2:   #get path if there is one
        image_path = '/'.join(args.path_to_file.split('/')[:-1])+'/'
    else :
        image_path = ""

    straightenBoard(filename, path=image_path)