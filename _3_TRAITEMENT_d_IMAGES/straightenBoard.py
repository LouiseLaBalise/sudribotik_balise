import sys
import os
import cv2
import numpy as np
import detectAruco

def starightenBoard(argv):

    #Get Filename
    filename = argv[0]

    #Load image
    image = cv2.imread(filename)

    #Define ArUco dict
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

    #Get detected corners
    ret, corners, ids = os.system(f"python3 detectAruco.py {filename}")

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
    starightenBoard(sys.argv[1:])