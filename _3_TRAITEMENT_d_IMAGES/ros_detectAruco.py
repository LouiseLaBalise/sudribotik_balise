import cv2
import numpy as np


"""
Detect ArUco markers.
    frame (numpy.ndArray) ->    data array of the image.

Return function success, corners positions, their ids.
"""
def detectAruco(frame):

    #Get minimal Aruco ductionnary needed 
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    
    #Create parametors for detection
    aruco_parameters = cv2.aruco.DetectorParameters()

    #Create Aruco detector
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_parameters)

    #Detect ArUco markers
    corners, ids, rejected_corners = aruco_detector.detectMarkers(img)
