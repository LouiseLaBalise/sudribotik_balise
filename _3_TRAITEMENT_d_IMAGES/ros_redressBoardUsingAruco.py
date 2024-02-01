import cv2
import os
import numpy as np
import argparse
import time



FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)


"""
Redress a board with 4 Aruco tags.
    filename (str)      ->      name of the inputed image.
                                output will be store next to filename.
    corner_ids (tuple)  ->      ids of the 4 mendatory ArUco tags
    method (str)        ->      chose method, CORNER is by default.

Return function success and filename.
"""
def redressBoardUsingAruco(frame, corner_ids = (20, 21, 22, 23), method="CORNER"):

    #FOR EDITION 2024
    HEIGHT_BETWEEN_TWO_ARUCO_INTERIORS_IN_MM = 1100
    HEIGHT_BETWEEN_BOARD_AND_ARUCO_EXTERIOR_IN_MM = 450
    WIDTH_BETWEEN_BOARD_AND_ARUCO_EXTERIOR_IN_MM = 692.3

    #Quit if not 4 tags in params
    if len(corner_ids)!=4:
        print(f"Log [{FILE_NAME}]: Il faut 4 ids de tags ArUco. Impossible de redresser l'image.")
        return False, frame

    #Create ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    #Create parametors for detection
    aruco_parameters = cv2.aruco.DetectorParameters()
    #Create Aruco detector
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_parameters)
    #Detect ArUco markers
    corners, ids, rejected_corners = aruco_detector.detectMarkers(frame)

    #Quit if there is not even 1 id detected
    if ids is None :
        print(f"Log [{FILE_NAME}]: Aucun tag détecté. Il en faut au moins 4.")
        return False, frame

    #All 4 Aruco markers are needed to perform image to reddress
    corner_ids_no_detected = [] #will store ids
    corners_pos_detected_ids = [] #will store corners positions
    ids = ids.flatten() #flatten ids array to facilitate its navigation

    for _id in corner_ids: #test to see if they have been detected
        if _id not in ids:
            corner_ids_no_detected.append(_id)
        else :
            index_of_id = np.where(ids ==_id)[0].item() #get the index of the id
            corners_pos_detected_ids.append(corners[index_of_id]) #add each corners to a list


    #If not all ids have been detected stop function
    if corner_ids_no_detected:
        print(f"Log [{FILE_NAME}]: Tag(s) {corner_ids_no_detected} non detecté(s). Impossible de redresser l'image.")
        return False, frame

    #Sort corners pos to have [bottom-right, bottom-left, top-left, top-right]
    sorted_tags = corners_pos_detected_ids
    sorted_tags = sorted(sorted_tags, reverse=True, key=lambda coord: coord[0][0][1])#Reverse sort by y axis
    sorted_tags[:2] = sorted(sorted_tags[:2], key=lambda coord: coord[0][0][0])#Normal sort by x axis for the bottom
    sorted_tags[2:] = sorted(sorted_tags[2:], reverse=True, key=lambda coord: coord[0][0][0])#Reverse sort by x axis for the top

    #Calc offset to get real corners of the board
    pixels_per_mm = (sorted_tags[0][0][3][1]-sorted_tags[3][0][0][1])/HEIGHT_BETWEEN_TWO_ARUCO_INTERIORS_IN_MM
    width_offset = WIDTH_BETWEEN_BOARD_AND_ARUCO_EXTERIOR_IN_MM * pixels_per_mm
    height_offset = HEIGHT_BETWEEN_BOARD_AND_ARUCO_EXTERIOR_IN_MM * pixels_per_mm

    if method=="CORNER" or method: #default methods
        #Sort corners inside of tags following the previous sorting pattern
        for k in range(4):
            temp_sort = sorted_tags[k][0].tolist() #convert to list to facilitate sorting
            temp_sort = sorted(temp_sort, reverse=True, key=lambda coord: coord[1])
            temp_sort[:2] = sorted(temp_sort[:2], key=lambda coord: coord[0])
            temp_sort[2:] = sorted(temp_sort[2:],  reverse=True, key=lambda coord: coord[0])
            sorted_tags[k][0] = np.array(temp_sort, dtype=np.float32)#back to np array


        #Add each tag outter corner to src points
        src_pts = np.float32([])
        src_pts = np.append(src_pts, sorted_tags[0][0][0])
        src_pts = np.append(src_pts, sorted_tags[1][0][1])
        src_pts = np.append(src_pts, sorted_tags[2][0][2])
        src_pts = np.append(src_pts, sorted_tags[3][0][3])

        #Reshape src points to separate each tag
        src_pts = src_pts.reshape(-1, 2)


    #Reddress board using center of each Aruco tags NOT WORKING RIGHT NOW
    elif method=="CENTER":
        #Get source points (where points are on the distorded board)
        src_pts = np.float32([])

        for tag_corners in sorted_tags:
            #1st calc center of each tag
            center_x = int((tag_corners[0][0][0] + tag_corners[0][2][0]) / 2)
            center_y = int((tag_corners[0][0][1] + tag_corners[0][2][1]) / 2)

            #2nd add each tag center to src points
            src_pts = np.append(src_pts, [center_x, center_y])

        #3rd reshape src points to separate each tag
        src_pts = src_pts.reshape(-1, 2)


    #Get max height - pattern is src_pts[up/down and left/right][x=0/y=1]
    height_up = np.sqrt(((src_pts[0][0] - src_pts[3][0]) ** 2) + ((src_pts[0][1] - src_pts[3][1]) ** 2))
    height_down = np.sqrt(((src_pts[1][0] - src_pts[2][0]) ** 2) + ((src_pts[1][1] - src_pts[2][1]) ** 2))
    max_height = int(max(height_up, height_down))

    #Get max width
    width_up = np.sqrt(((src_pts[0][0] - src_pts[1][0]) ** 2) + ((src_pts[0][1] - src_pts[1][1]) ** 2))
    width_down = np.sqrt(((src_pts[2][0] - src_pts[3][0]) ** 2) + ((src_pts[2][1] - src_pts[3][1]) ** 2))
    max_width = int(max(width_up, width_down))

    #Get destination points (where src points will be map in the output image) and appropriately rotate the board
    #/!\ note that in some scenarios board will not be rotated due to a very distorted photo
    # as board height appearing taller than its width
    if (src_pts[1][0] - src_pts[0][0] > src_pts[0][1] - src_pts[3][1]): #for edition 2024 this config would be choosen
        dst_pts = np.float32([[0, max_height-1],
                                [max_width - 1, max_height - 1],
                                [max_width - 1, 0],
                                [0, 0]])
    else:
        print(f"Log [{FILE_NAME}]: Board rotated.")
        dst_pts = np.float32([[0, 0],
                          [0, max_height-1],
                          [max_width - 1, max_height - 1],
                          [max_width - 1, 0],])

    #Cast src_pts to float32
    src_pts = np.float32(src_pts)

    #Get matrix transformation
    transform_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

    #Apply transformation matrix to the image
    warped_image = cv2.warpPerspective(frame, transform_matrix, (max_width, max_height),flags=cv2.INTER_LINEAR)


    return True, warped_image


