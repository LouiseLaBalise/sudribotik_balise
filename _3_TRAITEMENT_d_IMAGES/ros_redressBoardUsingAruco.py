import cv2
import os
import numpy as np
import argparse
import json



FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)


"""
Calibrate the camera to get the matrix transformation between the camera view and the board.
Matrix transformation, height and width of the final frame will be stored in the init/configuration.json file.
An outputed image is generated in test_24/ to see calibration results.

    frame (np.array)    ->      inputed image.
    corner_ids (tuple)  ->      ids of the 4 mendatory ArUco tags.
    method (str)        ->      chose method between CORNER and CENTER, it will be the point of reference
                                used to determine the matrix transformation. CORNER is set by default.

Return function success.
"""
def calibrateCameraUsingAruco(frame, corner_ids = (20, 21, 22, 23), method="CORNER"):

    #Get Aruco constants
    configuration_FILEPATH = FILE_PATH.split("_3_TRAITEMENT_d_IMAGES")[0]+"init/configuration.json"    
    with open (configuration_FILEPATH, "r") as f:
        config = json.load(f)

    #FOR EDITION 2024
    HEIGHT_BETWEEN_TWO_ARUCO_INTERIORS_IN_MM = config["HEIGHT_BETWEEN_TWO_ARUCO_INTERIORS_IN_MM"][0]
    HEIGHT_BETWEEN_BOARD_AND_ARUCO_EXTERIOR_IN_MM = config["HEIGHT_BETWEEN_BOARD_AND_ARUCO_EXTERIOR_IN_MM"][0]
    WIDTH_BETWEEN_BOARD_AND_ARUCO_EXTERIOR_IN_MM = config["WIDTH_BETWEEN_BOARD_AND_ARUCO_EXTERIOR_IN_MM"][0]

    #Quit if not 4 tags in params
    if len(corner_ids)!=4:
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Il faut 4 ids de tags ArUco. Impossible de calibrer la caméra.")
        return False

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
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Aucun tag détecté. Il en faut au moins 4.")
        return False

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
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Tag(s) {corner_ids_no_detected} non detecté(s). Impossible de calibrer la caméra.")
        return False

    #Sort corners pos to have [bottom-left, top-left, top-right, bottom-right]
    sorted_tags = corners_pos_detected_ids
    sorted_tags = sorted(sorted_tags, key=lambda coord: coord[0][0][0])#sort by x axis
    sorted_tags[:2] = sorted(sorted_tags[:2], reverse=True, key=lambda coord: coord[0][0][1])#reverse sort by y axis for the left
    sorted_tags[2:] = sorted(sorted_tags[2:],key=lambda coord: coord[0][0][1])#normal sort by y axis for the right

    #Calc offset to get real corners of the board
    pixels_per_mm = (sorted_tags[0][0][3][1]-sorted_tags[3][0][0][1])/HEIGHT_BETWEEN_TWO_ARUCO_INTERIORS_IN_MM
    width_offset = WIDTH_BETWEEN_BOARD_AND_ARUCO_EXTERIOR_IN_MM * pixels_per_mm
    height_offset = HEIGHT_BETWEEN_BOARD_AND_ARUCO_EXTERIOR_IN_MM * pixels_per_mm

    if method=="CORNER" or method: #default methods

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


    #Get max width
    width_down = np.sqrt(((src_pts[3][0] - src_pts[0][0]) ** 2) + ((src_pts[0][1] - src_pts[3][1]) ** 2))
    width_up = np.sqrt(((src_pts[2][0] - src_pts[1][0]) ** 2) + ((src_pts[1][1] - src_pts[2][1]) ** 2))
    width = int(max(width_up, width_down))

    #Get max height
    height_left = np.sqrt(((src_pts[1][0] - src_pts[0][0]) ** 2) + ((src_pts[0][1] - src_pts[1][1]) ** 2))
    height_right = np.sqrt(((src_pts[3][0] - src_pts[2][0]) ** 2) + ((src_pts[3][1] - src_pts[2][1]) ** 2))
    height = int(max(height_left, height_right))

    #Get destination points (where src points will be map in the output image)
    dst_pts = np.float32([[0, height-1],
                          [0, 0],
                          [width - 1, 0],
                          [width - 1, height - 1]])

    #Cast src_pts to float32
    src_pts = np.float32(src_pts)

    #Get matrix transformation
    transform_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

    #Save warp perspective parameters
    config["TRANSFORM_MATRIX"] = transform_matrix.tolist()
    config["REDRESS_SIZE"] = [width, height]
    with open(configuration_FILEPATH, "w") as json_file:
        json.dump(config, json_file)
    print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Calibration terminée.")

    #Prettify json file to be human readable
    with open(configuration_FILEPATH, 'r') as file:
            content = file.read()

            quote = False
            result = ''

            #Loop over each character to see if there is a quote
            for char in content:

                #Add a line jump after each coma placed before a quote
                if char == ',' and quote:
                    result += ',\n'
                else:
                    result += char

                #If a quote is detected directly look for the next char
                if char == '"' or char == "]":
                    quote = True
                    continue
                else :
                    quote = False

    #Save result
    with open(configuration_FILEPATH, 'w') as file:
        file.write(result)

    #Save a redressed image
    warped_image = cv2.warpPerspective(frame, transform_matrix, [width, height], flags=cv2.INTER_LINEAR)
    out_filename = FILE_PATH.split("_3_TRAITEMENT_d_IMAGES")[0]+"test_24/calibration_result.jpg"
    cv2.imwrite(filename=out_filename, img=warped_image)

    print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Résultat de la calibration disponnible dans test_24/calibration_result.jpg.")
    return True



"""
Redress an image using a transform matrix.
    frame (np.array)                ->      inputed image.
    transform_matrix (np.array)     ->      transformation matrix between the camera view and the board.
    size (list)                     ->      width and height of the outputed image.

Return function success and frame redressed.
"""
def redressImage(frame, transform_matrix, size):

    #Apply transformation matrix to the image
    warped_image = cv2.warpPerspective(frame, transform_matrix, size,flags=cv2.INTER_LINEAR)

    return True, warped_image



if __name__=="__main__":
    frame = cv2.imread(filename="/home/rayane/Royone/Inge3M/projet/Balise_2024/board.jpg")
    calibrateCameraUsingAruco(frame)