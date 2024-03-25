import argparse
import cv2
import numpy as np
import os
import sys


FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)


def detectAruco(filename:str, drawId=True, axis=False):
    """
    Detect ArUco markers.

    Parameters:
        - filename (str): name/path of the inputed image.
                          output will be store next to filename.
        - drawId (bool): draw a square and id on detected markers.
        - axis (bool): show axis of detected markers.

    Returns:
        - bool: function success.
        - list: corners positions.
        - list: tag ids.
        - str: image path.
    """

    #Get photo to parse
    img = cv2.imread(filename=filename)

    #In case there is no image
    if np.all(img) is None:
        print(f"Log [{FILE_NAME}]: Image non détectée.")
        sys.exit(0)

    #ArUco dictionary, all markers are sourced here,
    #4x4 are the officials Eurobot2024 cup Fra markers and since we just
    #want to get numbers between 1 and 90, 100 is the best size to optimize time research
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

    #Create parametors for detection
    aruco_parameters = cv2.aruco.DetectorParameters()

    #Create Aruco detector
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_parameters)

    #Detect ArUco markers
    corners, ids, rejected_corners = aruco_detector.detectMarkers(img)

    #Tell and quit if no ids
    if ids is None:
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Aucun tag n'a été détecté.")
        return False, None, None, filename

    #Create an image based on the inputed one
    output_image = img.copy()
    out_filename = filename
    suffixe = ""

    #Draw square on markers based on there positions previously detected
    if (drawId and ids.any()) :
        suffixe+="_aruco" #add suffixe to filename
        for k in range(len(ids)):
            corner_set = corners[k].astype(int) #cast to int

            #Draw rectangle
            cv2.polylines(output_image, [corner_set], isClosed=True, color=(0, 255, 0), thickness=6)

            #Get center
            center_x = int((corner_set[0][0][0] + corner_set[0][2][0]) / 2)
            center_y = int((corner_set[0][0][1] + corner_set[0][2][1]) / 2)


            #Put Id in center
            cv2.putText(output_image, f"{ids[k]}", (center_x-35, corner_set[0][0][1]+70),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 5, cv2.LINE_AA)

        #Save output image
        out_filename = filename.split('.')
        out_filename = f"{'.'.join(out_filename[:-1])}{suffixe}.{out_filename[-1]}"
        cv2.imwrite(filename=out_filename, img=output_image)


    
    #Flatten ids
    ids = list(ids.flatten())
    #Reshape corners to facilitate their navigation
    corners = list(map(lambda x : x.reshape((4,2)), corners))

    return True, corners, ids, out_filename


if __name__ == "__main__":

    import takePhoto#sorry this is the only method to overstep no module error

    #Create a parser for CLI options
    parser = argparse.ArgumentParser(prog=FILE_NAME,
                                     description="Detect ArUco tags on an image.")

    #All arguments available
    parser.add_argument("path_to_file",                                     #argument name
                        action="store",                                     #mendatory
                        type=str,                                           #must be string
                        help="Path to file")                                #help text

    parser.add_argument("-i",                           #short option
                       "--drawId",                      #long option
                       action="store_true",             #store true if called false if not
                       help="Draw axis on detected aruco") #help text

    parser.add_argument("-a",                           #short option
                       "--axis",                        #long option
                       action="store_true",             #store true if called false if not
                       help="Draw axis on detected aruco") #help text

    #Implement CLI options for photo (-p -t -q -dn)
    parser = takePhoto.initParser(parser)

    #Get all arguments
    args = parser.parse_args()
    filename = args.path_to_file #get filename
    drawId = args.drawId #get drawId
    axis = args.axis #get axis flag

    #Take a photo if mentioned
    if args.photo:
        #Only take options which are note None
        dict_param_takePhoto = {"name":filename,"tms":args.timeout,"quality":args.quality, "denoise":args.denoise_n}
        filename = takePhoto.takePhoto(**{k:v for k,v in dict_param_takePhoto.items() if v is not None})

    #Run function
    result=detectAruco(filename, drawId=drawId) #no axis for now
    print(result)
