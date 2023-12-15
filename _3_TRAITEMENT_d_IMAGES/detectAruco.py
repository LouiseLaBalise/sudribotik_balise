import cv2
import numpy as np
import sys
import argparse

"""
Detect ArUco markers.
    filename (str)      ->      name/path of the inputed image.
                                output will be store next to filename.
    drawId (bool)       ->      draw a square and id on detected markers.
    axis (bool)         ->      show axis of detected markers.

Return Aruco succes, corners positions, their ids, image path.
"""
def detectAruco(filename:str, drawId=False, axis=False):

    #Get photo to parse
    img = cv2.imread(filename=filename)

    #In case there is no image
    if np.all(img) is None:
        print("Image non détectée.")
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
    try:
        if (not ids.any()):
            print("\nAucun tag n'a été détecté.")
            return False, None, None, filename

    #ids.any() retun an AttributeError if ids is None so
    # we repeat the action in the except block as if it has pass the test
    #the reason beeing  we cannot test a numpy array without any() or all()
    #todo : test if None.all() is considered
    except AttributeError as e:
            print("\nAucun tag n'a été détecté.")
            return False, None, None, filename

    #Create an image based on the inputed one
    output_image = img.copy()
    out_filename = filename

    #Draw square on markers based on there positions previously detected
    if (drawId and ids.any()) :
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
        out_filename = f"{'.'.join(out_filename[:-1])}_aruco.{out_filename[-1]}"
        cv2.imwrite(filename=out_filename, img=output_image)

    #Draw axis on markers
    if (axis and ids.any() and False):

        #Matrix and Distortion coeff were calculated with calibration.py
        matrix_coeff = [ 2.5959453540558084e+03, 0., 1.7015437024875371e+03,
                        0.,2.5981150581304091e+03, 1.2459640171617973e+03,
                        0., 0., 1. ]
        matrix_coeff = np.array(matrix_coeff).reshape(3, 3)

        distortion_coeff = np.array([ 2.0203034110644411e-01, -3.8937675520950621e-01,
       2.8240913779598222e-03, 5.4936365788275619e-03,
       -9.1462806356767012e-02 ])

        for i in range(len(ids)):

            #Estimate their position
            rvec, tvec, marker_points = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02,
                                                                            matrix_coeff,
                                                                            distortion_coeff)
            #(rvec-tvec).any()
            #Then draw their axis
            cv2.aruco.drawAxis(output_image, matrix_coeff, distortion_coeff, rvec, tvec, 0.01)

    return True, corners, ids, out_filename


if __name__ == "__main__":

    import takePhoto#sorry this is the only method to overstep no module error

    #Create a parser for CLI options
    parser = argparse.ArgumentParser(prog="detectAruco.py",
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
    detectAruco(filename, drawId=drawId) #no axis for now
