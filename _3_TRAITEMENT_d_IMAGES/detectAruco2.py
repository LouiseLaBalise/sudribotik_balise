import cv2
import numpy as np
import getopt
import takePhoto
import sys
import os

# Constants
MARKER_LENGTH = 0.02
AXIS_LENGTH = 0.01

def usage():
    print("Detect ArUco markers.\n"
    ".detecAruco.py <path_to_image.jpg> [option] [argument]\n"
    "\t\tor\n"
    "./detecAruco.py [option] [argument]\n"
    "\t-h, --help, no argument -> Display help.\n"
    "\t-p, --photo, no argument -> Take a photo before parsing.\n"
    "\t-a, --axis, no argument -> Draw axis on found markers.\n"
    "\t-n, --name, <filename> -> Set a new name.\n"
    "\t-t, --timeout, <time in ms> -> Set a timeout.\n"
    "\t-q, --quality, <quality out of 100%> -> Set quality for the photo.\n")

def main(argv):

    #Test arguments
    if not (argv):
        usage()
        print("\nError : No arguments detected\n")
        sys.exit(1)

    #Get photo path if the first option is not an option
    if (argv[0][0]!='-'):
        image_path = argv.pop(0)

    #Option and arguments from cli
    options, arguments = getopt.getopt(argv, "hpan:t:q:", ["help=",
                                                        "photo=",
                                                        "axis=",
                                                        "name=",
                                                        "timeout=",
                                                        "quality="])

    dict_param_takePhoto = {"name":None,"tms":None,"quality":None}
    flag_photo = False
    flag_axis = False


    for opt, arg in options: #loop through cli options
        if opt in ("-h", "-help"):
            usage()
        elif opt in ("-p", "--photo"):
            flag_photo = True
        elif opt in ("-a", "--axis"):
            flag_axis = True
        elif opt in ("-n", "--name"):
            dict_param_takePhoto["name"] = arg
        elif opt in ("-t", "--timeout"):
            dict_param_takePhoto["tms"] = arg
        elif opt in ("-q", "--quality"):
            dict_param_takePhoto["quality"] = arg
        else :
            print("use ./detectAruco -h to see usage.")

    if flag_photo:
        #Clean dict_param_takePhoto from None variables
        dict_param_takePhoto = {k:v for k,v in dict_param_takePhoto.items() if v is not None}

        #Take photo
        image_path = takePhoto.takePhoto(**dict_param_takePhoto)

    # Read the image
    img = cv2.imread(image_path)

    # Check if the image was successfully read
    if img is None:
        print("Image not detected.")
        sys.exit(0)

    # Your ArUco detection code here
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

    #Create parametors for detection
    aruco_parameters = cv2.aruco.DetectorParameters_create()

    #Detect ArUco markers
    corners, ids, rejected_corners = cv2.aruco.detectMarkers(img,
                                                             aruco_dict,
                                                             parameters=aruco_parameters)

    #Tell if no ids
    if (not ids):
        print("No ids detected.")
    
    # Create an image based on the input one
    output_image = img.copy()

    # Draw square on markers based on their positions previously detected
    if corners and ids:
        cv2.aruco.drawDetectedMarkers(output_image, corners, ids)

    # Draw axis on markers
    if flag_axis and ids:
        #Matrix and Distortion coeff were calculated with calibration.py
        matrix_coeff = [ 2.5959453540558084e+03, 0., 1.7015437024875371e+03,
                        0.,2.5981150581304091e+03, 1.2459640171617973e+03,
                        0., 0., 1. ]
        matrix_coeff = np.array(matrix_coeff).reshape(3, 3)

        distortion_coeff = np.array([ 2.0203034110644411e-01, -3.8937675520950621e-01,
       2.8240913779598222e-03, 5.4936365788275619e-03,
       -9.1462806356767012e-02 ])
        
        for i in range(len(ids)):
            # Estimate their position
            rvec, tvec, marker_points = cv2.aruco.estimatePoseSingleMarkers(corners[i], MARKER_LENGTH, matrix_coeff, distortion_coeff)
            #(rvec-tvec).any()
            # Then draw their axis
            cv2.aruco.drawAxis(output_image, matrix_coeff, distortion_coeff, rvec, tvec, AXIS_LENGTH)

    # Save output image
    base_name, extension = os.path.splitext(image_path)
    out_img_path = f"{base_name}_parsed{extension}"
    cv2.imwrite(out_img_path, output_image)
    print(f"Photo parsed (stored in {out_img_path})")

if __name__ == "__main__":
    main(sys.argv[1:])
