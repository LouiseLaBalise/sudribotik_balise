import os
import sys
import argparse
import numpy as np
import cv2
import subprocess


FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)
MAX_NB_PHOTOS = 5000 #To limit ressources taken

"""
Take a photo.
    name (string)    ->       name of the photo
    tms (int)        ->       timeout in ms before the photo is taken
    quality (int)    ->       quality output out of 100%
    denoise (bool)   ->       denoise the photo taken
    anonymous (bool) ->       keep photo or just its pixels data value (for ROS). Don't work with denoise.

Return the path where the photo has been stored.
"""
def takePhoto(name="photo.jpg", tms=0.01, quality=50, denoise=False, anonymous=False):
    if tms<1 : tms=1 #timeout at 0 cause bugs
    #1 step : create path to store the img
    photo_idx = -1
    name, ext = name.split('.')
    ext = '.' + ext
    path_to_current_photo = ""
    suf=""
    if denoise: suf+="d"

    #Take an anonymous photo
    if anonymous:
        ret = False
        #Take photo
        process = subprocess.run(["raspistill", "-n", "-t", str(tms), "-o", "-"], stdout=subprocess.PIPE)
        image_bytes=process.stdout

        #Convert it to an Opencv image format
        nparr = np.frombuffer(image_bytes, np.uint8)
        image_opencv = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if image_opencv.any(): ret=True
        return ret, image_opencv


    #Each time a photo is taken its name increment
    while(os.path.isfile(path_to_current_photo) or path_to_current_photo==""):
        photo_idx +=1
        #if (photo_idx==0):photo_idx="" #dont put index on first photo
        path_to_current_photo = f"/home/ubuntu/Eurobot_2024/_3_TRAITEMENT_d_IMAGES/media/{name}_{photo_idx}{suf}{ext}"

    if (photo_idx > MAX_NB_PHOTOS):
        print(f"Log [{FILE_NAME}]: Nombre de photos max atteint, veuillez supprimer des photos pour en reprendre d'autres.")
        sys.exit()

    #2 step : send cmd line to take a photo
    exit_status = os.system(f"raspistill -o {path_to_current_photo} -t {tms} -q {quality}")
    if (not exit_status) :
        print(f"Log [{FILE_NAME}]: Photo taken (stored in {path_to_current_photo})")
    else :
        print(f"Log [{FILE_NAME}]: Il ya eu une erreur lors de la prise de photo (error from file : {os.path.basename(__file__)})")
        return None

    #Denoise
    if denoise:
        print(f"Log [{FILE_NAME}]: Denoising...")
        #Get image just taken
        img = cv2.imread(filename=path_to_current_photo)
        #Denoise it
        img = cv2.fastNlMeansDenoisingColored(img, None, 10, 10, 7, 21)
        #Save it
        cv2.imwrite(filename=path_to_current_photo, img=img)
        print(f"Log [{FILE_NAME}]: Denoise done.")

    return path_to_current_photo



"""
Initialize CLI options for other function implemanting takePhoto.py module

    parser (argparse.ArgumentParser)    ->     parser

Return the parser with new options.
"""
def initParser(parser:argparse.ArgumentParser):

    parser.add_argument("-p",                           #short option
                       "--photo",                       #long option
                       action="store_true",             #store true if called false if not
                       help="Take a photo.")            #help text

    parser.add_argument("-t",                                               #short option
                        "--timeout",                                        #long option
                        action="store",                                     #store argument
                        type=int,                                           #must be integers
                        metavar="tms",                                      #variables significations
                        help="Set a timeout for photo (must be -p).")       #help text

    parser.add_argument("-q",                                               #short option
                        "--quality",                                        #long option
                        action="store",                                     #store argument
                        type=int,                                           #must be integers
                        metavar="quality",                                  #variables significations
                        choices=range(0, 101),                              #only integers from 0 to 100
                        help="Set quality for photo (must be -p).")         #help text

    parser.add_argument("-dn",                          #short option
                       "--denoise_n",                   #long option
                       action="store_true",             #store true if called false if not
                       help="Denoise photo")            #help text

    return parser

    #Conditional statement to implement in functions calling the photo initParser
    #Take a photo if mentioned
    if args.photo:
        #Only take options which are note None
        dict_param_takePhoto = {"name":filename,"tms":args.timeout,"quality":args.quality, "denoise":args.denoise_n}
        filename = takePhoto.takePhoto(**{k:v for k,v in dict_param_takePhoto.items() if v is not None})







#In case user only want to take a photo from terminal
if __name__ == "__main__":
    takePhoto()
