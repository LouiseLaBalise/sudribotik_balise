import os
import sys
import argparse

MAX_NB_PHOTOS = 50 #To limit ressources taken

"""Command to take a photo
    -name(string) : name of the photo
    -tms(int) : time in ms required to take the photo
    -quality(int) : quality output out of 100%

Return the path where the photo has been stored.
"""
def takePhoto(name="photo.jpg", tms=50, quality=50):
    #1 step : create path to store the img
    photo_idx = -1
    name, ext = name.split('.')
    ext = '.' + ext
    path_to_current_photo = ""

    #Each time a photo is taken its name increment
    while(os.path.isfile(path_to_current_photo) or path_to_current_photo==""):
        photo_idx +=1
        #if (photo_idx==0):photo_idx="" #dont put index on first photo
        path_to_current_photo = f"/home/ubuntu/Eurobot_2024/_3_TRAITEMENT_d_IMAGES/media/{name}_{photo_idx}{ext}"

    if (photo_idx > MAX_NB_PHOTOS):
        print("Nombre de photos max atteint, veuillez supprimer des photos pour en reprendre d'autres.")
        sys.exit()

    #2 step : send cmd line to take a photo
    exit_status = os.system(f"raspistill -o {path_to_current_photo} -t {tms} -q {quality}")
    if (not exit_status) :
        print(f"Photo taken (stored in {path_to_current_photo})")
    else :
        print(f"Il ya eu une erreur lors de la prise de photo (error from file : {os.path.basename(__file__)})")
        return None

    return path_to_current_photo



"""Initialize CLI options for other function implemanting takePhoto.py module
    -parser(argparse.ArgumentParser) : parser

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
    
    return parser







#In case user only want to take a photo from terminal
if __name__ == "__main__":
    takePhoto()
