import os
import sys
import cv2
import numpy as np
import argparse



FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)


def colorDetection(filename:str, hue:tuple, saturation=(50, 255), value=(50, 255),
                   contoured=False, rectangled=False, denoise=False, minSurface=0, maxSurface=99999):
    """
    Detect HSV color range on an image.

    Parameters:
        - filename (str): name of the inputed image.
        - hue (tuple): range of hue values on the hue compass between 0 and 180.
        - saturation (tuple): range of saturation between 0 and 255.
        - values (tuple): range of values between 0 and 255.
                          output will be store next to filename.
        - contoured (bool): flag for contouring detected colors on the output.
        - rectangled (bool): flag for drawing a rectangle on detected colors on the output.
        - denoise (bool): flag for denoising inputed image.
        - minSurface (int): number of minimumu pixels for a detected area to be take into account.
        - maxSurface (int): number of maximum pixels for a detected area to be take into account.

    Returns:
        - bool: function success.
        - list: positions of all area detected.
        - str: image path.
    """

    ret = False #return succes

    #List of positions for detected colors
    color_positions = []

    #Get frame
    frame = cv2.imread(filename=filename)

    suffixe = ""
    #Set suffixe parameters
    if saturation!=(50,255): suffixe+="_sat"+str(saturation[0])+'-'+str(saturation[1])
    if value!=(50,255): suffixe+="_val"+str(value[0])+'-'+str(value[1])
    if minSurface!=0: suffixe+="_mi"+str(minSurface)
    if maxSurface!=99999: suffixe+="_mx"+str(maxSurface)
    #Denoise or not
    if denoise:
        suffixe+='_denois'
        frame = cv2.fastNlMeansDenoisingColored(frame, None, 10, 10, 7, 21)
    if contoured: suffixe+="_contr"
    if rectangled: suffixe+="_rect"

    #Convert image from original colorspace bgr to hsv
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #Define hsv limits to see
    lower_limit_hsv = np.array([hue[0], saturation[0], value[0]])
    upper_limit_hsv = np.array([hue[1], saturation[1], value[1]])

    #Create mask to only show hsv limits calculated from bgr
    mask = cv2.inRange(hsv_frame, lower_limit_hsv, upper_limit_hsv)

    #In case of red hue (or if hue value range go from before 0 to after 0 )
    if (hue[0]>hue[1]):
        #Mask before 0
        lower_limit_hsv1 = np.array([hue[0], saturation[0], value[0]])
        upper_limit_hsv1 = np.array([179, saturation[1], value[1]])
        sub_mask1 = cv2.inRange(hsv_frame, lower_limit_hsv1, upper_limit_hsv1)
        #Mas after 0
        lower_limit_hsv2 = np.array([0, saturation[0], value[0]])
        upper_limit_hsv2 = np.array([hue[1], saturation[1], value[1]])
        sub_mask2 = cv2.inRange(hsv_frame, lower_limit_hsv2, upper_limit_hsv2)
        #Summ masks
        mask = sub_mask1+sub_mask2

    #Draw contours if they are contours deyected on the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        #Filter on minSurface or maxSurface
        cnt_px = cv2.contourArea(cnt)
        if cnt_px<=minSurface or cnt_px>=maxSurface :
            continue

        x, y, w, h = cv2.boundingRect(cnt) #get rectangles coordinates
        color_positions.append((x, y, x+w, y+h))

        if contoured :
            cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 3)#draw contours

        if rectangled :
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)#draw rectangles

    #Save output
    out_filename = filename.split('.')
    out_filename = f"{'.'.join(out_filename[:-1])}_hue{hue[0]}-{hue[1]}{suffixe}.{out_filename[-1]}"
    cv2.imwrite(filename=out_filename, img=frame)

    ret = True
    return ret, color_positions, out_filename


if __name__=="__main__":

    import takePhoto#sorry this is the only method to overstep no module error

    #Create a parser for CLI options
    parser = argparse.ArgumentParser(prog=FILE_NAME,
                                     description="Detect specified range of color on an image.")

    #All arguments available
    parser.add_argument("path_to_file",                                     #argument name
                        action="store",                                     #mendatory
                        type=str,                                           #must be string
                        help="Path to file")                                #help text

    parser.add_argument("hue",                                              #argument name
                        action="store",                                     #mendatory
                        nargs='+',                                          #must be 2 arguments
                        type=int,                                           #must be integers
                        metavar=("hue_min, hue_max"),                       #variables significations
                        choices=range(0, 181),                              #only integers from 0 to 180
                        help="Min and Max hue range")                       #help text

    parser.add_argument("-s",                                               #short option
                        "--saturation",                                     #long option
                        action="store",                                     #store arguments
                        nargs=2,                                            #must be 2 arguments
                        type=int,                                           #must be integers
                        metavar=('min', 'max'),                             #variables significations
                        choices=range(0, 256),                              #only integers from 0 to 255
                        default=(50, 255),                                  #default values
                        help="Specify saturation out of 255")               #help text

    parser.add_argument("-v",                                               #short option
                        "--value",                                          #long option
                        action="store",                                     #store arguments
                        nargs=2,                                            #must be 2 arguments
                        type=int,                                           #must be integers
                        metavar=('min', 'max'),                             #variables significations
                        choices=range(0, 256),                              #only integers from 0 to 255
                        default=(50, 255),                                  #default values
                        help="Specify value out of 255")                    #help text

    parser.add_argument("-r",                           #short option
                       "--rectangled",                  #long option
                       action="store_true",             #store true if called false if not
                       help="Draw a rectangular shape on all detected colors") #help text

    parser.add_argument("-c",                           #short option
                       "--contoured",                   #long option
                       action="store_true",             #store true if called false if not
                       help="Draw the contours on all detected colors") #help text

    parser.add_argument("-d",                           #short option
                       "--denoise",                     #long option
                       action="store_true",             #store true if called false if not
                       help="Denoise image before color detection") #help text

    parser.add_argument("-mi",                          #short option
                       "--minSurface",                  #long option
                       action="store",                  #store an argument
                       type=int,                        #must be an integer
                       default=0,                       #default values
                       help="Filter by setting a minimal surface in pixels") #help text

    parser.add_argument("-mx",                          #short option
                       "--maxSurface",                  #long option
                       action="store",                  #store an argument
                       type=int,                        #must be an integer
                       default=99999,                   #default values
                       help="Filter by setting a maximal surface in pixels") #help text

    #Implement CLI options for photo (-p -t -q -dn)
    parser = takePhoto.initParser(parser)

    #Get all arguments
    args = parser.parse_args()
    filename = args.path_to_file #get filename
    if len(args.hue) != 2 :
        print(f"Log [{FILE_NAME}]: Error : must be 2 hue values min and max. Check --help for details." )
        sys.exit(0)
    hue_min, hue_max = args.hue[0], args.hue[1] #get hue range
    sat_min, sat_max = args.saturation[0], args.saturation[1] #get saturation range
    val_min, val_max = args.value[0], args.value[1] #get value range
    rectangled = args.rectangled #get rectangled
    contoured = args.contoured #get contoured
    denoise = args.denoise #get denoise
    minSurface = args.minSurface #get minSurface
    maxSurface = args.maxSurface #get maxSurface

    #Take a photo if mentioned
    if args.photo:
        #Only take options which are note None
        dict_param_takePhoto = {"name":filename,"tms":args.timeout,"quality":args.quality, "denoise":args.denoise_n}
        filename = takePhoto.takePhoto(**{k:v for k,v in dict_param_takePhoto.items() if v is not None})


    #Run function
    result=colorDetection(filename, (hue_min, hue_max), (sat_min, sat_max), (val_min, val_max),
                   contoured=contoured, rectangled=rectangled, denoise=denoise, minSurface=minSurface, maxSurface=maxSurface)
    print(result)
