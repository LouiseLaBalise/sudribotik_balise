import cv2
import numpy as np
import takePhoto
import argparse


"""
Straighten a board with 4 Aruco tags.
    filename (str)      ->      name of the inputed image.
    path (str)          ->      path of the filename from current working dir to filename.
                                output will be store next to filename. 
    corner_ids (tuple)  ->      ids of the 4 mendatory ArUco tags                    
    method (str)        ->      chose method, CORNER is by default.

Return function success and new filename.
"""
def straightenBoardUsingAruco(filename, path="media/", corner_ids = (20, 21, 23, 22), method="CORNER"):

    #Load image
    image = cv2.imread(filename=path+filename)

    #Quit if not 4 tags in params
    if len(corner_ids)!=4:
        print("Il faut 4 ids de tags ArUco. Impossible de redresser l'image.")
        return False, None

    #Create ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    #Create parametors for detection
    aruco_parameters = cv2.aruco.DetectorParameters()
    #Create Aruco detector
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_parameters)
    #Detect ArUco markers
    corners, ids, rejected_corners = aruco_detector.detectMarkers(image)

    #All 4 Aruco markers are needed to perform image straightening
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
        print(f"Tag(s) {corner_ids_no_detected} non detecté(s). Impossible de redresser l'image.")
        return False, None
    
    #Sort corners pos to have [bottom-left, bottom-right, top-right, top-left]
    sorted_corners = corners_pos_detected_ids
    sorted_corners = sorted(sorted_corners, reverse=True, key=lambda coord: coord[0][0][1])#Reverse sort by y axis    
    sorted_corners[:2] = sorted(sorted_corners[:2], key=lambda coord: coord[0][0][0])#Normal sort by x axis for the bottom    
    sorted_corners[2:] = sorted(sorted_corners[2:], reverse=True, key=lambda coord: coord[0][0][0])#Reverse sort by x axis for the top

    #Straighten board using center of each Aruco tags NOT WORKING RIGHT NOW
    if method=="CENTER":
        #Get source points (where points are on the distorded board) 
        src_pts = np.float32([])

        for tag_corners in sorted_corners: 
            #1st calc center of each tag
            center_x = int((tag_corners[0][0][0] + tag_corners[0][2][0]) / 2)
            center_y = int((tag_corners[0][0][1] + tag_corners[0][2][1]) / 2)

            #2nd add each tag center to src points
            src_pts = np.append(src_pts, [center_x, center_y])
        
        #3rd reshape src points to separate each tag
        src_pts = src_pts.reshape(-1, 2)

        #4th get max width - src_pts[up or down and left or right][x=0 or y=1]
        width_up = np.sqrt(((src_pts[0][0] - src_pts[3][0]) ** 2) + ((src_pts[0][1] - src_pts[3][1]) ** 2))
        width_down = np.sqrt(((src_pts[1][0] - src_pts[2][0]) ** 2) + ((src_pts[1][1] - src_pts[2][1]) ** 2))
        max_width = max(int(width_up), int(width_down))

        #5th get max height
        height_up = np.sqrt(((src_pts[0][0] - src_pts[1][0]) ** 2) + ((src_pts[0][1] - src_pts[1][1]) ** 2))
        height_down = np.sqrt(((src_pts[2][0] - src_pts[3][0]) ** 2) + ((src_pts[2][1] - src_pts[3][1]) ** 2))
        max_height = max(int(height_up), int(height_down))

        #6th get destination points (where src points will be map in the output image) and cast src_pts to float32
        dst_pts = np.float32([[0, 0],
                              [0, max_height - 1],
                              [max_width - 1, max_height - 1],
                              [max_width - 1, 0]])
        src_pts = np.float32(src_pts)    


    elif method=="CORNER" or method: #default method
        src_pts = np.float32([])            

        #Add each tag outter corner to src points
        src_pts = np.append(src_pts, sorted_corners[0][0][0])
        src_pts = np.append(src_pts, sorted_corners[1][0][3])
        src_pts = np.append(src_pts, sorted_corners[2][0][2])
        src_pts = np.append(src_pts, sorted_corners[3][0][1])

        #Reshape src points to separate each tag
        src_pts = src_pts.reshape(-1, 2)
        

        # Draw the point on the image
        #cv2.circle(image, np.int32(src_pts[0]), 15, (255, 0, 0), -1)  # -1 fills the circle with color
        #cv2.circle(image, np.int32(src_pts[1]), 15, (0, 255, 0), -1)  # -1 fills the circle with color
        #cv2.circle(image, np.int32(src_pts[2]), 15, (0, 0, 255), -1)  # -1 fills the circle with color
        #cv2.circle(image, np.int32(src_pts[3]), 15, (0, 0, 0), -1)  # -1 fills the circle with color

        #Get max height - src_pts[up or down and left or right][x=0 or y=1]
        height_up = np.sqrt(((src_pts[0][0] - src_pts[3][0]) ** 2) + ((src_pts[0][1] - src_pts[3][1]) ** 2))
        height_down = np.sqrt(((src_pts[1][0] - src_pts[2][0]) ** 2) + ((src_pts[1][1] - src_pts[2][1]) ** 2))
        max_height = max(int(height_up), int(height_down))

        #Get max width
        width_up = np.sqrt(((src_pts[0][0] - src_pts[1][0]) ** 2) + ((src_pts[0][1] - src_pts[1][1]) ** 2))
        width_down = np.sqrt(((src_pts[2][0] - src_pts[3][0]) ** 2) + ((src_pts[2][1] - src_pts[3][1]) ** 2))
        max_width = max(int(width_up), int(width_down))

        #Get destination points (where src points will be map in the output image) and cast src_pts to float32
        dst_pts = np.float32([[0, max_height-1],
                              [max_width - 1, max_height - 1],
                              [max_width - 1, 0],
                              [0, 0]])
        src_pts = np.float32(src_pts)

    #Get matrix transformation
    transform_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

    #Apply transformation matrix to the image
    warped_image = cv2.warpPerspective(image, transform_matrix, (max_width, max_height),flags=cv2.INTER_LINEAR)

    
    #Save image
    out_filename = filename.split('.')
    out_filename = f"{out_filename[0]}_redressed.{out_filename[1]}"
    cv2.imwrite(filename=path+out_filename, img=warped_image)
    return True, out_filename




if __name__ == "__main__":

    #Create a parser for CLI options
    parser = argparse.ArgumentParser(prog="straightenBoardUsingAruco.py",
                                     description="It's literrally in the name.") 

    #All arguments available    
    parser.add_argument("path_to_file",                                     #argument name
                        action="store",                                     #mendatory
                        type=str,                                           #must be string
                        help="Path to file")                                #help text
    
    parser.add_argument("-i",                                               #short option
                        "--ids",                                            #long option
                        action="store",                                     #store arguments
                        nargs=4,                                            #must be 4 arguments
                        type=int,                                           #must be integers
                        metavar=('id1', 'id2', 'id3', 'id4'),               #variables significations
                        default=(20, 21, 22, 23),                           #default values
                        help="Id number of corners needed to straighten the image")      #help text
    
    parser.add_argument("-m",                                               #short option
                        "--method",                                         #long option
                        action="store",                                     #store arguments
                        type=str,                                           #must be string
                        choices=("CORNER", "CENTER"),                       #only available choices  
                        default="CORNER",                                   #default value
                        help="Specify method of straightenning")            #help text

    
    #Implement CLI options for photo (-p -t -q -dn)
    parser = takePhoto.initParser(parser)

    #Get all arguments
    args = parser.parse_args()
    filename = args.path_to_file.split('/')[-1] #get filename
    corner_ids = args.ids #get corner ids
    method = args.method #get method

    if len(args.path_to_file.split('/')) >=2:   #get path if there is one
        image_path = '/'.join(args.path_to_file.split('/')[:-1])+'/'
    else :
        image_path = ""

    #Take a photo if mentioned
    if args.photo:
        #Only take options which are note None
        dict_param_takePhoto = {"name":filename,"tms":args.timeout,"quality":args.quality, "denoise":args.denoise_n}
        takePhoto.takePhoto(**{k:v for k,v in dict_param_takePhoto.items() if v is not None})
        image_path = "media/"

    straightenBoardUsingAruco(filename, path=image_path, corner_ids=corner_ids, method=method)