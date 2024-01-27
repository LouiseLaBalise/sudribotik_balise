#!/usr/bin/env python
import os
import sys
import time
import json
import rospy
import cv2
import math

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)

sys.path.insert(1, FILE_PATH.split("_2_ROS")[0]) #add parent folder to python path
from _3_TRAITEMENT_d_IMAGES import ros_redressBoardUsingAruco, ros_detectAruco, ros_detectColor
from balise_msgs.msg import PositionPx, PositionPxWithType, ArrayPositionPx, ArrayPositionPxWithType


"""
Publish position of Robots, Elements and Arucos on the board
"""
def publisher():
 
    #Get Aruco constants
    aruco_id_filepath = FILE_PATH.split("_2_ROS")[0]+"init/gameElements_identification.json"
    with open (aruco_id_filepath, "r") as f:
        aruco_tags_const = json.load(f)

    # This node will publish blue and yellow robot position
    robotsPos_pub = rospy.Publisher("balise/position/robots", ArrayPositionPxWithType, queue_size=10)

    # This node will publish all blue and yellow PAMIs position
    pamisPos_pub = rospy.Publisher("balise/position/pamis", ArrayPositionPxWithType, queue_size=10)

    # This node will publish all fragile and resistant plants
    plantsPos_pub = rospy.Publisher("balise/position/plants", ArrayPositionPx, queue_size=10)

    # This node will publish all iron pots
    #potPos_pub = rospy.Publisher("balise/position/pots", ArrayPositionPx, queue_size=10)

    #Tell node name to rospy
    rospy.init_node("b_cam_position", anonymous=True)

    #Set publish rate
    rate = rospy.Rate(10) #en hz


    #Open camera for video capturing
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    while not rospy.is_shutdown():

        ret,frame = cap.read() #read an image from camera

        #Go to next loop if there is no image to read
        if not ret:
            print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Problème avec la caméra.")
            continue

        #This node will publish all blue, yellow and mid solar panel position and orientation
        # before image is redressed.
        solarPos_pub = rospy.Publisher("balise/position/solarpanel", ArrayPositionPxWithType, queue_size=10)
        solarPos_pub.publish(getPositionSolarPanelMsg(frame,
                                                      aruco_tags_const["SOLAR_PANEL_ID"][0]))

        #Redress board
        ret=True#, frame_redressed = ros_redressBoardUsingAruco.redressBoardUsingAruco(frame)        
        #Go to next loop if board can't be redressed
        if not ret:
            print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Impossible de redresser le plateau de jeu.")
            continue


        #Search for Aruco tags
        ret, corners, ids = ros_detectAruco.detectAruco(frame)#frame_redressed
        #Go to next loop if Aruco cannot be detected
        if not ret:
            print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Aucun Aruco détecté.")
            continue

        robotsPos_pub.publish(getPositionRobotMsg(corners, ids,
                                                  aruco_tags_const["BLUE_ROBOT_MAIN_ID"][0],
                                                  aruco_tags_const["YELLOW_ROBOT_MAIN_ID"][0]))
        pamisPos_pub.publish(getPositionPamiMsg(corners, ids,
                                                  aruco_tags_const["BLUE_PAMI_IDS"],
                                                  aruco_tags_const["YELLOW_PAMI_IDS"]))
        plantsPos_pub.publish(getPositionPlantMsg(frame,
                                                  aruco_tags_const["PLANT_HSV_RANGE"],
                                                  aruco_tags_const["PLANT_HSV_SURFACE"]))
        #potPos_pub.publish(getPositionPotMsg())

        rate.sleep() #wait according to publish rate

    cap.release() #free camera




"""
Calculate center of an Aruco tag.
    corners (list)    ->      all detected aruco corners.
    index (int)       ->     index of the tag.

Return a tuple (int, int).
"""
def getCenterArucoTag(corners, index):
    #Get x pos (these could be inversed but not important here)
    low_x = corners[index][0][0]
    high_x = corners[index][2][0]

    #Get y pos
    low_y = corners[index][0][1]
    high_y = corners[index][2][1]

    return ((low_x+high_x)//2 , (low_y+high_y)//2)


"""
Get message for realtime position of all robots.
    corners (list)      ->      all detected aruco corners.
    ids (list)          ->      all detected aruco ids.
    blue_rid (int)      ->      blue robot id.
    yellow_rid (int)    ->      yellow robot id.

Return an ArrayPositionPxWithType msg.
"""
def getPositionRobotMsg(corners, ids, blue_rid, yellow_rid):
    msg = [] #create msg
    blue_msg = PositionPxWithType() 
    yellow_msg = PositionPxWithType()

    #Blue robot has been detected
    if blue_rid in ids:
        #Get center of tag
        blue_msg.x, blue_msg.y = getCenterArucoTag(corners, ids.index(blue_rid))
        blue_msg.theta = 0 #No angle for now        
        blue_msg.type = "blue" #Robot is blue
        #Append the blue pos into msg
        msg.append(blue_msg)
        
    
    #Yellow robot has been detected
    if yellow_rid in ids:
        #Get center of the tag
        yellow_msg.x, yellow_msg.y = getCenterArucoTag(corners, ids.index(yellow_rid))
        yellow_msg.theta = 0 #No angle for now        
        yellow_msg.type = "yellow" #Robot is yellow
        #Append the yellow pos into msg
        msg.append(yellow_msg)

    return msg


"""
Get message for realtime position of all pamis.
    corners (list)      ->      all detected aruco corners.
    ids (list)          ->      all detected aruco ids.
    blue_pids (list)    ->      blue pami ids.
    yellow_pids (list)  ->      yellow pami ids.

Return an ArrayPositionPxWithType msg.
"""
def getPositionPamiMsg(corners, ids, blue_pids, yellow_pids):
    msg = [] #create msg
    blue_msg = PositionPxWithType() 
    yellow_msg = PositionPxWithType()

    #Select only pamis's corners
    all_blue_pami_ids = list( set(ids) & set(blue_pids) )
    all_yellow_pami_ids = list( set(ids) & set(yellow_pids) )

    #Add all blue pamis position in msg
    for blue_pami_id in all_blue_pami_ids :
        #Get center of tag        
        blue_msg.x, blue_msg.y = getCenterArucoTag(corners, ids.index(blue_pami_id))
        blue_msg.theta = 0 #No angle for now
        blue_msg.type = "blue" #Robot is an blue
        #Append the blue pos into msg
        msg.append(blue_msg)
        
    
    #Add all yellow pamis position in msg
    for yellow_pami_id in all_yellow_pami_ids :
        #Get center of tag
        yellow_msg.x, yellow_msg.y = getCenterArucoTag(corners, ids.index(yellow_pami_id))
        yellow_msg.theta = 0 #No angle for now        
        yellow_msg.type = "yellow" #Robot is an yellow
        #Append the yellow pos into msg
        msg.append(yellow_msg)

    return msg



"""
Get message for realtime position of all plants.
    frame (numpy.ndArray)         ->      data array of the image.
    plant_hsv_range (list)        ->      plants hsv ranges.
    plant_hsv_surface (list)      ->      plants hsv surfaces.

Return an ArrayPositionPx msg.
"""
def getPositionPlantMsg(frame, plant_hsv_range, plant_hsv_surface):
    msg = [] #create msg
    plant_msg = PositionPx()

    hue = (plant_hsv_range[0][0], plant_hsv_range[1][0]) #store hue range
    sat = (plant_hsv_range[0][1], plant_hsv_range[1][1]) #store saturation range
    val = (plant_hsv_range[0][2], plant_hsv_range[1][2]) #store value range
    minS = plant_hsv_surface[0] #store min surface
    maxS = plant_hsv_surface[1] #store max surface

    #Get corners of detected colors
    ret, colors_detected_corners = ros_detectColor.colorDetection(frame, hue, sat, val, minS, maxS)

    #Ally robot has been detected
    for corner in colors_detected_corners :
        #Get center of tag        
        plant_msg.x = (corner[0] + corner[2]) //2
        plant_msg.y = (corner[1] + corner[3]) //2
        plant_msg.theta = 0 #No angle for now
        #Append the plant pos into msg
        msg.append(plant_msg)

    return msg


"""
Get angle rotation of an aruco tag given its corners.
    corner(numpy.ndArray)    ->    corner of a aruco tag.

Return an angle in degrees. From 0° to 360°.
"""
def getAngle(corner):
    #Get points
    top_left, top_right, bot_right, bot_left = corner

    #Get parameters for atan func
    adjacent = top_right[0] - top_left[0]
    opposite = top_left[1] - top_right[1]

    #Get angle
    angle = math.atan2(adjacent, opposite)

    #Cast from radians to degrees 
    angle = angle * 180/math.pi
    
    #And add pi/2 (i fucking dont know why)
    angle+=90

    #Force range from 0° to 360°
    if angle < 0 :
        angle+=360

    return int(angle)

"""
Get message for realtime position of all solar panels.
Also redistibute panels into 3 groups : blue, yellow or mid
    frame (numpy.ndArray)         ->      data array of the image.
    solar_id (int)      ->      solar panel id.

Return an ArrayPositionPxWithType msg.
"""
def getPositionSolarPanelMsg(frame, solar_id):
    msg = [] #create msg
    panel_msg = PositionPxWithType()

    """#pour tester sur docker
    frame = cv2.imread(filename="/Louise_eurobot_2024/board.jpg")
    ret, frame = ros_redressBoardUsingAruco.redressBoardUsingAruco(frame)
    t=1#numero de tag à tester"""

    #Search for Aruco tags
    ret, corners, ids = ros_detectAruco.detectAruco(frame)

    #Quit if no ids
    if not ids :
        return msg
    
    #Collect all panel corners
    solarpanel_corners = [ corners[k] for k,id in enumerate(ids) if id==solar_id ]

    #Loop over evry detected panel  
    for solar_panel_corner in solarpanel_corners:
        #Get its position
        panel_msg.x = (solar_panel_corner[0][0] + solar_panel_corner[2][0]) //2
        panel_msg.y = (solar_panel_corner[0][1] + solar_panel_corner[2][1]) //2

        #Get its angle
        panel_msg.theta = getAngle(solar_panel_corner)

        #Place panel in a group based on its position on raw frame
        frame_length_divided_by_three = frame.shape[1] // 3
        
        #Case left so yell
        if panel_msg.x < frame_length_divided_by_three:
            panel_msg.type = "yellow"
        
        #Case middle so mid
        elif panel_msg.x < frame_length_divided_by_three*2:
            panel_msg.type = "mid"

        #Case right so blue
        else :
            panel_msg.type = "blue"

        """#pour tester sur docker
        if t==3:
            cv2.circle(frame, (panel_msg.x, panel_msg.y), 8, (0, 0, 255), -1)
            cv2.imwrite(filename="/Louise_eurobot_2024/test_board.jpg", img=frame)
            print(panel_msg)
            sys.exit(0)
        
        t+=1"""

        msg.append(panel_msg)
    
    return msg



if __name__ == '__main__':
    publisher()
