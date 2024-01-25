#!/usr/bin/env python
import os
import sys
import time
import json
import rospy
import cv2

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

    # This node will publish ally and enemy robot position
    robotsPos_pub = rospy.Publisher("balise/position/robots", ArrayPositionPxWithType, queue_size=10)

    # This node will publish all allies and enemies PAMIs position
    pamisPos_pub = rospy.Publisher("balise/position/pamis", ArrayPositionPxWithType, queue_size=10)

    # This node will publish all fragile and resistant plants
    plantsPos_pub = rospy.Publisher("balise/position/plants", ArrayPositionPx, queue_size=10)

    # This node will publish all iron pots
    #potPos_pub = rospy.Publisher("balise/position/pots", ArrayPositionPx, queue_size=10)

    # This node will publish all allies, enemies and mid solar panel position and orientation
    #solarPos_pub = rospy.Publisher("balise/position/solarpanel", ArrayPositionPxWithType, queue_size=10)

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
                                                  aruco_tags_const["ALLY_ROBOT_MAIN_IDS"],
                                                  aruco_tags_const["ENEMY_ROBOT_MAIN_IDS"],))
        pamisPos_pub.publish(getPositionPamiMsg(corners, ids,
                                                  aruco_tags_const["ALLY_ROBOT_MAIN_IDS"],
                                                  aruco_tags_const["ENEMY_ROBOT_MAIN_IDS"],))
        plantsPos_pub.publish(getPositionPlantMsg(frame,
                                                  aruco_tags_const["PLANT_HSV_RANGE"],
                                                  aruco_tags_const["PLANT_HSV_SURFACE"],))
        #potPos_pub.publish(getPositionPotMsg())
        #solarPos_pub.publish(getPositionSolarPanelMsg())

        rate.sleep() #wait according to publish rate

    cap.release() #free camera




"""
Calculate center of an Aruco tag.
    corners (numpy.ndarray) ->      all detected corners.
    index (int)              ->     index of the tag.

Return a tuple (int, int).
"""
def getCenterArucoTag(corners, index):
    #Get x pos (these could be inversed but not important here)
    low_x = corners[index][0][0][0]
    high_x = corners[index][0][3][0]

    #Get y pos
    low_y = corners[index][0][0][1]
    high_y = corners[index][0][3][1]

    return (low_x+high_x//2, low_y+high_y//2)


"""
Get message for realtime position of all robots.
    corners (numpy.ndarray) ->      all detected aruco corners.
    ids (numpy.ndarray)     ->      all detected aruco ids.
    ally_rids (list)        ->      ally robot ids.
    enem_rids (list)        ->      enemy robot ids.

Return an ArrayPositionPxWithType msg.
"""
def getPositionRobotMsg(corners, ids, ally_rids, enem_rids):
    msg = [] #create msg
    ally_msg = PositionPxWithType() 
    enemy_msg = PositionPxWithType()

    #Ally robot has been detected
    if ally_rids in ids:
        #Get center of tag        
        ally_msg.x, ally_msg.y = getCenterArucoTag(corners, list(ids).index(ally_rids))
        ally_msg.theta = 0 #No angle for now        
        ally_msg.type = "ally" #Robot is an ally
        #Append the ally pos into msg
        msg.append(ally_msg)
        
    
    #Enemy robot has been detected
    if enem_rids in ids:
        #Get center of the tag
        enemy_msg.x, enemy_msg.y = getCenterArucoTag(corners, list(ids).index(enem_rids))
        enemy_msg.theta = 0 #No angle for now        
        enemy_msg.type = "enemy" #Robot is an enemy
        #Append the enemy pos into msg
        msg.append(enemy_msg)

    return msg


"""
Get message for realtime position of all pamis.
    corners (numpy.ndarray) ->      all detected aruco corners.
    ids (numpy.ndarray)     ->      all detected aruco ids.
    ally_pids (list)        ->      ally pami ids.
    enem_pids (list)        ->      enemy pami ids.

Return an ArrayPositionPxWithType msg.
"""
def getPositionPamiMsg(corners, ids, ally_pids, enem_pids):
    msg = [] #create msg
    ally_msg = PositionPxWithType() 
    enemy_msg = PositionPxWithType()

    #Ally robot has been detected
    if ally_pids in ids:
        #Get center of tag        
        ally_msg.x, ally_msg.y = getCenterArucoTag(corners, list(ids).index(ally_pids))
        ally_msg.theta = 0 #No angle for now        
        ally_msg.type = "ally" #Robot is an ally
        #Append the ally pos into msg
        msg.append(ally_msg)
        
    
    #Enemy robot has been detected
    if enem_pids in ids:
        #Get center of the tag
        enemy_msg.x, enemy_msg.y = getCenterArucoTag(corners, list(ids).index(enem_pids))
        enemy_msg.theta = 0 #No angle for now        
        enemy_msg.type = "enemy" #Robot is an enemy
        #Append the enemy pos into msg
        msg.append(enemy_msg)

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





if __name__ == '__main__':
    publisher()