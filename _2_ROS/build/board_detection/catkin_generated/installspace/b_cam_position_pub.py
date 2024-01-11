#!/usr/bin/env python3
import os
import sys
import time
import json
import rospy
import cv2

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)

sys.path.insert(1, FILE_PATH.split("_2_ROS")[0]) #add parent folder to python path
from _3_TRAITEMENT_d_IMAGES import ros_redressBoardUsingAruco, ros_detectAruco
from balise_msgs.msg import PositionPx, PositionPxWithType, ArrayPositionPx, ArrayPositionPxWithType


"""
Publish position of Robots, Elements and Arucos on the board
"""
def publisher():
 
    #Get Aruco constants
    aruco_id_filepath = FILE_PATH.split("_2_ROS")[0]+"init/aruco_identification.json"
    with open (aruco_id_filepath, "r") as f:
        aruco_tags_const = json.load(f)

    # This node will publish ally and enemy robot position
    robotsPos_pub = rospy.Publisher("balise/position/robots", ArrayPositionPxWithType, queue_size=10)

    # This node will publish all allies and enemies PAMIs position
    pamisPos_pub = rospy.Publisher("balise/position/pamis", ArrayPositionPxWithType, queue_size=10)

    # This node will publish all fragile and resistant plants
    plantsPos_pub = rospy.Publisher("balise/position/plants", ArrayPositionPxWithType, queue_size=10)

    # This node will publish all iron pots
    potPos_pub = rospy.Publisher("balise/position/pots", ArrayPositionPx, queue_size=10)

    #Tell node name to rospy
    rospy.init_node("b_cam_position", anonymous=True)

    #Set publish rate
    rate = rospy.Rate(10) #10hz


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
        #pamisPos_pub.publish(getPositionPamiMsg())
        #plantsPos_pub.publish(getPositionPlantMsg())
        #potPos_pub.publish(getPositionPotMsg())

        rate.sleep() #wait according to publish rate

    cap.release() #free camera



"""
Get realtime position of a robot.
    corners (numpy.ndarray) ->      all detected corners.
    ids (numpy.ndarray)     ->      all detected ids.
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
        #Get center of the tag
        ally_msg.x = int((corners[list(ids).index(ally_rids)][0][0][0] + corners[ids.index(ally_rids)][0][2][0]) / 2)
        ally_msg.y = int((corners[list(ids).index(ally_rids)][0][0][1] + corners[ids.index(ally_rids)][0][2][1]) / 2)
        ally_msg.theta = 0 #No angle for now        
        ally_msg.type = "ally" #Robot is an ally
        #Append the ally pos into msg
        msg.append(ally_msg)
        print(5)
    
    #Enemy robot has been detected
    if enem_rids in ids:
        #Get center of the tag
        enemy_msg.x = int((corners[list(ids).index(enem_rids)][0][0][0] + corners[ids.index(enem_rids)][0][2][0]) / 2)
        enemy_msg.y = int((corners[list(ids).index(enem_rids)][0][0][1] + corners[ids.index(enem_rids)][0][2][1]) / 2)
        enemy_msg.theta = 0 #No angle for now        
        enemy_msg.type = "enemy" #Robot is an enemy
        #Append the enemy pos into msg
        msg.append(enemy_msg)

    return msg








if __name__ == '__main__':
    publisher()