#!/usr/bin/env python

import rospy
from balise_msgs.msg import PositionRobot, PositionGameElements, PositionAruco


"""
Publish position of Robots, Elements and Arucos on the board
"""
def publisher():

    all_pub = {} #will store all publishers from this node

    # This node will publish to the topic balise/position/robot
    robotPos_pub = rospy.Publisher("balise/position/robots", PositionRobot, queue_size=10)
    # This node will publish to the topic balise/position/gameElements
    gameElementsPos_pub = rospy.Publisher("balise/position/gameElements", PositionGameElements, queue_size=10)
    # This node will publish to the topic balise/position/aruco
    all_pub["aruco_pos"] = rospy.Publisher("balise/position/aruco", PositionAruco, queue_size=10)

    #Tell node name to rospy
    rospy.init_node("b_cam_position", anonymous=True)

    #Set publish rate
    rate = rospy.Rate(10) #10hz

    while not rospy.is_shutdown():
        robotPos_pub.publish(getPositionRobot())
        gameElementsPos_pub.publish(getGameElements())

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass