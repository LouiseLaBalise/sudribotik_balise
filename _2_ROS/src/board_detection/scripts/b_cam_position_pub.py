#!/usr/bin/env python

import rospy
import cv2
from balise_msgs.msg import PositionPx, PositionPxWithType, ArrayPositionPx, ArrayPositionPxWithType


"""
Publish position of Robots, Elements and Arucos on the board
"""
def publisher():

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

    while not rospy.is_shutdown():

        cap = cv2.VideoCapture(0) #take a photo
        ret,frame = cap.read() #read it

        robotsPos_pub.publish(getPositionRobotMsg())
        pamisPos_pub.publish(getPosition())
        plantsPos_pub.publish(getGameElements())
        robotsPos_pub.publish(getPositionRobot())

        rate.sleep() #wait according to publish rate

"""
Get realtime position of a robot.

Return an ArrayPositionPxWithType msg.
"""
def getPositionRobotMsg():
    msg = ArrayPositionPxWithType()



if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass