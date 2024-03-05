#!/usr/bin/env python3
import os
import rospy
from beacon_msgs.msg import ArrayPositionPxWithType

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)


class PamisNode:
    """
    This Ros node is a used as a gateway between the beacon and pamis.
    """

def pamiPosCallback(data):
    """
    Callback for pamis position.
    Insert data in a database.
    """
    #Fill the dict with pami data 
    for pami in data.array_of_positionspx_with_type:
        pass#send location to the pami over wifi




def subscriber():
    """
    Fetch and parse data from ros to pamis
    """
    # Tell node name to rospy
    rospy.init_node('listener', anonymous=True)

    # This node will listen to these topics
    rospy.Subscriber("beacon/position/pamis", ArrayPositionPxWithType, pamiPosCallback)
    #rospy.Subscriber("beacon/position/pots", ArrayPositionPx, callback)

    # Keeps python from exiting until this node is stopped
    # also it permits to this node to listen to new messages on mentioned topics
    # and to run specified callbacks
    rospy.spin()



if __name__ == '__main__':
    subscriber()