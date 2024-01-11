#!/usr/bin/env python3
import rospy
from balise_msgs.msg import PositionPx, PositionPxWithType, ArrayPositionPx, ArrayPositionPxWithType




def callback(data):
    rospy.loginfo(f"{data.array_of_positionspx_with_type}")
    
def listener():

    # Tell node name to rospy
    rospy.init_node('listener', anonymous=True)

    # This node will listen to robots position
    rospy.Subscriber("balise/position/robots", ArrayPositionPxWithType, callback)

    # Keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()