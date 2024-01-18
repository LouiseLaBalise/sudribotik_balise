#!/usr/bin/env python
import rospy
from balise_msgs.msg import PositionPx, PositionPxWithType, ArrayPositionPx, ArrayPositionPxWithType




def callback(data):
    for robot in data.array_of_positionspx_with_type:
        rospy.loginfo(f"{robot.type} on ({robot.x}, {robot.y})")
    
def listener():

    # Tell node name to rospy
    rospy.init_node('listener', anonymous=True)

    # This node will listen to robots position
    rospy.Subscriber("balise/position/robots", ArrayPositionPxWithType, callback)

    # Keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()