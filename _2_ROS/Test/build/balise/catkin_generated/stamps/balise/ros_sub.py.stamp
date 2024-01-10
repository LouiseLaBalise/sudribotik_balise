#!/usr/bin/env python

# --- ROS :
import rospy
from balise.msg import ObjArray

def callback(data):
    rospy.loginfo("Received:\n%s", data)


def listener():
    rospy.init_node('b_listener')
    rospy.Subscriber('balise/position/objets', ObjArray, callback)
    rospy.Subscriber('balise/position/robots', ObjArray, callback)
    rospy.Subscriber('balise/position/plants', ObjArray, callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()