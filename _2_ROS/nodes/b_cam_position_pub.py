#!/usr/bin/env python

import rospy




def publisher():

    all_pub = {} #will store all publishers from this node

    # This node will publish to the topic balise/position/robot
    all_pub["robots_pos"] = rospy.Publisher("balise/position/robots", String, queue_size=10)