#!/usr/bin/env python3
import os
import sys
import rospy
import traceback
import json
import socket
import selectors
from std_msgs.msg import Int32MultiArray
from beacon_msgs.msg import ArrayPositionPxWithType

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)
CONFIG_FILEPATH = FILE_PATH.split("_2_ROS")[0]+"init/configuration.json"    
sys.path.insert(1, FILE_PATH.split("_2_ROS")[0]) #add parent folder to python path
from init import prettify_json
from _4_BASE_DE_DONNEES import databaseManager


class PamisNode:
    """
    This Ros node is a used as a gateway between the beacon and pamis.
    """

    def __init__(self, color="BLUE"):

        #Initialize  and tell node name to rospy
        rospy.init_node('r_lidar', anonymous=True)

        #This node will listen to these topics
        rospy.Subscriber("beacon/position/pamis", ArrayPositionPxWithType, self.pamiPosCallback)
        self.color = color #save color
        with open (CONFIG_FILEPATH, "r") as f:
            self.config = json.load(f)
        self.pami_info = {key:None for key in self.config[color+"_PAMI_IDS"]} #get pami id


        #This node will publish to this topic
        self.connected_pamis_pub = rospy.Publisher("beacon/connected/pamis", Int32MultiArray, queue_size=10)
        
        #Initialize socket
        self.initSocket()
        

    def initSocket(self):
        """
        Initialize socket for pami to connect to.
        """
        HOST = "192.168.0.110" #ip address on the tplink
        PORT = 45000 #port communication

        #Initialize ipv4 TCP socket
        lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        #Bind host and port to our just created socket
        lsock.bind((HOST, PORT))

        #Listen mode
        lsock.listen()
        print(f"Listening on {HOST, PORT}")
        
        #Not blocking mode because we dont want to stop the program waiting for data to be received§/!
        lsock.setblocking(False)

        #Watch out for new data
        self.selector = selectors.DefaultSelector()
        self.selector.register(lsock, selectors.EVENT_READ, data=None)



    def run(self):

        #Set publish rate
        rate = rospy.Rate(10) #in hz

        while not rospy.is_shutdown():
            
            #Wait timeout time for something to be read
            if self.selector : 
                events = self.selector.select(timeout=None)
            else :
                events = []
        
            for key, mask in events: 
                print(events)

            #Publish 
            #self.connected_pamis_pub.publish()

            rate.sleep() #wait according to publish rate



    def pamiPosCallback(self, data):
        """
        Callback to  update pamis position.
        """
        #Fill the dict with pami data 
        for pami in data.array_of_positionspx_with_type:
            tag_id = int(pami.type.split('_')[1]) #get current id
            self.pami_info[tag_id] = (pami.x, pami.y, pami.theta) #get pami position




    def subscriber():
        """
        Fetch and parse data from ros to pamis
        """
        # Tell node name to rospy
        rospy.init_node('listener', anonymous=True)

        #rospy.Subscriber("beacon/position/pots", ArrayPositionPx, callback)

        # Keeps python from exiting until this node is stopped
        # also it permits to this node to listen to new messages on mentioned topics
        # and to run specified callbacks
        rospy.spin()



if __name__ == '__main__':
    #Launch the node
    try:
        pami_node = PamisNode(color="BLUE")#instantiate it
        pami_node.run()
    except Exception as e:
        traceback.print_exc()
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : {e}")
