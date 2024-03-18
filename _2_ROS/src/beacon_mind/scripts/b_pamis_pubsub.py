#!/usr/bin/env python3
import os
import sys
import rospy
import traceback
import json
import socket
import types
import selectors
import subprocess
from std_msgs.msg import Int32MultiArray
from beacon_msgs.msg import ArrayPositionPxWithType, ArrayPositionPx

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
        rospy.init_node('b_pamis', anonymous=True)

        #This node will listen to these topics
        rospy.Subscriber("beacon/position/pamis", ArrayPositionPxWithType, self.pamiPosCallback)
        rospy.Subscriber("beacon/position/plants", ArrayPositionPx, self.plantPosCallback)
        self.color = color #save color
        with open (CONFIG_FILEPATH, "r") as f:
            self.config = json.load(f)
        self.pami_info = {key:None for key in self.config[color+"_PAMI_IDS"]} #get pami id
        self.pami_targets = self.config[color+"_PAMI_TARGETS"] #get pami targets


        #This node will publish to this topic
        self.connected_pamis_pub = rospy.Publisher("beacon/connected/pamis", Int32MultiArray, queue_size=10)
        
        #Initialize socket
        self.initSocket()
        

    def initSocket(self):
        """
        Initialize socket for pami to connect to.
        """
        HOST = self.config["SELF_IP_ADDRESS_ON_TPLINK"][0]#ip address of the tplink
        PORT = 45000 #port communication

        #Initialize ipv4 TCP socket
        lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        #Bind host and port to our just created socket
        lsock.bind((HOST, PORT))

        #Listen mode
        lsock.listen()
        
        #Not blocking mode because we dont want to stop the program waiting for data to be received§/!
        lsock.setblocking(False)

        #Watch out for new data
        self.selector = selectors.DefaultSelector()
        self.selector.register(lsock, selectors.EVENT_READ, data=None)


    def createSocketClient(self, socket):
        """
        Create a new socket between server and a client.

        Parameters:
            - socket (socket):  
        """
        #Accept new connection request receive by our socket
        socket_to_client, address_client = socket.accept()

        #Set the new socket to no blocking mode
        socket_to_client.setblocking(False)

        #Create a simple data object
        data = types.SimpleNamespace(address=address_client, input=b"", output=b"")

        #Specify which event to monitor
        events = selectors.EVENT_READ | selectors.EVENT_WRITE

        #Save the new socket
        self.selector.register(socket_to_client, events, data=data)


    def serviceSocketClient(self, key, mask):
        """
        Process data requested by a pami.

        Parameters:
            - key (selector): data attach to a socket
            - mask (int): event mask selecting a specific event 
        """

        socket_pami = key.fileobj #get pami's socket
        data = key.data #get attached data
        flag = 0

        #If a read is requested
        if mask & selectors.EVENT_READ:

            received_data = socket.recv() #get receive data of max 1024 bytes
            pami_number = key.laddr[0][-1] #get pami number
            pami_tag = self.config[self.color+"_PAMI_IDS"][pami_number] #get tag number of the pami

            #Give the pami its position
            if received_data == b"GET_SELF_POSITION":
                data.output = self.pami_info[pami_tag] #give the pami its position
            
            #Give the pami its target
            elif received_data == b"GOTO_POSITION":
                data.output = self.pami_targets[pami_number]
            
            #Give nothing if the request is not known
            else:
                data.output = None

        #Usually a write event is in the mask too 
        if mask & selectors.EVENT_WRITE:
            #If the request was known and fulfill send the response
            if data.output:
                flag = 1
                socket_pami.send(data.output)

        #Close connection to socket
        self.selector.unregister(socket_pami)
        socket_pami.close()
        print(f"Closing connection to {data.address}\nResult : {flag}\n\n")


    def run(self):

        #Set publish rate
        rate = rospy.Rate(10) #in hz

        while not rospy.is_shutdown():
            
            #Wait timeout time for something to be read 
            events = self.selector.select(timeout=0.1)
            
            #Loop through events
            for key, mask in events:

                #If no data are received
                if key.data is None:
                    #Create a new socket connection
                    self.createSocketClient(key.fileobj)
                    
                else:
                    self.serviceSocketClient(key, mask)  


            #Publish 
            self.connected_pamis_pub.publish(self.getConnectedPamiMsg())

            rate.sleep() #wait according to publish rate



    def pamiPosCallback(self, data):
        """
        Callback to  update pamis position.
        """
        #Fill the dict with pami data 
        for pami in data.array_of_positionspx_with_type:
            tag_id = int(pami.type.split('_')[1]) #get current id
            self.pami_info[tag_id] = (pami.x, pami.y, pami.theta) #get pami position


    def plantPosCallback(self, data):
        """
        Callback for plants position.
        """
        #Update plant position  
        self.plant_position = data.array_of_positionspx


    def getConnectedPamiMsg(self):
        """
        Get message for connected pamis. The tag is given.
        """
        msg = Int32MultiArray()

        #Get ip address of all pami
        pami_ip_address = self.config["IP_ADDRESS_PAMIS"]

        #Ping every ip
        ping_result = []
        for ip in pami_ip_address:
            #Just one ping with 0.1 second wait time
            result_bytes = subprocess.run(["ping", "-c", "1", "-W", "0.15", ip],
                                    stdout=subprocess.PIPE)
            result = result_bytes.stdout.decode("ascii").split() #get string result
            ping_result.append("0%" in result) #append result bool
        
        #Get connected pami
        msg.data = [tag for index, tag in enumerate(self.config[self.color+"_PAMI_IDS"]) if ping_result[index]]
        return msg

        



if __name__ == '__main__':
    #Launch the node
    try:
        pami_node = PamisNode(color="BLUE")#instantiate it
        pami_node.run()
    except Exception as e:
        traceback.print_exc()
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : {e}")
