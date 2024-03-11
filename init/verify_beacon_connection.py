#!/usr/bin/env python3
import os
import sys
import json
import subprocess
import time

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)
sys.path.insert(1, FILE_PATH.split("init")[0]) #add parent folder to python path
from init import network_manager

#Load config file   
CONFIG_FILEPATH = FILE_PATH.split("init")[0]+"init/configuration.json"
with open (CONFIG_FILEPATH, "r") as f:
    config = json.load(f)


#Louise must be connected to TP-Link
net_stat = network_manager.get_network_status()
if net_stat["eth0"] != config["SELF_IP_ADDRESS_ON_TPLINK"]:

    #Condition not satisfied
    print(f"Log [{FILE_NAME}]: Impossible de lancer le Louise roscore. "+\
          f"Êtes-vous bien relié par câble LAN au boîtier TP-Link ?")
    
    #Shell true because export is a shell function,and set stdout to DEVNULL bc we don't care of ouput
    subprocess.run(["export", "BEACON_CAN_COMPETE='false'"], shell=True,
                   stdout=subprocess.DEVNULL)
    exit(1)

#Force Louise to activate its hotpot if not already switched on
if net_stat["wlan0"] != config["SELF_IP_ADDRESS_ON_SELF_HOTSPOT"]:

    s_time = 7 #sleep time in seconds before switching on hotspot

    print(f"Log [{FILE_NAME}]: Le hotspot ne semble pas être activé. "+\
          f"Il sera donct activé automatquement dans {s_time} secondes.")
    print(f"Log [{FILE_NAME}]: ", end="")
    for k in range(s_time):
        print(k, end=" ")
        time.sleep(1) #wait 1 second

    #Launch hotspot
    subprocess.run(["nmcli", "con", "up", '"Pifi AP Mode"'], stdout=subprocess.PIPE)

    #Re-test for hotspot ip
    if network_manager.get_network_status()["wlan0"] != config["SELF_IP_ADDRESS_ON_SELF_HOTSPOT"]:

        #Condition not satisfied
        print(f"Log [{FILE_NAME}]: Le hotspot n'a pas pu être activé. "+\
              f"Impossible de lancer le Louise roscore")
        
        subprocess.run(["export", "BEACON_CAN_COMPETE='false'"], shell=True,
                       stdout=subprocess.DEVNULL)
        exit(1)

    

#Set Ros env variable to enable same roscore comunication
subprocess.run(["export", "ROS_MASTER_URI=http://"+config["LOUISE_IP_ON_TPLINK"]+":11311"], shell=True,
               stdout=subprocess.DEVNULL)
subprocess.run(["export", "ROS_IP="+config["SELF_IP_ADDRESS_ON_TPLINK"]], shell=True,
               stdout=subprocess.DEVNULL)

#All condition are satisfied
print(f"Log [{FILE_NAME}]: Lancement du Louise roscore.")
subprocess.run(["export", "BEACON_CAN_COMPETE='true'"], shell=True, stdout=subprocess.DEVNULL)
