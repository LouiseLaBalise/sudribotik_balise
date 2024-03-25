#!/usr/bin/env python3
import json
import os
import subprocess
import sys

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)
sys.path.insert(1, FILE_PATH.split("init")[0]) #add parent folder to python path
from init import network_manager

#Load config file   
CONFIG_FILEPATH = FILE_PATH.split("init")[0]+"init/configuration.json"
with open (CONFIG_FILEPATH, "r") as f:
    config = json.load(f)


def start_verification():
    """
    Verify that Louise is on the TP Link network prior to launch roscore.
    """

    #Louise must be connected to TP-Link
    net_stat = network_manager.get_network_status()
    if net_stat["eth0"] != config["SELF_IP_ADDRESS_ON_TPLINK"][0]:

        #Condition not satisfied
        print(f"Log [{FILE_NAME}]: Impossible de lancer le Louise roscore. "+\
            f"Êtes-vous bien relié par câble LAN au boîtier TP-Link ?")
        
        return False

    #Force Louise to activate its hotpot if not already switched on
    if net_stat["wlan0"] != config["SELF_IP_ADDRESS_ON_SELF_HOTSPOT"][0]:

        s_time = 6 #sleep time in seconds before switching on hotspot

        print(f"Log [{FILE_NAME}]: Le hotspot ne semble pas être activé. "+\
            f"Il sera donc automatiquement activé dans {s_time} secondes.")

        #Launch hotspot
        subprocess.run(["nmcli", "con", "up", "Pifi AP Mode"], stdout=subprocess.PIPE)

        #Re-test for hotspot ip
        if network_manager.get_network_status()["wlan0"] != config["SELF_IP_ADDRESS_ON_SELF_HOTSPOT"][0]:

            #Condition not satisfied
            print(f"Log [{FILE_NAME}]: Le hotspot n'a pas pu être activé. "+\
                f"Impossible de lancer le Louise roscore")
            

            return False


    #All condition are satisfied
    return True



if __name__ == "__main__":
    start_verification()