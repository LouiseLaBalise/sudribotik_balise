#!/usr/bin/env python3
import os
import sys
import subprocess

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)
sys.path.insert(1, FILE_PATH.split("init")[0]) #add parent folder to python path
from init import verify_beacon_connection

#Run competition_start.launch after verification success
if verify_beacon_connection.start_verification():
    print(f"Log [{FILE_NAME}]: Lancement du Louise roscore.")
    subprocess.run(["roslaunch", "beacon_launch", "competition_start.launch"],
                   stdout=subprocess.PIPE)

