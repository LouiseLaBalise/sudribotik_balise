#!/bin/bash
#Run competition_start.launch

python3 $(pwd)/init/verify_beacon_connection.py

roslaunch beacon_launch competition_start.launch
