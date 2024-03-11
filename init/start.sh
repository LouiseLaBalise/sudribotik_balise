#!/bin/bash
#Run competition_start.launch

export BEACON_CAN_COMPETE="false"
python3 $(pwd)/init/verify_beacon_connection.py

if [ "$BEACON_CAN_COMPETE" = "true" ]; then
    roslaunch beacon_launch competition_start.launch
fi
