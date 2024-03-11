#!/bin/bash
#Run competition_start.launch

condition=$(python3 $(pwd)/init/verify_beacon_connection.py)

if $condition; then
    export BEACON_CAN_COMPETE="true"
else
    export BEACON_CAN_COMPETE="false"
fi
