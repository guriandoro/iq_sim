#!/bin/bash

# Start coord: -33.9559789,-55.6752125,584,353

mkdir -p $HOME/.config/ardupilot/
echo "Uruguay=-34.9559789,-54.6752125,584,353" > $HOME/.config/ardupilot/locations.txt

cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris -L Uruguay --console
