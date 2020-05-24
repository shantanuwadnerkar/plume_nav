#!/bin/bash

cd src/

# GADEN
git clone git@github.com:shantanuwadnerkar/gaden.git
git clone git@github.com:MAPIRlab/olfaction_msgs.git

# RotorS
# git clone git@github.com:shantanuwadnerkar/rotors_simulator.git
# git clone git@github.com:ethz-asl/mav_comm.git

cd ..

# change the build tool to catkin-tools
catkin_make

# printf "\n\n\n\n\n\n\n\n\n"
echo "PROJECT BUILT!!"
