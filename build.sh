#!/bin/bash

cd src/

git clone git@github.com:shantanuwadnerkar/gaden.git
git clone git@github.com:shantanuwadnerkar/rotors_simulator.git
git clone git@github.com:MAPIRlab/olfaction_msgs.git
git clone git@github.com:ethz-asl/mav_comm.git

cd ..

catkin build

printf "\n\n\n\n\n\n\n\n\n"
echo "PROJECT BUILT!!"
