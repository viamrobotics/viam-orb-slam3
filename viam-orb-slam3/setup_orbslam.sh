#!/bin/bash
echo "Installing ORB_SLAM3 external dependencies"
sudo apt update
sudo apt install -y cmake libglew-dev libopencv-dev libeigen3-dev libssl-dev libboost-all-dev libpangolin-dev
