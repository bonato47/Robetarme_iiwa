#!/bin/bash

cd src
git clone git@github.com:epfl-lasa/kuka_fri.git
git clone --recursive https://github.com/jrl-umi3218/SpaceVecAlg.git
git clone --recursive https://github.com/jrl-umi3218/RBDyn.git
git clone --recursive https://github.com/jrl-umi3218/mc_rbdyn_urdf.git
git clone https://github.com/mosra/corrade.git
git clone https://github.com/epfl-lasa/robot_controllers.git
git clone  --branch feature/dockerise https://github.com/epfl-lasa/iiwa_ros.git
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
cd send_pos
mkdir Third_party
cd Third_party
git clone https://github.com/coin-or/qpOASES.git
cd ../..
source devel /setup.bash
cd build
cmake ..
make

read terminate
