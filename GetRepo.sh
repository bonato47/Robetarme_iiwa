#!/bin/bash

cd src
git clone https://github.com/epfl-lasa/kuka_fri.git
git clone --recursive https://github.com/jrl-umi3218/SpaceVecAlg.git
git clone --recursive https://github.com/jrl-umi3218/RBDyn.git
git clone --recursive https://github.com/jrl-umi3218/mc_rbdyn_urdf.git
git clone https://github.com/mosra/corrade.git
git clone https://github.com/epfl-lasa/robot_controllers.git

cd ..
source devel/setup.bash
catkin_make
read terminate