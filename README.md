# Robetarme_iiwa
READ_ME

These files has to purpose to run the iiwa arm robot by following:
1) a Dynamical system
2) a path 

In src/send_pose you have one node for the dynamical system (Ds.cpp) , one node to follow the path (Follow_traj) and one node (Send_pos) to send the robot to the desired position (quat,pos) by ROSParam. 


Prerequisite :
You need to install the following:

- Docker
- aica-technology/docker-images : https://github.com/aica-technology/docker-images

Docker process: 
You need first to build the docker from the folder docker:
bash build_docker.sh

Then activate the container in a server mode.
bash start_docker.sh server

Here you can open different terminal with the line:
bash start_docker.sh connect

Simulation:
To launch the simulation, you need to run in the docker terminal.

roslaunch iiwa_gazebo iiwa_gazebo.launch controller:=PositionController

Then to run the Ds node, open a new terminal in another docker terminal and write:
rosrun send_pos Ds

To replace it at initialize pos you can then run :
rosrun send_pos Send_pos
