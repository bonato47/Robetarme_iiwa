# Robetarme_iiwa
READ_ME

These files has to purpose to run the iiwa arm robot by following:
1) a Dynamical system
2) a path 

In src/send_pose you have one node for the dynamical system (Ds.cpp) , one node to follow the path (Follow_traj) and one node to send the robot to thje desired position (quat,pos) by ROSParam. 


Installing process:
You need to install docker. 
then you need to run the GetRepo.sh from your Ros_workspace.


Simulation:
To launch the simulation, you need to run the docker.

open a terminal at you Ros_workspace, and run:

cd src/iiwa_ros/src/docker
bash install_docker.bash
bash build_docker.bash

Now to open your container run :

bash start_docker.bash interactive

And to launch the simulation run :

roslaunch iiwa_gazebo iiwa_gazebo.launch controller:=PositionController

Then to run the Ds node, opena terminal in your workspace, 
write source devel_/setup.bash
rosrun send_pos Ds

To replace it at initialize pos you can then run :
rosrun send_pos Send_pos
