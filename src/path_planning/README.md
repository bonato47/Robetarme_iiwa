# Instructions to run
* Place the package in catkin_ws/src
* Requires iiwa_ros and all the associated dependencies

# Nodes and their purpose
* Fundamental nodes:
    * ```getIK_node```: Performs the inverse kinematic computation for a subscribed EE position and publishes the computed joint state
    * ```record```: Saves the end-effector position (3) + orientation (4) + joint position (7) data during robot's motion into a csv file
    * ```follow_waypoints```: Reads from a csv file and sequentially publishes the desired waypoint to be reached by the end-effector

* Example nodes:
    * ```circle```: Publishes waypoints which lie on a circle
    * ```straight_line```: Publishes waypoints which lie on a line

# Config
Contains the ```param.yaml``` file which can be used to alter the different parameters determing the behaviour of the robot.



