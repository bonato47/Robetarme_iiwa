from roboticstoolbox.robot.ERobot import ERobot
import pathlib
import typing as tp
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import pandas as pd
import glob


""" class ExtendedFK(ERobot):

    def __init__(self, urdf_file: tp.Union[str, pathlib.PosixPath]) -> None: 

        (eLinks, name, urdf, file_path) = ERobot.URDF_read(urdf_file)
        super().__init__(eLinks, name=name) 
        
robot_name = "ur10"
urdf_path = "home/ros/ros_ws/src/cobod_arm_study/urdf/" + robot_name + ".urdf"
 """
nameFiles = glob.glob("trajectories/trajectory_joints_ur10*.csv")

data = pd.read_csv(nameFiles[0],header=None)
robot = rtb.models.URDF.UR10()

print(robot)

for i in range(len(data)):    
    robot.plot(q=data.iloc[i].values, backend='pyplot', dt=0.002, vellipse=True)
