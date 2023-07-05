from roboticstoolbox.robot.ERobot import ERobot
import pathlib
import typing as tp
import roboticstoolbox as rtb
import matplotlib.pyplot as plt


""" class ExtendedFK(ERobot):

    def __init__(self, urdf_file: tp.Union[str, pathlib.PosixPath]) -> None: 

        (eLinks, name, urdf, file_path) = ERobot.URDF_read(urdf_file)
        super().__init__(eLinks, name=name) 
        
robot_name = "ur10"
urdf_path = "home/ros/ros_ws/src/cobod_arm_study/urdf/" + robot_name + ".urdf"
 """

robot = rtb.models.URDF.UR10()

print(robot)
T = robot.fkine([0,0,0,0,0,0])  # forward kinematics
sol = robot.ikine_LM(T)                          # inverse kinematics


robot.plot(q=sol.q, backend='pyplot', dt=5, vellipse=True)
T = robot.fkine([1,0,0,0,0,0])  # forward kinematics
sol = robot.ikine_LM(T)             
robot.plot(q=sol.q, backend='pyplot', dt=5, vellipse=True)