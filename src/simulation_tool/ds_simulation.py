import os
import xml.etree.ElementTree as ET
import rospy

from math import radians
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from dataclasses import dataclass, field, asdict
from typing import List, Tuple
from coppeliasim_zmqremoteapi_client import *

current_dir = os.path.dirname(os.path.abspath(__file__))
PATH_SCENE = current_dir + "/ds_trial.ttt"
FREQUENCY = 100

URDF_FILE = current_dir + "/urdf/iiwa7.urdf"

@dataclass()
class Robot():
    """Robot dataclass."""
    name: str = ""
    nb_dofs: int = 0
    max_limits: Tuple[int, int] = (0.0, 0.0) # [rad]
    max_vel: float = 0.0 # [rad/s]
    max_acc: float = 0.0 # [rad/s^2]
    max_jerk: float = 0.0 # [rad/s^3]
    max_torque: float = 0.0 # [rad/s^3]

    lst_joints: List[int] = field(default_factory=list)
    lst_joints_name: List[str] = field(default_factory=list)
    lst_joints_type: List[str] = field(default_factory=list)
    lst_limits: List[Tuple[int, int]] = field(default_factory=list)
    lst_max_vel: List[int] = field(default_factory=list) # [rad/s]
    lst_max_acc: List[int] = field(default_factory=list) # [rad/s^2]
    lst_max_jerk: List[int] = field(default_factory=list) # [rad/s^3]
    lst_max_torque: List[int] = field(default_factory=list) # [rad/s^3]

    def print(self):
        """Print the robot informations."""
        print(f"Robot informations:")
        for key, value in asdict(self).items():
            print(f"\t{key}: {value}")

client = RemoteAPIClient()
sim = client.getObject("sim")
client.setStepping(True)

def main():
    """Main function."""
    kuka_iiwa14 = read_urdf(URDF_FILE)
    sim.loadScene(PATH_SCENE)

    link_joints(kuka_iiwa14)
    kuka_iiwa14.print()
    setup_rostopics()

    sim.startSimulation()
    while sim.getSimulationState() != sim.simulation_stopped:
        client.step()
    sim.stopSimulation()


def read_urdf(str_urdf: str) -> Robot:
    """ Read the URDF file using standard XML language.

    args:
        str_urdf (str): Path to the URDF file.

    returns:
        Robot: Robot dataclass containing all the informations from URDF file.
    """
    new_robot = Robot()
    robot = ET.parse(str_urdf).getroot()

    # Extract information from the URDF
    new_robot.name = robot.attrib["name"]

    nb_dofs = 0
    for joint in robot.findall(".//joint"):
        try:
            joint.attrib["type"]
        except:
            continue

        joint_limit = joint.find("limit")
        if joint_limit is not None:
            nb_dofs += 1
            new_robot.lst_joints_name.append(joint.attrib["name"])
            new_robot.lst_joints_type.append(joint.attrib["type"])

            new_robot.lst_limits.append(
                (joint_limit.attrib["lower"], joint_limit.attrib["upper"])
            )
            new_robot.lst_max_vel.append(joint_limit.attrib["velocity"])
            new_robot.lst_max_torque.append(joint_limit.attrib["effort"])

    new_robot.nb_dofs = nb_dofs

    return new_robot

def link_joints(robot: Robot):
    """ Link the joint to the robot.

    args:
        robot (Robot): Robot dataclass.
    """
    for i in range(robot.nb_dofs):
        robot.lst_joints.append(sim.getObject("./joint", {'index': i}))

def setup_rostopics():
    """Setup all the different ROS requirements."""
    rospy.init_node("ds_simulation", anonymous=True)
    ros_rate = rospy.Rate(FREQUENCY)
    rospy.sleep(2)

    return ros_rate


if __name__ == "__main__":
    main()