import os
import xml.etree.ElementTree as ET
import rospy
import signal

from math import radians, degrees
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from dataclasses import dataclass, field, asdict
from typing import List, Tuple
from coppeliasim_zmqremoteapi_client import *

new_torque = False
cmd_torques = None
current_dir = os.path.dirname(os.path.abspath(__file__))

VERBOSE = False
PATH_SCENE = current_dir + "/ds_trial.ttt"
FREQUENCY = 100

URDF_FILE = current_dir + "/urdf/iiwa7.urdf"
INIT_POS = [
    -0.5919698011605155,
    0.6201465625958642,
    -0.0653170266187102,
    -1.4804073686310648,
    -0.9575874780333455,
    -0.8252918783639069,
    0.8145857988451297
]

@dataclass()
class Robot():
    """ Robot dataclass. """
    name: str = ""
    nb_dofs: int = 0
    max_limits: Tuple[int, int] = (radians(-180), radians(180)) # [rad]
    max_vel: float = radians(90) # [rad/s]
    max_acc: float = 5.0 # [rad/s^2]
    max_jerk: float = 0.5 # [rad/s^3]
    max_torque: float = 80.0 # [Nm]

    lst_joints: List[int] = field(default_factory=list)
    lst_joints_name: List[str] = field(default_factory=list)
    lst_joints_type: List[str] = field(default_factory=list)

    lst_limits: List[Tuple[float, float]] = field(default_factory=list)
    lst_max_vel: List[float] = field(default_factory=list) # [rad/s]
    lst_max_acc: List[float] = field(default_factory=list) # [rad/s^2]
    lst_max_jerk: List[float] = field(default_factory=list) # [rad/s^3]
    lst_max_torque: List[float] = field(default_factory=list) # [Nm]

    def init_default(self):
        """ Init list with default values if they are empty. """
        if not self.lst_limits:
            self.lst_limits = [self.max_limits] * self.nb_dofs

        if not self.lst_max_vel:
            self.lst_max_vel = [self.max_vel] * self.nb_dofs

        if not self.lst_max_acc:
            self.lst_max_acc = [self.max_acc] * self.nb_dofs

        if not self.lst_max_jerk:
            self.lst_max_jerk = [self.max_jerk] * self.nb_dofs

        if not self.lst_max_torque:
            self.lst_max_torque = [self.max_torque] * self.nb_dofs

    def print(self):
        """ Print the robot informations. """
        print("Robot informations:")
        for key, value in asdict(self).items():
            print(f"\t{key}: {value}")


def main():
    """ Main function. """
    global new_torque

    kuka_iiwa14 = read_urdf(URDF_FILE)
    kuka_iiwa14.init_default()

    if VERBOSE:
        kuka_iiwa14.print()

    ros_rate = setup_rostopics()

    client = RemoteAPIClient()
    sim = client.getObject("sim")
    # client.setStepping(True)
    setup_simulation(sim, kuka_iiwa14)
    init_robot_pos(sim, kuka_iiwa14)

    print("Simulation ready to start")
    while not rospy.is_shutdown():
        if new_torque:
            if sim.getSimulationState() == sim.simulation_stopped:
                sim.startSimulation()

            new_torque = False
            set_torque(sim, kuka_iiwa14)

            # client.step()

        ros_rate.sleep()

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
                (float(joint_limit.attrib["lower"]), float(joint_limit.attrib["upper"]))
            )
            new_robot.lst_max_vel.append(float(joint_limit.attrib["velocity"]))
            new_robot.lst_max_torque.append(float(joint_limit.attrib["effort"]))

    new_robot.nb_dofs = nb_dofs

    return new_robot


def link_joints(robot: Robot, sim: any):
    """ Link the joint to the robot.

    args:
        robot (Robot): Robot dataclass.
        sim (any): CoppeliaSim simulated client.
    """
    for i in range(robot.nb_dofs):
        robot.lst_joints.append(sim.getObject("./joint", {'index': i}))


def setup_rostopics() -> Tuple[rospy.Rate, rospy.Publisher]:
    """ Setup all the different ROS requirements. """
    rospy.init_node("ds_simulation", anonymous=True)
    ros_rate = rospy.Rate(FREQUENCY)
    rospy.sleep(2)

    # Subscriber
    rospy.Subscriber("/iiwa/TorqueController/command", Float64MultiArray, cbk_torque_command)

    # Publisher
    # pub_joints_states = rospy.Publisher("/iiwa/JointStates", Float64MultiArray, queue_size=10)

    return ros_rate#, pub_joints_states


def setup_simulation(sim: any, robot: Robot):
    """ Setup the simulation using the parameters passed in input.

    args:
        sim (any): CoppeliaSim simulated client.
        robot (Robot): Robot dataclass.
    """
    sim.loadScene(PATH_SCENE)
    link_joints(robot, sim)

    for i in range(robot.nb_dofs):
        robot_joint = robot.lst_joints[i]

        sim.setJointInterval(
            robot_joint, False, [
                robot.lst_limits[i][0], robot.lst_limits[i][1] - robot.lst_limits[i][0]
            ]
        )

        sim.setJointMode(robot_joint, sim.jointmode_dynamic, 0)
        sim.setObjectInt32Param(robot_joint, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position)


def init_robot_pos(sim: any, robot: Robot):
    """ Initialize the robot position. """
    for i in range(robot.nb_dofs):
        sim.setJointPosition(robot.lst_joints[i], INIT_POS[i])


def set_torque(sim: any, robot: Robot):
    """ Set the torque to the robot.

    args:
        sim (any): CoppeliaSim simulated client.
        robot (Robot): Robot dataclass.
    """
    for i in range(robot.nb_dofs):
        robot_joint = robot.lst_joints[i]
        sim.setJointTargetForce(robot_joint, cmd_torques[i])


def cbk_torque_command(msg_torques: Float64MultiArray):
    """ Callback function to retrieve the torques from the controller.

    args:
        msg_torques (Float64MultiArray): Torques from the controller.
    """
    global new_torque
    global cmd_torques

    new_torque = True
    cmd_torques = msg_torques.data


def handler():
    """ Handler function to stop the simulation. """
    exit(1)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, handler)
    main()