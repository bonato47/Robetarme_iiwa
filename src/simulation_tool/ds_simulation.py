import os
import rospy

from math import radians
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from dataclasses import dataclass ,field
from typing import List, Tuple
from coppeliasim_zmqremoteapi_client import *

current_dir = os.path.dirname(os.path.abspath(__file__))
PATH_SCENE = current_dir + "/ds_trial.ttt"
FREQUENCY = 100

@dataclass(frozen=True)
class Robot():
    """Robot dataclass."""
    name: str = "iiwa14"
    nb_dofs: int = 7
    joints: List[int] = field(default_factory=list, init=False)
    max_vel: float = radians(110) # [rad/s]
    max_acc: float = radians(40) # [rad/s^2]
    max_jerk: float = radians(80) # [rad/s^3]

client = RemoteAPIClient()
sim = client.getObject("sim")
client.setStepping(True)


def main():
    """Main function."""
    kuka_iiwa14 = Robot()
    sim.loadScene(PATH_SCENE)
    setup_robot(kuka_iiwa14)
    setup_rostopics()

    sim.startSimulation()
    while sim.getSimulationState() != sim.simulation_stopped:
        client.step()
    sim.stopSimulation()


def setup_robot(robot: Robot):
    """ Setup the robot.

    args:
        robot (Robot): Robot dataclass.
    """
    for i in range(robot.nb_dofs):
        sim.getObject("./joint", {'index': i})

def setup_rostopics():
    """Setup all the different ROS requirements."""
    rospy.init_node("ds_simulation", anonymous=True)
    ros_rate = rospy.Rate(FREQUENCY)
    rospy.sleep(2)

    return ros_rate


if __name__ == "__main__":
    main()