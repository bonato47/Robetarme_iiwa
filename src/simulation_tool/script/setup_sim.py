import os
import sys
import yaml
import xml.etree.ElementTree as ET
import time
import signal
import argparse
import asyncio

from typing import Dict
from math import radians

from dataclasses import dataclass, field, asdict
from typing import List, Tuple
from coppeliasim_zmqremoteapi_client.asyncio import RemoteAPIClient

VERBOSE = False
SHUTDOWN_KEY = False
current_dir = os.path.dirname(os.path.abspath(__file__)) + "/"


@dataclass()
class Robot():
    """ Robot dataclass. """
    name: str = ""
    nb_dofs: int = 0
    max_limits: Tuple[float, float] = (radians(-180), radians(180))  # [rad]
    max_vel: float = radians(90)  # [rad/s]
    max_acc: float = 5.0  # [rad/s^2]
    max_jerk: float = 0.5  # [rad/s^3]
    max_torque: float = 80.0  # [Nm]

    lst_joints: List[int] = field(default_factory=list)
    lst_joints_name: List[str] = field(default_factory=list)
    lst_joints_type: List[str] = field(default_factory=list)

    lst_limits: List[Tuple[float, float]] = field(
        default_factory=list
    )  # [rad]
    lst_max_vel: List[float] = field(default_factory=list)  # [rad/s]
    lst_max_acc: List[float] = field(default_factory=list)  # [rad/s^2]
    lst_max_jerk: List[float] = field(default_factory=list)  # [rad/s^3]
    lst_max_torque: List[float] = field(default_factory=list)  # [Nm]

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


async def main():
    """ Main function. """
    global SHUTDOWN_KEY
    lst_processes = []
    lst_parents_roslaunch = []

    # Bind ctrl + c command to shutdown function
    signal.signal(signal.SIGINT, on_ctrl_c_pressed)

    # Read YAML file
    yaml_config = read_yaml_file(parse_input_arguments())

    robot = read_urdf(current_dir + yaml_config["filepath"]["urdf"])
    robot.init_default()

    if VERBOSE:
        robot.print()

    lst_processes.append(await send_command_bash(["roscore"]))
    await asyncio.sleep(1)

    try:
        script_path = os.path.expanduser(yaml_config["filepath"]["coppeliasim"])
        lst_processes.append(await send_command_bash([script_path]))
    except:
        print("CoppeliaSim not found. Please check the path.")
        await shutdown(
            lst_processes=lst_processes,
            lst_parents_roslaunch=lst_parents_roslaunch
        )

    async with RemoteAPIClient() as client:
        sim = await client.require('sim')
        simURDF = await client.require('simURDF')

        await client.call(
            ("simURDF.import"),
            (
                current_dir + yaml_config["filepath"]["urdf"],
                int(0b01000001100) # bitwise operation
            )
        )
        await attach_script_to_object(sim, None, yaml_config["filepath"]["lua_control"])

        # setup_simulation(sim, robot, yaml_config)

        print("Simulation ready to start")
        while not SHUTDOWN_KEY:
            await asyncio.sleep(1)  # to not overload process
            pass

        await shutdown(
            lst_processes=lst_processes,
            lst_parents_roslaunch=lst_parents_roslaunch
        )


def parse_input_arguments() -> str:
    """ Parse input arguments and return the needed ones.

    return:
        yaml_filename (str): YAML filename where to take input parameters.
    """
    parser = argparse.ArgumentParser(
        description="""Script to setup a coppeliasim simulation based on YAML config file.""")

    # Required yaml filename
    parser.add_argument(
        "yaml_filename",
        type=str,
        help="YAML configuration filepath where setup is detailed."
    )

    # Parse arguments
    args = parser.parse_args()

    return args.yaml_filename


def read_yaml_file(filename: str) -> Dict[str, str]:
    """ Read the right yaml file and return the related dictionnary.

    args:
        filename (str): YAML filename where to take input parameters.

    returns:
        config dict[str, str]: YAML configuration as a python dictionnary.
    """
    with open(current_dir + filename, "r", encoding="utf-8") as yaml_file:
        try:
            config = yaml.safe_load(yaml_file)
        except yaml.YAMLError as exc:
            print(exc)

    return config


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

            new_robot.lst_limits.append((
                float(joint_limit.attrib["lower"]),
                float(joint_limit.attrib["upper"])
            ))
            new_robot.lst_max_vel.append(float(joint_limit.attrib["velocity"]))
            new_robot.lst_max_torque.append(
                float(joint_limit.attrib["effort"])
            )

    new_robot.nb_dofs = nb_dofs

    return new_robot


async def send_command_bash(bash_command: List[str]) -> asyncio.subprocess.Process:
    """Send specific bash command.

    args:
        bash_command (List[str]): Bash command to be launched using asyncio subprocess.

    return:
        (asyncio.subprocess.Process): Process currently running after calling this function.
    """
    return await asyncio.create_subprocess_exec(*bash_command)


async def attach_script_to_object(sim, objectHandle, scriptPath):
    """ Read the script content from the file and attach it to the object.

    args:
        sim (any): CoppeliaSim simulated client.
        objectHandle (int): Handle of the object to attach the script to.
        scriptPath (str): Path to the script file.
    """
    lua_file_content = ""
    dummyHandle = await sim.createDummy(0.1)
    child_script_handle = await sim.addScript(sim.scripttype_childscript)
    await sim.associateScriptWithObject(child_script_handle, dummyHandle)

    with open(scriptPath, "r") as lua_control:
        lua_file_content = lua_control.read()

    await sim.setScriptStringParam(
        child_script_handle,
        sim.scriptstringparam_text,
        lua_file_content
    )


def link_joints(robot: Robot, sim: any):
    """ Link the joint to the robot.

    args:
        robot (Robot): Robot dataclass.
        sim (any): CoppeliaSim simulated client.
    """
    for i in range(robot.nb_dofs):
        robot.lst_joints.append(sim.getObject("./joint", {'index': i}))


def setup_simulation(sim: any, robot: Robot, config: Dict[str, str]):
    """ Setup the simulation using the parameters passed in input.

    args:
        sim (any): CoppeliaSim simulated client.
        robot (Robot): Robot dataclass.
        config (dict[str, str]): Simulation configuration dictionnary.
    """
    sim.loadScene(current_dir + config["filepath"]["scene"])
    link_joints(robot, sim)

    for i in range(robot.nb_dofs):
        robot_joint = robot.lst_joints[i]

        sim.setJointInterval(
            robot_joint, False, [
                robot.lst_limits[i][0], robot.lst_limits[i][1] -
                robot.lst_limits[i][0]
            ]
        )

        sim.setJointMode(robot_joint, sim.jointmode_dynamic, 0)
        sim.setObjectInt32Param(
            robot_joint, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position)


def init_robot_pos(sim: any, robot: Robot, init_pos: List[float]):
    """ Initialize the robot position. """
    for i in range(robot.nb_dofs):
        sim.setJointPosition(robot.lst_joints[i], init_pos[i])
        sim.setJointTargetPosition(robot.lst_joints[i], init_pos[i])


async def shutdown(**kwargs):
    """Shutdown the current script and all the related ros process.

    args:
        **kwargs (dict[str, str]): Dictionnary of different processes that have to
                                   be shutdown to well quit this script.
    """
    if kwargs["lst_processes"]:
        for proc in reversed(kwargs["lst_processes"]):
            if proc.returncode is None:
                print(f"Terminating process {proc.pid}...")
                proc.terminate()
                await proc.wait()

    if kwargs["lst_parents_roslaunch"]:
        for par_roslaunch in kwargs["lst_parents_roslaunch"]:
            par_roslaunch.shutdown()


def on_ctrl_c_pressed(_signum: int, _frame: any):
    """ Keyboard Listener to bypass normal ctrl-c behavior to well shutdown process.
    """
    global SHUTDOWN_KEY
    SHUTDOWN_KEY = True


if __name__ == "__main__":
    asyncio.run(main())
