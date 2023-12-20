import os
import yaml
import xml.etree.ElementTree as ET
import signal
import argparse
import asyncio

from typing import Dict, List
from dataclasses import dataclass, field, asdict
from coppeliasim_zmqremoteapi_client.asyncio import RemoteAPIClient


VERBOSE = False
SHUTDOWN_KEY = False
current_dir = os.path.dirname(os.path.abspath(__file__)) + "/"


@dataclass()
class Robot():
    """ Robot dataclass. """
    handle: int = 0
    world_link: str = ""
    nb_dofs: int = 0
    lst_joints: List[int] = field(default_factory=list)
    lst_joints_name: List[str] = field(default_factory=list)

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

    # Read YAML and URDF files
    yaml_config = read_yaml_file(parse_input_arguments())
    robot = read_urdf(current_dir + yaml_config["filepath"]["urdf"])

    # Launch roscore and coppeliasim
    lst_processes.append(await send_command_bash(["roscore"]))
    await asyncio.sleep(1)

    try:
        script_path = os.path.expanduser(
            yaml_config["filepath"]["coppeliasim"])
        lst_processes.append(await send_command_bash([script_path]))
    except:
        print("CoppeliaSim not found. Please check the path.")
        await shutdown(
            lst_processes=lst_processes,
            lst_parents_roslaunch=lst_parents_roslaunch
        )

    # Set up the simulation environment with the right robot
    async with RemoteAPIClient() as client:
        sim = await client.require('sim')
        simURDF = await client.require('simURDF')

        await sim.loadScene(current_dir + yaml_config["filepath"]["scene"])
        await client.call(
            ("simURDF.import"),
            (
                current_dir + yaml_config["filepath"]["urdf"],
                int(0b01000001100),  # bitwise operation
            )
        )

        robot.handle = await sim.getObject(robot.world_link)
        await attach_script_to_object(
            sim,
            robot.handle,
            yaml_config["filepath"]["lua_control"]
        )
        await setup_simulation(sim, robot, yaml_config["robot"]["init_pos"])

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
        Robot: Robot dataclass containing information from URDF file.
    """
    new_robot = Robot()
    urdf_tree = ET.parse(str_urdf)

    # Fill the robot world link name
    new_robot.world_link = "/" + urdf_tree.find("link").attrib["name"] + "_visual"

    # Fill the robot joints name
    nb_dofs = 0
    for joint in urdf_tree.findall(".//joint"):
        try:
            joint.attrib["type"]
        except:
            continue

        joint_limit = joint.find("limit")
        if joint_limit is not None:
            nb_dofs += 1
            new_robot.lst_joints_name.append(joint.attrib["name"])

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


async def attach_script_to_object(sim, object_handle, script_path):
    """ Read the script content from the file and attach it to the object.

    args:
        sim (any): CoppeliaSim simulated client.
        object_handle (int): Handle of the object to attach the script to.
        script_path (str): Path to the script file.
    """
    lua_file_content = ""
    child_script_handle = await sim.addScript(sim.scripttype_childscript)
    await sim.associateScriptWithObject(child_script_handle, object_handle)

    with open(script_path, "r") as lua_control:
        lua_file_content = lua_control.read()

    await sim.setScriptStringParam(
        child_script_handle,
        sim.scriptstringparam_text,
        lua_file_content
    )


async def link_joints(sim: any, robot: Robot):
    """ Link the joint to the robot.

    args:
        sim (any): CoppeliaSim simulated client.
        robot (Robot): Robot dataclass.
    """
    for j in robot.lst_joints_name:
        robot.lst_joints.append(await sim.getObject(robot.world_link + "/" + j))


async def setup_simulation(sim: any, robot: Robot, init_pos: List[float]):
    """ Setup the simulation using the parameters passed in input.

    args:
        sim (any): CoppeliaSim simulated client.
        robot (Robot): Robot dataclass.
        init_pos (List[float]): Initial position of the robot.
    """
    await link_joints(sim, robot)

    for i, j in enumerate(robot.lst_joints):
        await sim.setJointMode(j, sim.jointmode_dynamic, 0)
        await sim.setObjectInt32Param(
            j, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position
        )

        await sim.setJointPosition(j, init_pos[i])
        await sim.setJointTargetPosition(j, init_pos[i])


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
