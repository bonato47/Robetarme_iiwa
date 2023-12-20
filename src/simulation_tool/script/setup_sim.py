import os
import yaml
import xml.etree.ElementTree as ET
import signal
import argparse
import asyncio

from typing import Dict, List
from dataclasses import dataclass, field, asdict
from coppeliasim_zmqremoteapi_client.asyncio import RemoteAPIClient


VERBOSE = True
SHUTDOWN_KEY = False
current_dir = os.path.dirname(os.path.abspath(__file__)) + "/"


@dataclass()
class Robot():
    """ Robot dataclass. """
    name = ""
    handle: int = 0
    world_link: str = ""
    nb_dofs: int = 0

    lst_joints: List[int] = field(default_factory=list)
    lst_joints_name: List[str] = field(default_factory=list)
    lst_links: List[int] = field(default_factory=list)
    lst_links_name: List[str] = field(default_factory=list)

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

    if VERBOSE:
        robot.print()

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
        await setup_robot(client, sim, robot, yaml_config)

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
    new_robot.world_link = "/" + \
        urdf_tree.find("link").attrib["name"] + "_visual"

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

    # Fill the robot links name
    for link in urdf_tree.findall(".//link"):
        link_resp = link.find("collision")
        if link_resp is not None:
            new_robot.lst_links_name.append(link.attrib["name"] + "_respondable")

    return new_robot


async def send_command_bash(bash_command: List[str]) -> asyncio.subprocess.Process:
    """Send specific bash command.

    args:
        bash_command (List[str]): Bash command to be launched using asyncio subprocess.

    return:
        (asyncio.subprocess.Process): Process currently running after calling this function.
    """
    return await asyncio.create_subprocess_exec(*bash_command)


async def attach_script_to_object(sim, robot: Robot, control_type: str):
    """ Read the script content from the file and attach it to the object.

    args:
        sim (any): CoppeliaSim simulated client.
        robot (Robot): Robot dataclass.
        control_type (str): Control type to be used for the robot.
    """
    lua_file_content = ""
    child_script_handle = await sim.addScript(sim.scripttype_childscript)
    await sim.associateScriptWithObject(child_script_handle, robot.handle)

    # Fill the lua file content with the different scriptable parts
    with open("../lua/common_part.lua", "r") as lua_common:
        lua_file_content += lua_common.read().replace("CONTROLLER_TYPE", control_type.lower())

    str_joints_name = "jointNames = {"
    str_joints_name += ", ".join([f"'{j}'" for j in robot.lst_joints_name]) + "}"

    str_joints_handle = "jointHandles = {"
    str_joints_handle += ", ".join([f"'{j}'" for j in robot.lst_joints]) + "}"

    str_links_handle = "linkHandles = {"
    str_links_handle += ", ".join([f"'{j}'" for j in robot.lst_links]) + "}"

    with open("../lua/get_handles.lua", "r") as lua_handles:
        str_get_handles = lua_handles.read()

    str_get_handles = str_get_handles.replace("JOINT_NAMES", str_joints_name)
    str_get_handles = str_get_handles.replace(
        "JOINT_HANDLES", str_joints_handle
    )
    str_get_handles = str_get_handles.replace(
        "LINK_HANDLES", str_links_handle
    )
    str_get_handles = str_get_handles.replace(
        "BASE_HANDLE_NAME",
        "'" + robot.world_link + "/" + robot.lst_links_name[0] + "'"
    )
    str_get_handles = str_get_handles.replace(
        "EE_HANDLE_NAME",
        "'" + robot.world_link + "/" + robot.lst_links_name[0] + "'"
    )

    lua_file_content += str_get_handles

    with open("../lua/set_cmd.lua", "r") as lua_cmd:
        lua_file_content += lua_cmd.read().replace("CMD", control_type)

    await sim.setScriptStringParam(
        child_script_handle,
        sim.scriptstringparam_text,
        lua_file_content
    )


async def setup_robot(client: any, sim: any, robot: Robot, config: Dict[str, str]):
    """ Setup the simulation using the parameters passed in input.

    args:
        client (any): CoppeliaSim python zero mq client.
        sim (any): CoppeliaSim simulated client.
        robot (Robot): Robot dataclass.
        config (dict[str, str]): YAML configuration as a python dictionnary.
    """
    robot.name = await client.call(
        ("simURDF.import"),
        (
            current_dir + config["filepath"]["urdf"],
            int(0b01000001100),  # bitwise operation
        )
    )

    for j in robot.lst_joints_name:
        robot.lst_joints.append(await sim.getObject(robot.world_link + "/" + j))

    for l in robot.lst_links_name:
        robot.lst_links.append(await sim.getObject(robot.world_link + "/" + l))

    for i, j in enumerate(robot.lst_joints):
        await sim.setJointMode(j, sim.jointmode_dynamic, 0)
        await sim.setJointPosition(j, config["robot"]["init_pos"][i])

        if config["robot"]["control_type"] == "Position":
            await sim.setObjectInt32Param(
                j, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position
            )
            await sim.setJointTargetPosition(j, config["robot"]["init_cmd"][i])
        elif config["robot"]["control_type"] == "Velocity":
            await sim.setObjectInt32Param(
                j, sim.jointintparam_dynctrlmode, sim.jointdynctrl_velocity
            )
            await sim.setJointTargetVelocity(j, config["robot"]["init_cmd"][i])
        elif config["robot"]["control_type"] == "Force":
            await sim.setObjectInt32Param(
                j, sim.jointintparam_dynctrlmode, sim.jointdynctrl_force
            )
            await sim.setJointForce(j, config["robot"]["init_cmd"][i])

    robot.handle = await sim.getObject(robot.world_link)
    await attach_script_to_object(
        sim,
        robot,
        config["robot"]["control_type"]
    )


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
