import os
import yaml
import signal
import argparse
import asyncio

from typing import Dict, List
from dataclasses import dataclass, field, asdict
from coppeliasim_zmqremoteapi_client.asyncio import RemoteAPIClient
from urdf_parser_py.urdf import URDF


VERBOSE = True
SHUTDOWN_KEY = False
current_dir = os.path.dirname(os.path.abspath(__file__)) + "/"

#TODO: add footplatform as an option
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
    global VERBOSE

    new_robot = Robot()
    urdf_description = URDF.from_xml_file(str_urdf)

    # Fill the robot world link name
    for link in urdf_description.links:
        if link.name == "world":
            new_robot.world_link = "/" + link.name + "_visual"
            break

    if new_robot.world_link == "":
        if urdf_description.links[0].collision is None:
            new_robot.world_link = "/" + urdf_description.links[0].name + "_visual"
        else:
            new_robot.world_link = "/" + urdf_description.links[0].name + "_respondable"

    # Fill the robot joints name
    nb_dofs = 0
    for joint in urdf_description.joints:
        if joint.limit is not None:
            nb_dofs += 1
            new_robot.lst_joints_name.append(joint.name)

    new_robot.nb_dofs = nb_dofs

    # Fill the robot links name
    for link in urdf_description.links:
        if link.collision is not None:
            new_robot.lst_links_name.append(link.name + "_respondable")

    if VERBOSE:
        new_robot.print()

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
    with open("../lua/robot_controller.lua", "r") as lua_common:
        lua_file_content += lua_common.read().replace("CONTROLLER_TYPE", control_type.lower())

    lua_file_content = lua_file_content.replace("NB_DOFS", str(robot.nb_dofs))
    lua_file_content = lua_file_content.replace(
        "JOINT_NAMES", create_replacement("jointHandles = {", robot.lst_joints_name)
    )
    lua_file_content = lua_file_content.replace(
        "JOINT_HANDLES", create_replacement("jointHandles = {", robot.lst_joints)
    )
    lua_file_content = lua_file_content.replace(
        "LINK_HANDLES", create_replacement("linkHandles = {", robot.lst_links)
    )
    lua_file_content = lua_file_content.replace(
        "BASE_HANDLE_NAME",
        "'" + robot.world_link + "/" + robot.lst_links_name[0] + "'"
    )
    lua_file_content = lua_file_content.replace(
        "EE_HANDLE_NAME",
        "'" + robot.world_link + "/" + robot.lst_links_name[0] + "'"
    )

    lua_file_content = lua_file_content.replace("CMD", control_type)

    await sim.setScriptStringParam(
        child_script_handle,
        sim.scriptstringparam_text,
        lua_file_content
    )


def create_replacement(prefix: str, lst_str: List[str]) -> str:
    """ Create a replacement string for a specific string.

        args:
            prefix (str): Prefix to be added before the replacement.
            lst_str (List[str]): List of string to be replaced.

        return:
            new_str (str): New string with the replacement.
    """
    new_str = prefix
    new_str += ", ".join([f"'{s}'" for s in lst_str]) + "}"

    return new_str


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

    await sim.setObjectPosition(robot.handle, config["robot"]["xyz"])
    await sim.setObjectOrientation(robot.handle, config["robot"]["rpy"])


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
