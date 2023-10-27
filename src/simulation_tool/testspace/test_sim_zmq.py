import os
from coppeliasim_zmqremoteapi_client import *

current_dir = os.path.dirname(os.path.abspath(__file__))
PATH_SCENE = current_dir + "/paint_scene_mk2.ttt"

client = RemoteAPIClient()
sim = client.getObject("sim")
client.setStepping(True)


def main():
    """Main function."""
    sim.loadScene(PATH_SCENE)

    sim.startSimulation()
    while sim.getSimulationState() != sim.simulation_stopped:
        laser_point = retrieve_laser_data("./LaserPointer")
        print(laser_point)
        client.step()
    sim.stopSimulation()


def retrieve_laser_data(str_lasername: str) -> list:
    """Retrieve laser data.
    
    args:
        str_lasername (str): Name of the laser sensor.

    returns:
        list: Laser data.
    """
    sensor_laser = sim.getObject(str_lasername)
    result, _, pt, _, _ = sim.handleProximitySensor(sensor_laser)

    if (result > 0):
        m = sim.getObjectMatrix(sensor_laser, sim.handle_world)
        pt1 = sim.multiplyVector(m, pt)

        return pt1

    return None


if __name__ == "__main__":
    main()