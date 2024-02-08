import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose, Twist
import math
import numpy as np

start_record = False
timeStart= 0.0
pose_data = {'x': [], 'y': [], 'z': [], 'timestamps': []}
ee_info_data = {'x': [], 'y': [], 'z': [], 'timestamps': []}

def pose_callback(data):
    if start_record:
        current_time = rospy.Time.now() - timeStart
        pose_data['x'].append((data.position.x, rospy.get_time()))  # Storing x-coordinate and timestamp
        pose_data['y'].append((data.position.y, rospy.get_time()))  # Storing y-coordinate and timestamp
        pose_data['z'].append((data.position.z, rospy.get_time()))  # Storing z-coordinate and timestamp
        pose_data['timestamps'].append(current_time)  # Storing timestamp



def ee_info_callback(data):
    if start_record:
        current_time = rospy.Time.now() - timeStart
        ee_info_data['x'].append((data.linear.x, rospy.get_time()))  # Storing linear x-speed and timestamp
        ee_info_data['y'].append((data.linear.y, rospy.get_time()))  # Storing linear y-speed and timestamp
        ee_info_data['z'].append((data.linear.z, rospy.get_time()))  # Storing linear z-speed and timestamp
        ee_info_data['timestamps'].append(current_time)  # Storing timestamp


def listener(freq):
    rospy.init_node('topic_subscriber', anonymous=True)
    rospy.Subscriber("passive_control/vel_quat", Pose, pose_callback)
    rospy.Subscriber("ur5/ee_info/Vel", Twist, ee_info_callback)
    ros_rate = rospy.Rate(freq)
    return ros_rate

if __name__ == '__main__':
    try:
        ros_rate = listener(100)
    except rospy.ROSInterruptException:
        print("Ros interruption")
    
    while not start_record:
        start_record = rospy.get_param("/startController")
        timeStart = rospy.Time.now()

        ros_rate.sleep()

    loop = True
    while loop:
        ros_rate.sleep()
        loop = not rospy.get_param("/finishDS")
    # Assuming pose_data and ee_info_data are dictionaries with 'x', 'y', 'z', and 'timestamps' keys

    # Create subplots for x, y, and z coordinates of position and velocity
    fig, axs = plt.subplots(3, 1, figsize=(8, 12))

    # Plot x, y, and z coordinates of velocity  against timestamps
    for i, axis in enumerate(['x', 'y', 'z']):
        coordinates, timestamps = zip(*pose_data[axis])
        axs[i].plot(timestamps, coordinates, label=f'{axis}-coordinate')
        axs[i].set_xlabel('Time')
        axs[i].set_ylabel(f'{axis.capitalize()}-coordinate')
        axs[i].legend()

    # Plot x, y, and z coordinates of velocity against timestamps
    for i, axis in enumerate(['x', 'y', 'z']):
        coordinates, timestamps = zip(*ee_info_data[axis])
        axs[i].plot(timestamps, coordinates, label=f'Linear {axis}-velocity')
        axs[i].set_xlabel('Time')
        axs[i].set_ylabel(f'{axis.capitalize()}-velocity')
        axs[i].legend()
            
    # max_vel_temp = math.sqrt(data.linear.x**3+data.linear.y**2+data.linear.z**2)

    # if max_vel_temp > max_velocity_robot:
    #     max_velocity_robot = max_vel_temp

    # axs[-1].annotate(f'Max Velocity of the robot: {max_velocity_robot}',
    #                  xy=(0.5, 0), xytext=(0, -60), textcoords='offset points',
    #                  ha='center', va='top', fontsize=10, color='blue')

    plt.suptitle('Position and Velocity Coordinates vs. Timestamps', fontsize=16)
    plt.tight_layout()
    plt.show()
    rospy.set_param("/finishDS", False)

