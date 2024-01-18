import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose, Twist

pose_data = {'x': [], 'y': [], 'z': [], 'timestamps': []}
ee_info_data = {'x': [], 'y': [], 'z': [], 'timestamps': []}

def pose_callback(data):
    pose_data['x'].append((data.position.x, rospy.get_time()))  # Storing x-coordinate and timestamp
    pose_data['y'].append((data.position.y, rospy.get_time()))  # Storing y-coordinate and timestamp
    pose_data['z'].append((data.position.z, rospy.get_time()))  # Storing z-coordinate and timestamp
    pose_data['timestamps'].append(rospy.get_time())  # Storing timestamp

def ee_info_callback(data):
    ee_info_data['x'].append((data.linear.x, rospy.get_time()))  # Storing linear x-speed and timestamp
    ee_info_data['y'].append((data.linear.y, rospy.get_time()))  # Storing linear y-speed and timestamp
    ee_info_data['z'].append((data.linear.z, rospy.get_time()))  # Storing linear z-speed and timestamp
    ee_info_data['timestamps'].append(rospy.get_time())  # Storing timestamp

def listener():
    rospy.init_node('topic_subscriber', anonymous=True)
    rospy.Subscriber("passive_control/vel_quat", Pose, pose_callback)
    rospy.Subscriber("ur5/ee_info/Vel", Twist, ee_info_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()

        # Create subplots for x, y, and z coordinates of position and velocity
        fig, axs = plt.subplots(3, 1, figsize=(8, 12))

        # Plot x, y, and z coordinates of position against timestamps
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

        plt.suptitle('Position and Velocity Coordinates vs. Timestamps', fontsize=16)
        plt.tight_layout()
        plt.show()

    except rospy.ROSInterruptException:
        pass
