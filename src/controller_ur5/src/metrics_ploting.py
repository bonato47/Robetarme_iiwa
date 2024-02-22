import rospy
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from geometry_msgs.msg import Pose, Twist
from geometry_msgs.msg import PolygonStamped, Point32
import numpy as np
import pandas as pd
import os

start_record = False
timeStart = 0.0
pose_data = {'x': [], 'y': [], 'z': [], 'timestamps': []}
ee_info_Vel_data = {'x': [], 'y': [], 'z': [], 'timestamps': []}


def pose_callback(data):
    if start_record:
        current_time = rospy.Time.now() - timeStart
        pose_data['x'].append((data.position.x, current_time.to_sec()))  # Storing x-coordinate and timestamp
        pose_data['y'].append((data.position.y, current_time.to_sec()))  # Storing y-coordinate and timestamp
        pose_data['z'].append((data.position.z, current_time.to_sec()))  # Storing z-coordinate and timestamp
        pose_data['timestamps'].append(current_time.to_sec())  # Storing timestamp


def ee_info_Vel_callback(data):
    if start_record:
        current_time = rospy.Time.now() - timeStart
        ee_info_Vel_data['x'].append((data.linear.x, current_time.to_sec()))  # Storing linear x-speed and timestamp
        ee_info_Vel_data['y'].append((data.linear.y, current_time.to_sec()))  # Storing linear y-speed and timestamp
        ee_info_Vel_data['z'].append((data.linear.z, current_time.to_sec()))  # Storing linear z-speed and timestamp
        ee_info_Vel_data['timestamps'].append(current_time.to_sec())  # Storing timestamp
        

class HeightMapGenerator:
    def __init__(self, desired_radius=0.03, resolution=100):
        self.target_info_Pose_data = {'x': [], 'y': [], 'z': []}
        self.desired_radius = desired_radius
        self.resolution = resolution
        self.ee_info_Pose_data = {'x': [], 'y': [], 'z': []}
        self.x_positions_ee = []
        self.z_positions_ee = []
        self.x_positions_target = []
        self.z_positions_target = []
        self.x_grid = []
        self.y_grid = []
        self.height_map = []
        self.positions = []
        self.std_deviation = []
        self.covariance_matrix = []

    def ee_info_Pose_callback(self, data):
        if start_record:
            current_time = rospy.Time.now() - timeStart
            self.ee_info_Pose_data['x'].append(data.position.x)  # Storing x position and timestamp
            self.ee_info_Pose_data['y'].append(data.position.y)  # Storing y position and timestamp
            self.ee_info_Pose_data['z'].append(data.position.z)  # Storing z position and timestamp
            self.generate_height_map()
            
    def target_info_Pose_callback(self, data):
        # Extracting points from the PolygonStamped message
        points = data.polygon.points

        # Assuming you want to append each point to the dictionary
        for point in points:
            self.target_info_Pose_data['x'].append(point.x)
            self.target_info_Pose_data['y'].append(point.y)
            self.target_info_Pose_data['z'].append(point.z)

        if data:
            self.init_()
            
    def init_(self):

        # target_df = pd.DataFrame(self.target_info_Pose_data)
        self.x_positions_target = np.array(self.target_info_Pose_data['x'])
        self.z_positions_target = np.array(self.target_info_Pose_data['z'])

        self.x_grid, self.y_grid = np.meshgrid(
            np.linspace(min(self.x_positions_target, default=-1), max(self.x_positions_target, default=1), self.resolution),
            np.linspace(min(self.z_positions_target, default=-1), max(self.z_positions_target, default=1), self.resolution))
        self.height_map = np.zeros_like(self.x_grid)
        self.std_deviation = self.desired_radius / np.sqrt(2)
        self.covariance_matrix = np.eye(2) * self.std_deviation ** 2
        

    def generate_height_map(self):
        if self.ee_info_Pose_data:
            ee_df_one_step = {
                'x': [self.ee_info_Pose_data['x'][-1]],
                'y': [self.ee_info_Pose_data['y'][-1]],
                'z': [self.ee_info_Pose_data['z'][-1]]
            }
            self.positions = np.dstack((self.x_grid, self.y_grid))

            # Extract X, Y, and Z coordinates from the end effector DataFrame
            x_positions_ee = ee_df_one_step['x'][-1]
            z_positions_ee = ee_df_one_step['z'][-1]

            # Calculate the mean for each time step
            mean = [x_positions_ee, z_positions_ee]

            # Vectorized computation of the Gaussian distribution with the specified covariance matrix
            rv = multivariate_normal(mean, self.covariance_matrix)
            self.height_map += rv.pdf(self.positions)
            
    def plot_height_map(self):
        
        max_length = max(map(len, self.ee_info_Pose_data.values())) - 100
        ee_info_Pose_data_padded = {key: value + [float('nan')] * (max_length - len(value)) for key, value in
                                    self.ee_info_Pose_data.items()}

        ee_df = pd.DataFrame.from_dict(ee_info_Pose_data_padded, orient='index').transpose()
        self.x_positions_ee = ee_df['x']
        self.z_positions_ee = ee_df['z']
        print(self.z_positions_ee)

        plt.contourf(self.x_grid, self.y_grid, self.height_map, cmap='viridis', alpha=0.5)
        plt.colorbar(label='Probability Density')
        plt.scatter(self.x_positions_ee, self.z_positions_ee, c='red', marker='.', s=1, alpha=0.1,
                    label='End Effector Poses')
        plt.plot(self.x_positions_target, self.z_positions_target, color='blue', label='Target Polygon', linewidth=2)
        plt.plot([self.x_positions_target[-1], self.x_positions_target[0]],
                 [self.z_positions_target[-1], self.z_positions_target[0]],
                 color='blue', linewidth=2)
        
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title(f'Height Map with Gaussian Distributions (Radius = {self.desired_radius}) and Target Polygon')
        plt.legend()
        plt.show()

def listener(freq):
    global start_record, timeStart
    start_record = False
    timeStart = 0.0

    rospy.init_node('topic_subscriber', anonymous=True)
    rospy.Subscriber("passive_control/vel_quat", Pose, pose_callback)
    rospy.Subscriber("ur5/ee_info/Vel", Twist, ee_info_Vel_callback)

    heightmap = HeightMapGenerator(desired_radius=0.03, resolution=100)
    rospy.Subscriber("ur5/ee_info/Pose", Pose, heightmap.ee_info_Pose_callback)
    rospy.Subscriber("original_polygon", PolygonStamped, heightmap.target_info_Pose_callback)

    ros_rate = rospy.Rate(freq)
    return ros_rate, heightmap


if __name__ == '__main__':
    try:
        ros_rate, heightmap = listener(100)
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

    # Assuming pose_data and ee_info_Vel_data are dictionaries with 'x', 'y', 'z', and 'timestamps' keys
    fig, axs = plt.subplots(3, 1, figsize=(8, 12))

    for i, axis in enumerate(['x', 'y', 'z']):
        coordinates, timestamps = zip(*pose_data[axis])
        axs[i].plot(timestamps, coordinates, label=f'{axis}-coordinate')
        axs[i].set_xlabel('Time')
        axs[i].set_ylabel(f'{axis.capitalize()}-coordinate')
        axs[i].legend()

    for i, axis in enumerate(['x', 'y', 'z']):
        coordinates, timestamps = zip(*ee_info_Vel_data[axis])
        axs[i].plot(timestamps, coordinates, label=f'Linear {axis}-velocity')
        axs[i].set_xlabel('Time')
        axs[i].set_ylabel(f'{axis.capitalize()}-velocity')
        axs[i].legend()

    plt.suptitle('Position and Velocity Coordinates vs. Timestamps', fontsize=16)
    plt.tight_layout()
    plt.show()

    rospy.set_param("/finishDS", False)

    heightmap.plot_height_map()
