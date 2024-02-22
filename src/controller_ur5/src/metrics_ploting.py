import rospy
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from geometry_msgs.msg import Pose, Twist
from geometry_msgs.msg import PolygonStamped, Point32
import math
import numpy as np
import pandas as pd
import os


start_record = False
timeStart= 0.0
pose_data = {'x': [], 'y': [], 'z': [], 'timestamps': []}
ee_info_Vel_data = {'x': [], 'y': [], 'z': [], 'timestamps': []}
ee_info_Pose_data = {'x': [], 'y': [], 'z': []}
target_info_Pose_data = {'x': [], 'y': [], 'z': []}

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
        ee_info_Vel_data['z'].append((data.linear.z, current_time.to_sec())) # Storing linear z-speed and timestamp
        ee_info_Vel_data['timestamps'].append(current_time.to_sec())  # Storing timestamp
        
def ee_info_Pose_callback(data):
    if start_record:
        current_time = rospy.Time.now() - timeStart
        ee_info_Pose_data['x'].append(data.position.x)  # Storing x position and timestamp
        ee_info_Pose_data['y'].append(data.position.y)  # Storing y position and timestamp
        ee_info_Pose_data['z'].append(data.position.z)  # Storing z position and timestamp
        # ee_info_Pose_data['timestamps'].append(current_time.to_sec())  # Use consistent timestamp for all entries
        
def target_info_Pose_callback(data):
    global target_info_Pose_data

    # Extracting points from the PolygonStamped message
    points = data.polygon.points

    # Assuming you want to append each point to the dictionary
    for point in points:
        target_info_Pose_data['x'].append(point.x)
        target_info_Pose_data['y'].append(point.y)
        target_info_Pose_data['z'].append(point.z)
        
def generate_height_map(ee_df,
                        target_df,
                        desired_radius=0.03):    # Function to check if a point is inside a polygon
    def point_in_polygon(x, y, poly_x, poly_y):
        n = len(poly_x)
        inside = False
        j = n - 1

        for i in range(n):
            if ((poly_y[i] > y) != (poly_y[j] > y)) and (x < (poly_x[j] - poly_x[i]) * (y - poly_y[i]) / (poly_y[j] - poly_y[i]) + poly_x[i]):
                inside = not inside
            j = i

        return inside

    # Load the CSV file containing end effector poses

    # Extract X, Y, and Z coordinates from the end effector DataFrame
    x_positions_ee = ee_df['x']
    z_positions_ee = ee_df['z']

    # Load the CSV file containing the target (polygon) poses

    # Extract X and Y coordinates from the target DataFrame
    x_positions_target = target_df['x'].to_numpy()
    z_positions_target = target_df['z'].to_numpy()

    # Create a grid of X and Y values outside the loop
    x_grid, y_grid = np.meshgrid(np.linspace(min(x_positions_ee), max(x_positions_ee), 20),
                                  np.linspace(min(z_positions_ee), max(z_positions_ee), 20))

    # Initialize an empty height map
    height_map = np.zeros_like(x_grid)

    # Create positions array for vectorized computation
    positions = np.dstack((x_grid, y_grid))

    # Calculate the standard deviation based on the desired radius
    std_deviation = desired_radius / np.sqrt(2)

    # Create a covariance matrix with the calculated standard deviation
    covariance_matrix = np.eye(2) * std_deviation**2

    # Loop through each time step
    for i in range(len(x_positions_ee)):
        # Calculate the mean for each time step
        mean = [x_positions_ee[i], z_positions_ee[i]]

        # Vectorized computation of the Gaussian distribution with the specified covariance matrix
        rv = multivariate_normal(mean, covariance_matrix)
        height_map += rv.pdf(positions)

    # Create a contour plot with the height map
    plt.contourf(x_grid, y_grid, height_map, cmap='viridis', alpha=0.5)
    plt.colorbar(label='Probability Density')

    # Scatter plot the actual end effector poses with reduced visibility
    plt.scatter(x_positions_ee, z_positions_ee, c='red', marker='.', s=1, alpha=0.1, label='End Effector Poses')

    # Plot the target polygon, closing the loop by connecting the last and first points
    plt.plot(x_positions_target, z_positions_target, color='blue', label='Target Polygon', linewidth=2)
    plt.plot([x_positions_target[-1], x_positions_target[0]],
             [z_positions_target[-1], z_positions_target[0]],
             color='blue', linewidth=2)

    # Set labels and title
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title(f'Height Map with Gaussian Distributions (Radius = {desired_radius}) and Target Polygon')

    # Show the plot
    plt.legend()
    plt.show()
            
def listener(freq):
    rospy.init_node('topic_subscriber', anonymous=True)
    rospy.Subscriber("passive_control/vel_quat", Pose, pose_callback)
    rospy.Subscriber("ur5/ee_info/Vel", Twist, ee_info_Vel_callback)
    rospy.Subscriber("ur5/ee_info/Pose", Pose, ee_info_Pose_callback)
    rospy.Subscriber("original_polygon", PolygonStamped, target_info_Pose_callback)

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
    # Assuming pose_data and ee_info_Vel_data are dictionaries with 'x', 'y', 'z', and 'timestamps' keys

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
        coordinates, timestamps = zip(*ee_info_Vel_data[axis])
        axs[i].plot(timestamps, coordinates, label=f'Linear {axis}-velocity')
        axs[i].set_xlabel('Time')
        axs[i].set_ylabel(f'{axis.capitalize()}-velocity')
        axs[i].legend()
            
    plt.suptitle('Position and Velocity Coordinates vs. Timestamps', fontsize=16)
    plt.tight_layout()
    plt.show()
    
    # Get the absolute path to the current script
    script_path = os.path.abspath(__file__)
    script_directory = os.path.dirname(script_path)

    # Define the path to save the CSV file
    csv_file_path_ee = os.path.join(script_directory, '..', 'csv/ee_info_pose_data.csv')

    # Find the length of the longest list
    max_length = max(map(len, ee_info_Pose_data.values())) - 100

    # Pad shorter lists with NaN values
    ee_info_Pose_data_padded = {key: value + [float('nan')] * (max_length - len(value)) for key, value in ee_info_Pose_data.items()}

    # Create DataFrame
    ee_df = pd.DataFrame.from_dict(ee_info_Pose_data_padded, orient='index').transpose()

    ee_df.to_csv(csv_file_path_ee, index=False)
    
    csv_file_path_target = os.path.join(script_directory, '..', 'csv/target_info_pose_data.csv')
    target_df = pd.DataFrame(target_info_Pose_data)
    target_df.to_csv(csv_file_path_target, index=False)
    
    rospy.set_param("/finishDS", False)
    
    
    generate_height_map(ee_df,target_df,desired_radius=0.03)

 