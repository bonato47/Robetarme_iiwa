import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal

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
    y_positions_target = target_df['z'].to_numpy()

    # Create a grid of X and Y values outside the loop
    x_grid, y_grid = np.meshgrid(np.linspace(min(x_positions_ee), max(x_positions_ee), 100),
                                  np.linspace(min(z_positions_ee), max(z_positions_ee), 100))

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
    plt.plot(x_positions_target, y_positions_target, color='blue', label='Target Polygon', linewidth=2)
    plt.plot([x_positions_target[-1], x_positions_target[0]],
             [y_positions_target[-1], y_positions_target[0]],
             color='blue', linewidth=2)

    # Set labels and title
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title(f'Height Map with Gaussian Distributions (Radius = {desired_radius}) and Target Polygon')

    # Show the plot
    plt.legend()
    plt.show()

# if __name__ == '__main__':
#     generate_height_map()
