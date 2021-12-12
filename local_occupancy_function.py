import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

'''
Use the function local_occupancy(abs_pos, sensor_vals) to return a plot of the local occupancy.
You must give the inputs in the correct format. 
abs_pos: list of lists containing the absolute x y and phi coordinates in that order
sensor_vals: list of lists containing the 7 sensor vals in the order they are read by thymio
'''


sensor_distances = np.array([15, 9.6, 9, 8.5, 7.5, 7, 6, 5, 4, 3, 2.5, 2, 1.5, 0.5, 0.2, 0])
sensor_measurements = np.array([0, 1186,1420, 1510, 1715, 1830, 2069, 2250, 2468, 2850, 3155, 3468, 3940, 4355, 4432, 4500])

## Interpolation from sensor values to distances in cm
def sensor_val_to_cm_dist(val):
    """
    Returns the distance corresponding to the sensor value based
    on the sensor characteristics
    :param val: the sensor value that you want to convert to a distance
    :return: corresponding distance in cm
    """
    if val == 0:
        return np.inf

    f = interp1d(sensor_measurements, sensor_distances)
    return f(val).item()

center_offset = np.array([5.5,5.5])
sensor_pos_from_center = np.array([[0.9,9.4], [3.1,10.5], [5.5,11.0], [8.0,10.4], [10.2,9.3], [2.5,0], [8.5,0]])-center_offset
sensor_angles = np.array([120, 105, 90, 75, 60, -90, -90])*math.pi/180

def obstacles_pos_from_sensor_vals(sensor_vals):
    """
    Returns a list containing the position of the obstacles
    w.r.t the center of the Thymio robot.
    :param sensor_vals: sensor values provided clockwise starting from the top left sensor.
    :return: numpy.array() that contains the position of the different obstacles
    """
    dist_to_sensor = [sensor_val_to_cm_dist(x) for x in sensor_vals]
    dx_from_sensor = [d*math.cos(alpha) for (d, alpha) in zip(dist_to_sensor, sensor_angles)]
    dy_from_sensor = [d*math.sin(alpha) for (d, alpha) in zip(dist_to_sensor, sensor_angles)]
    obstacles_pos = [[x[0]+dx, x[1]+dy] for (x,dx,dy) in zip(sensor_pos_from_center,dx_from_sensor,dy_from_sensor )]
    return np.array(obstacles_pos)


def rotate(angle, coords):
    """
    Rotates the coordinates of a matrix by the desired angle
    :param angle: angle in radians by which we want to rotate
    :return: numpy.array() that contains rotated coordinates
    """
    R = np.array(((np.cos(angle), -np.sin(angle)),
                  (np.sin(angle), np.cos(angle))))

    return R.dot(coords.transpose()).transpose()

thymio_coords = np.array([[0,0], [11,0], [11,8.5], [10.2, 9.3],
                          [8, 10.4], [5.5,11], [3.1, 10.5],
                          [0.9, 9.4], [0, 8.5], [0,0]])-center_offset

def local_occupancy(abs_pos, sensor_vals):
    '''
    abs_pos: List of lists that will contain the absolute x,y,theta coordinates of the robot
    sensor_vals:  List of lists that will contain the 7 sensor values
    '''


    local_occupancy_grids = [obstacles_pos_from_sensor_vals(x) for x in sensor_vals]

    global_map, overall_thymio_coords = [], []

    for (local_grid, pos) in zip(local_occupancy_grids, abs_pos):
        # Rotate the local occupancy grid
        rotated_grid = rotate(pos[2] - math.pi / 2, local_grid)
        rotated_thymio_coords = rotate(pos[2] - math.pi / 2, thymio_coords)

        # Translate the grid at the position of the Thymio
        obstacles_pos = rotated_grid + np.array([pos[0], pos[1]])
        abs_Thymio_coords = rotated_thymio_coords + np.array([pos[0], pos[1]])

        # Store position of the obstacles and Thymio in the global map
        global_map.append(obstacles_pos)
        overall_thymio_coords.append(abs_Thymio_coords)

    global_map = np.array(np.vstack(global_map))

    plt.figure(figsize=(10, 10))
    plt.plot(abs_pos[:, 0], abs_pos[:, 1])
    plt.scatter(global_map[:, 0], global_map[:, 1], color="r", s=10)

    plt.plot(np.array(abs_pos)[:, 0],
             np.array(abs_pos)[:, 1], color="r", marker="o")

    for coords in overall_thymio_coords:
        plt.plot(coords[:, 0], coords[:, 1], color="g")

    plt.axis("equal");
    plt.show()




