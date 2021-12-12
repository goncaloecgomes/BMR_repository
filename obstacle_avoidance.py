# ## ------------------ Module for Obstale Avoidance for Thymio Robot - Basics of Mobile Robotics --------------------##

from tdmclient import ClientAsync
import Local_Nav as LN
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from local_occupancy import sensor_measurements, sensor_distances
from local_occupancy import thymio_coords, sensor_pos_from_center, sensor_angles
import math

client = ClientAsync()
node = client.aw(client.wait_for_node())

# set Cruise speed here
cruisin = 200;

motor_speed_left = cruisin
motor_speed_right = cruisin

# Define local_nav object with class attributes
local_nav = LN.Local_Nav(client, node, motor_speed_left, motor_speed_right)


# Store motor speeds in right format to update nodes
client.aw(node.lock_node())
cruise_speed = local_nav.motors(motor_speed_left,motor_speed_right)
stop = local_nav.motors(0,0)

# Test Local Navigation Class
counter = 0
# while counter < 200:
#
#     node = client.aw(client.wait_for_node()) #Update states and sensor values at each iteration
#     flag = local_nav.analyse_data() # store a flag if obstacle detected (red LEDS)
#     node.send_set_variables(cruise_speed)
#     if flag == 1:
#         task = local_nav.obstacle_avoidance() #trigger task depending on if flag detected
#     else:
#         pass
#
#     node.flush()
#     counter += 1
#     print(counter)

node.send_set_variables(stop)
node.flush()

# test = local_nav.sensor_val_to_cm_dist(2005)
# print(test)
#
# plt.figure(figsize=(5,5))
# plt.plot(sensor_measurements, sensor_distances)
# plt.ylabel("Distance in cm")
# plt.xlabel("Sensor values")
# plt.title("Distance corresponding to the sensor value")
# plt.show()
#
i=0

sensor_vals = local_nav.get_sensor_data()
all_sens_vals = [[sensor_vals]]
# while i < 10:
#
#     new_sensor_vals = local_nav.get_sensor_data()
#     all_sens_vals.append(new_sensor_vals)
#     print("sensed",all_sens_vals)
#     i += 1
#
obstacles_pos = local_nav.obstacles_pos_from_sensor_vals(sensor_vals)
#
# plt.figure(figsize=(7,7))
# plt.title("Local occupancy grid placing the detected obstacles arround the Thymio")
# plt.xlabel("Distance in cm perpendicular to the Thymio orientation")
# plt.ylabel("Distance in cm parallel to the Thymio orientation")
#
# plt.plot(thymio_coords[:,0], thymio_coords[:,1])
# plt.axis("equal")
# plt.scatter(obstacles_pos[:,0], obstacles_pos[:,1], marker="o", color="r")
# plt.show()

from Global_Nav import Global_Nav


thymio_size = 100 #size of thymio [TODO]
r = 2 # radius of the wheel (cm)
L = 9.5 # distance from wheel to wheel (cm)
v_max = 20 # cm/s
cruising = 300
motor_speed_max = 500
omega_to_motor = (motor_speed_max*r)/v_max
pixel_to_cm = 1/6

Kp = 0.7   # Proporcional gain
Kd = 0.05  # Derivative gain
previous_error = 0  # Derivative error

ts = 0.1  # time step
A = np.array([[0, 1, 0, 0, 0, 0],
              [1, ts, 0, 0, 0, 0],
              [0, 0, 0, 1, 0, 0],
              [0, 0, 1, ts, 0, 0],
              [0, 0, 0, 0, 0, 1],
              [0, 0, 0, 0, 1, ts]])

P0 = Global_Nav.covariance(1000,1000,1000,1000,1000,1000)  # state covariance matrix
Q = Global_Nav.covariance(0.0615, 0.04, 0.0615, 0.04, 0.001, 0.01)  # process noise covariance matrix
R = Global_Nav.covariance(0.0615, 0.1, 0.0615, 0.1, 0.1, 0.01)  # measurement covariance matrix

client = ClientAsync()
node = client.aw(client.wait_for_node())

# Define global_nav object with class attributes
GN = Global_Nav(client, node, cruising, ts, Kp, Kd, previous_error, r, L, omega_to_motor, pixel_to_cm, R, Q, A, P0)

ts = 0.1
motor_sensor = GN.get_motor_speed()
speed_sensor = motor_sensor/GN.omega_to_motor

X = np.array([      [0.0],     # x_dot0
                    [0],    # x0
                    [0.0],    # y_dot0
                    [0],    # y0
                    [0.0],    # phi_dot0
                    [0.0]])   # phi0


X[4] = (r/L)*(speed_sensor[1]-speed_sensor[0])               # phi_dot
X[5] = error                                                 # phi
X[0] = (r/2)*(speed_sensor[0]+speed_sensor[1])*np.cos(X[5])  # x_dot
X[2] = (r/2)*(speed_sensor[0]+speed_sensor[1])*np.sin(X[5])  # y_dot
X[1] += X[0]*ts                                              # x
X[3] += X[2]*ts                                              # y

from local_occupancy import map_sensor_vals, rel_dpos

###------------------------------------------------------
# STEP 1
###------------------------------------------------------

# Arbitrarily define the initial robot position as the origin of the map
abs_pos = [[0, 0, math.pi / 2]]  # List of lists that will contain the absolute x,y,theta coordinates of the robot

###------------------------------------------------------
# STEP 2
###------------------------------------------------------

# Provided the relative positions, compute the absolute positions at each step
for (dx, dy, dtheta) in rel_dpos[:]:
    (x, y, theta) = abs_pos[-1][0], abs_pos[-1][1], abs_pos[-1][2]
    d = np.sqrt(dx ** 2 + dy ** 2)
    new_pos = [x + d * np.cos(theta + dtheta), y + d * np.sin(theta + dtheta), (theta + dtheta) % (2 * math.pi)]
    # Appending the computed absolute x, y and theta coordinates to the list
    abs_pos.append(new_pos)

abs_pos = np.array(abs_pos)

###------------------------------------------------------
# STEP 3
###------------------------------------------------------

# Compute the local occupancy grid from the sensor values at each step
local_occupancy_grids = [local_nav.obstacles_pos_from_sensor_vals(x) for x in map_sensor_vals]

###------------------------------------------------------
# STEP 4
###------------------------------------------------------

# Create the global map based on the data acquired previously
global_map, overall_thymio_coords = [], []

for (local_grid, pos) in zip(local_occupancy_grids, abs_pos):
    # Rotate the local occupancy grid
    rotated_grid = local_nav.rotate(pos[2] - math.pi / 2, local_grid)
    rotated_thymio_coords = local_nav.rotate(pos[2] - math.pi / 2, thymio_coords)

    # Translate the grid at the position of the Thymio
    obstacles_pos = rotated_grid + np.array([pos[0], pos[1]])
    abs_Thymio_coords = rotated_thymio_coords + np.array([pos[0], pos[1]])

    # Store position of the obstacles and Thymio in the global map
    global_map.append(obstacles_pos)
    overall_thymio_coords.append(abs_Thymio_coords)

###------------------------------------------------------
# STEP 5
###------------------------------------------------------

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