# ## ------------------ Module for Obstale Avoidance for Thymio Robot - Basics of Mobile Robotics --------------------##

from tdmclient import ClientAsync
import Local_Nav as LN
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from local_occupancy import sensor_measurements, sensor_distances
from local_occupancy import thymio_coords, sensor_pos_from_center, sensor_angles

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

test = local_nav.sensor_val_to_cm_dist(2005)
print(test)

plt.figure(figsize=(5,5))
plt.plot(sensor_measurements, sensor_distances)
plt.ylabel("Distance in cm")
plt.xlabel("Sensor values")
plt.title("Distance corresponding to the sensor value")
plt.show()

sensor_vals = local_nav.get_sensor_data()

obstacles_pos = local_nav.obstacles_pos_from_sensor_vals(sensor_vals)

plt.figure(figsize=(7,7))
plt.title("Local occupancy grid placing the detected obstacles arround the Thymio")
plt.xlabel("Distance in cm perpendicular to the Thymio orientation")
plt.ylabel("Distance in cm parallel to the Thymio orientation")

plt.plot(thymio_coords[:,0], thymio_coords[:,1])
plt.axis("equal")
plt.scatter(obstacles_pos[:,0], obstacles_pos[:,1], marker="o", color="r")
plt.show()