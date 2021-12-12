import math
import numpy as np

## Sensor measurements
sensor_distances = np.array([15, 9.6, 9, 8.5, 7.5, 7, 6, 5, 4, 3, 2.5, 2, 1.5, 0.5, 0.2, 0])
sensor_measurements = np.array([0, 1186,1420, 1510, 1715, 1830, 2069, 2250, 2468, 2850, 3155, 3468, 3940, 4355, 4432, 4500])

#sensor_distances = np.array([0, 30])
#sensor_measurements = np.array([5120, 0])

# Thymio outline
center_offset = np.array([5.5,5.5])
thymio_coords = np.array([[0,0], [11,0], [11,8.5], [10.2, 9.3], 
                          [8, 10.4], [5.5,11], [3.1, 10.5], 
                          [0.9, 9.4], [0, 8.5], [0,0]])-center_offset

# Sensor positions and orientations
sensor_pos_from_center = np.array([[0.9,9.4], [3.1,10.5], [5.5,11.0], [8.0,10.4], [10.2,9.3], [2.5,0], [8.5,0]])-center_offset
sensor_angles = np.array([120, 105, 90, 75, 60, -90, -90])*math.pi/180


# Map construction exercise values 

rel_dpos = np.array([[0, 1, 0],
                     [0, 1, 0],
                     [0, 1, 0], 
                     [0, 1, 0],
                     [0, 0, 90],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 0, 90],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 0, 90],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 0, 90],
                     [0, 1, 0],
                     [0, 0, -90],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 0, -90],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 0, -90],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 0, -90],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 1, 0],
                     [0, 1, 0]], dtype=float)

rel_dpos[:, :2] = rel_dpos[:, :2]*20
rel_dpos[:, 2] = rel_dpos[:, 2]*math.pi/180


map_sensor_vals = [[0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [3276.75, 3947.08, 4286.0, 3907.68, 3243.2, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [3276.75, 3947.08, 4286.0, 3907.68, 3243.2, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 4286.0, 4286.0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [3276.75, 3947.08, 4286.0, 3907.68, 3243.2, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 4286.0, 4286.0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [3276.75, 3947.08, 4286.0, 3907.68, 3243.2, 0, 0],
                     [0, 0, 0, 0, 0, 4286.0, 4286.0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 2413.4700000000003, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 2416.2000000000003, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0]]
