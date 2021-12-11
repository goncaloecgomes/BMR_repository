# Import libraries
import sys
sys.setrecursionlimit(5000)
import numpy as np
import cv2 as cv
from tdmclient import ClientAsync
from Local_Nav import Local_Nav as LN
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

vid = cv.VideoCapture(1 + cv.CAP_DSHOW)

count = 0 
while count<50:
    ret, img = vid.read()
    count += 1


path, mid_point_back = GN.get_shortest_path(img, thymio_size)

# After the loop release the cap object
vid.release()

# Inicial states
path_cm = path*pixel_to_cm
thymio_pix = np.array([path[0][0], path[0][1]])
thymio_pose = np.array([path_cm[0][0], path_cm[0][1], 0.0])

X = np.array([      [0.0],     # x_dot0
        [thymio_pose[0]],    # x0
                    [0.0],    # y_dot0
        [thymio_pose[1]],    # y0
                    [0.0],    # phi_dot0
                    [0.0]])   # phi0 

vid = cv.VideoCapture(1+ cv.CAP_DSHOW)  
next_point = 1

while(True):
    isTrue, img = vid.read()      
    while (np.abs(X[3]-path_cm[-1][1]) > 1) or (np.abs(X[1]-path_cm[-1][0]) > 1):
        
        # define current goal
        goal_cm = np.array([path_cm[next_point][0],path_cm[next_point][1]])
        goal = np.array([path[next_point][0],path[next_point][1]])

        while (np.abs(X[3]-goal_cm[1]) > 1) or (np.abs(X[1]-goal_cm[0]) > 1):
            isTrue, img = vid.read()
            if isTrue:
                node = client.aw(client.wait_for_node()) 
                client.aw(node.lock_node())

                # vector from the mid back point of the thymio to the current goal
                thymio2goal = goal - mid_point_back 

                # vector from the mid back point of the thymio to the centre of the thymio
                orientation_vec = thymio_pix - mid_point_back
                
                # Find angle through dot product
                norm_1 = np.linalg.norm(orientation_vec)
                norm_2 = np.linalg.norm(thymio2goal)
                dot_product = np.dot(orientation_vec , thymio2goal)
                error = np.arccos(dot_product/(norm_1*norm_2))

                # Find whether the angle is positive or negative
                d = (thymio_pix[0]-mid_point_back[0])*(goal[1]-mid_point_back[1])-(thymio_pix[1]-mid_point_back[1])*(goal[0]-mid_point_back[0])
                if d > 0 :
                    error = -error
                
                # Control action to make the Thymio follow the reference
                vr, vl = GN.PDcontroller(error)
                
                motor_left = vl*omega_to_motor
                motor_right = vr*omega_to_motor 

                # Saturation
                if motor_left > cruising:
                    motor_left = cruising
                if motor_right > cruising:
                    motor_right = cruising
                if motor_right < -cruising:
                    motor_right = -cruising
                if motor_left < -cruising:
                    motor_left = -cruising   
               
                #Activate Obstacle avoidance if necessary
                local_nav = LN(client, node, cruising, cruising)
                flag_obs = local_nav.analyse_data()
                if flag_obs == 1:
                    task = local_nav.obstacle_avoidance2() #trigger task depending on if flag detected
                else:
                    adjust_speed = GN.motors(motor_left, motor_right)
                    print(adjust_speed)
                    node.send_set_variables(adjust_speed)
                    node.flush()    
                
                # Update states
                motor_sensor = GN.get_motor_speed()
                speed_sensor = motor_sensor/GN.omega_to_motor      # cm/s
                thymio_pix_new,thymio_pose,_,flag,mid_point_back_new = GN.find_thymio(img, pixel_to_cm)
                if flag==1: # if the camera can find the thymio
                    X[1] = thymio_pose[0]                                          # x
                    X[3] = thymio_pose[1]                                          # y
                    X[5] = error#thymio_pose[2]                                    # phi
                    X[4] = (r/L)*(speed_sensor[1]-speed_sensor[0])                 # phi_dot
                    X[0] = (r/2)*(speed_sensor[0]+speed_sensor[1])*np.cos(X[5])  # x_dot
                    X[2] = (r/2)*(speed_sensor[0]+speed_sensor[1])*np.sin(X[5])  # y_dot
                    thymio_pix = thymio_pix_new
                    mid_point_back = mid_point_back_new
                else:
                    X[4] = (r/L)*(speed_sensor[1]-speed_sensor[0])               # phi_dot
                    X[5] = error                                                 # phi
                    X[0] = (r/2)*(speed_sensor[0]+speed_sensor[1])*np.cos(X[5])  # x_dot
                    X[2] = (r/2)*(speed_sensor[0]+speed_sensor[1])*np.sin(X[5])  # y_dot
                    X[1] += X[0]*ts                                              # x
                    X[3] += X[2]*ts                                              # y
                    # thymio_pix[0] = X[1]/pixel_to_cm ---> uncomment all if the Kalman filter isn't being used
                    # thymio_pix[1] = X[3]/pixel_to_cm
                    # mid_point_back[0] += X[0]*ts 
                    # mid_point_back[1] += X[1]*ts
                X = GN.kalman_filter(X)
                thymio_pix[0] = X[1]/pixel_to_cm
                thymio_pix[1] = X[3]/pixel_to_cm
                mid_point_back[0] += X[0]*ts 
                mid_point_back[1] += X[1]*ts

                img = cv.circle(img, (int(thymio_pix[0]), int(thymio_pix[1])), radius=10, color=(0, 0, 255), thickness=-1)
                img = cv.circle(img, (goal[0], goal[1]), radius=10, color=(255, 0, 0), thickness=-1)
                
                cv.imshow("Video", img)
                # the 'q' button is set as the
                # quitting button you may use any
                # desired button of your choice
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break

        next_point += 1
        img = cv.circle(img, (int(thymio_pix[0]), int(thymio_pix[1])), radius=10, color=(0, 0, 255), thickness=-1)
        img = cv.circle(img, (goal[0], goal[1]), radius=10, color=(255, 0, 0), thickness=-1)
        cv.imshow("Video", img)
        # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    motor_left = 0
    motor_right = 0
    adjust_speed = GN.motors(motor_left, motor_right)
    node.send_set_variables(adjust_speed)
    node.flush()
    img = cv.circle(img, (int(thymio_pix[0]), int(thymio_pix[1])), radius=10, color=(0, 0, 255), thickness=-1)
    img = cv.circle(img, (goal[0], goal[1]), radius=10, color=(255, 0, 0), thickness=-1)
    cv.imshow("Video", img)
    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
    
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv.destroyAllWindows()


