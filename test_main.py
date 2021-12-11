# Import libraries
import numpy as np
import cv2 as cv
from tdmclient import ClientAsync
from Local_Nav import Local_Nav as LN
from Global_Nav import Global_Nav 
# import RepeatedTimer

r = 2 # radius of the wheel (cm)
L = 9.5 # distance from wheel to wheel (cm)
v_max = 20 # cm/s
cruising = 50 
motor_speed_max = 500
omega_to_motor = (motor_speed_max*r)/v_max
pixel_to_cm = 1/6

Kp = 1   # Proporcional gain
Ki = 0.01  # Integral gain
Kd = 1    # Derivative gain
error_sum = 0       # Integral error
previous_error = 0  # Derivative error

ts = 0.1  # time step
A = np.array([[1, ts, 0, 0, 0, 0],  # state transition matrix
              [0, 1, 0, 0, 0, 0],
              [0, 0, 1, ts, 0, 0],
              [0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 1, ts],
              [0, 0, 0, 0, 0, 1]])

P0 = Global_Nav.covariance(20, 5, 20, 5, 20, 5)  # state covariance matrix

Q = Global_Nav.covariance(0.01, 0.1, 0.01, 0.1, 0.001, 0.01)  # process noise covariance matrix

R = Global_Nav.covariance(1, 0.5, 1, 0.5, 0.1, 0.01)  # measurement covariance matrix

client = ClientAsync()
node = client.aw(client.wait_for_node())

# Define global_nav object with class attributes
GN = Global_Nav(client, node, cruising, ts, Kp, Ki, Kd, error_sum, previous_error, r, L, omega_to_motor, R, Q, A, P0)


#---------------------------------------------------------------------------
#                                 Find shortest Path
#---------------------------------------------------------------------------

# JUST FOR TESTING\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
path = np.array([400,250])
vid = cv.VideoCapture(1 + cv.CAP_DSHOW)

while True:
    isTrue, img = vid.read()
    if isTrue:
        # cv.imshow("Video", img)
        # # the 'q' button is set as the
        # # quitting button you may use any
        # # desired button of your choice
        # if cv.waitKey(1) & 0xFF == ord('q'):
            #break
        thymio_xy_pix,thymio_pose,_,flag = GN.find_thymio(img, pixel_to_cm)
        if flag==1:
            break

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv.destroyAllWindows()
print('thymio found')
#\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

# Inicial states
X = np.array([      [0],     # x_dot0
        [thymio_pose[0]],    # x0
                     [0],    # y_dot0
        [thymio_pose[1]],    # y0
                     [0],    # phi_dot0
        [thymio_pose[2]]])   # phi0 

path_cm = path*pixel_to_cm
vid = cv.VideoCapture(1+ cv.CAP_DSHOW)  
while(True):
    isTrue, img = vid.read()      
    next_point = 0
    while (np.abs(X[3]-path_cm[1]) > 0) and (np.abs(X[1]-path_cm[0]) > 0):
        #isTrue, img = vid.read()
        while (np.abs(X[3]-path_cm[next_point+1]) > 0) and (np.abs(X[1]-path_cm[next_point]) > 0):
            isTrue, img = vid.read()
            if isTrue:
                # cv.imshow("Video", img)
                # # the 'q' button is set as the
                # # quitting button you may use any
                # # desired button of your choice
                # if cv.waitKey(1) & 0xFF == ord('q'):
                #     break
                node = client.aw(client.wait_for_node()) 
                client.aw(node.lock_node())
                #await node.lock_node()
                # Calculate desired angle
                phi_d = np.arctan((path[next_point+1]-X[3])/(path[next_point]-X[1]))
                # Control action to make the Thymio follow the reference
                vr, vl = GN.PIDcontroller(phi_d, X[5])
                #print(vr, vl)
                motor_left = vl*omega_to_motor
                motor_right = vr*omega_to_motor            
                # Saturation
                if motor_left > 500:
                    motor_left = 500
                elif motor_right > 500:
                    motor_right = 500
                elif motor_right < -500:
                    motor_right = -500
                elif motor_left < -500:
                    motor_left = -500            
                # Activate Obstacle avoidance if necessary
                # Define local_nav object with class attributes
                # local_nav = LN(client, node, cruising, cruising)
                # flag_obs = local_nav.analyse_data()
                # if flag_obs == 1:
                #     task = local_nav.obstacle_avoidance() #trigger task depending on if flag detected
                # else:
                adjust_speed = GN.motors(motor_left, motor_right)
                node.send_set_variables(adjust_speed)
                node.flush()
                # Update states
                motor_sensor = GN.get_motor_speed()
                speed_sensor = motor_sensor/GN.omega_to_motor      # cm/s
                thymio_xy_pix,thymio_pose,_,flag = GN.find_thymio(img, pixel_to_cm)
                if flag==1: # if the camera can find the thymio
                    X[1] = thymio_pose[0]                                          # x
                    X[3] = thymio_pose[1]                                          # y
                    X[5] = thymio_pose[2]                                          # phi
                    X[4] = (r/L)*(speed_sensor[1]-speed_sensor[0])                 # phi_dot
                    X[0] = (r/2)*(speed_sensor[0]+speed_sensor[1])*np.cos(X[5])  # x_dot
                    X[2] = (r/2)*(speed_sensor[0]+speed_sensor[1])*np.sin(X[5])  # y_dot
                    # Draw a point on the frame
                    #img = cv.circle(img, (thymio_xy_pix[0], thymio_xy_pix[1]), radius=10, color=(0, 0, 255), thickness=-1)
                    img = cv.circle(img, (path[next_point], path[next_point+1]), radius=10, color=(255, 0, 0), thickness=-1)
                else:
                    X[4] = (r/L)*(speed_sensor[1]-speed_sensor[0])               # phi_dot
                    X[5] += X[4]*ts                                              # phi
                    X[0] = (r/2)*(speed_sensor[0]+speed_sensor[1])*np.cos(X[5])  # x_dot
                    X[2] = (r/2)*(speed_sensor[0]+speed_sensor[1])*np.sin(X[5])  # y_dot
                    X[1] += X[0]*ts                                              # x
                    X[3] += X[3]*ts                                              # y
                #img = cv.circle(img, (thymio_xy_pix[0], thymio_xy_pix[1]), radius=10, color=(0, 0, 255), thickness=-1)
                #img = cv.circle(img, (path[next_point], path[next_point+1]), radius=10, color=(255, 0, 0), thickness=-1)
                cv.imshow("Video", img)
                # the 'q' button is set as the
                # quitting button you may use any
                # desired button of your choice
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break
                #X = GN.kalman_filter(X)
                print(X[1]/pixel_to_cm, X[3]/pixel_to_cm)
        next_point += 1
        #img = cv.circle(img, (thymio_xy_pix[0], thymio_xy_pix[1]), radius=10, color=(0, 0, 255), thickness=-1)
        #img = cv.circle(img, (path[next_point], path[next_point+1]), radius=10, color=(255, 0, 0), thickness=-1)
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
    #img = cv.circle(img, (thymio_xy_pix[0], thymio_xy_pix[1]), radius=10, color=(0, 0, 255), thickness=-1)
    #img = cv.circle(img, (path[next_point], path[next_point+1]), radius=10, color=(255, 0, 0), thickness=-1)
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


