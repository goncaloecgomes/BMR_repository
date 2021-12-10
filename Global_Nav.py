from tdmclient import ClientAsync
import cv2 as cv
import numpy as np
from pyzbar.pyzbar import decode

# client = ClientAsync()

# node = client.aw(client.wait_for_node())

class Global_Nav():

    def __init__(self, client, node, cruising, time_step, Kp, Ki, Kd, error_sum, previous_error, r, L, omega_to_motor, R, Q, A, P0):
        self.motor_cruising = np.array([cruising, cruising])
        self.K = np.array([Kp, Ki, Kd])
        self.r = r
        self.L = L
        self.omega_to_motor = omega_to_motor
        self.R = R
        self.Q = Q
        self.A = A
        self.P_est = P0
        self.client = client
        self.ts = time_step
        self.error_sum = error_sum
        self.previous_error = previous_error
        self.node = node 


    def PIDcontroller(self, phi_d, phi):
        """ PID controller """
        error = phi_d - phi
        print(error)
        self.error_sum += error
        error_dif = error - self.previous_error
        self.previous_error = error

        omega = self.K[0]*error + self.K[1]*self.error_sum + self.K[2]*error_dif

        v = ((self.motor_cruising[0] + self.motor_cruising[1])/2)/(self.omega_to_motor*self.r)
        #v=0

        vr = (2*v+omega*self.L)/(2*self.r) # angular velocity of the right wheel
        vl = (2*v-omega*self.L)/(2*self.r) # angular velocity of the left wheel
        return vr, vl


    def get_motor_speed(self):
        self.client.aw(self.node.wait_for_variables({"motor.left.speed"}))
        left_motor_speed = self.node.v.motor.left.speed
        self.client.aw(self.node.wait_for_variables({"motor.right.speed"}))
        right_motor_speed = self.node.v.motor.right.speed
        return  np.array([left_motor_speed, right_motor_speed])


    def covariance(sigmax, sigmaxdot, sigmay, sigmaydot, sigmatheta, sigmathetadot):
        """creates diagonal covariance matrix of errors"""
        cov_matrix = np.array([[sigmax ** 2, 0, 0, 0, 0 ,0],
                               [0, sigmaxdot ** 2, 0, 0, 0, 0],
                               [0, 0, sigmay ** 2, 0, 0, 0],
                               [0, 0, 0, sigmaydot ** 2, 0, 0],
                               [0, 0, 0, 0, sigmatheta ** 2, 0],
                               [0, 0, 0, 0, 0, sigmathetadot ** 2]])
        return cov_matrix


    def kalman_filter(self, X):
        """takes the previous state (position and velocity in x and y) and covariance matrices  and uses the current sensor\
        data to update the state and covariance matrices i.e. get the robots current location"""
        # Use the previous state to predict the new state
        X_est_a_priori = np.dot(self.A, X)

        # Use the previous covariance to predict the new covariance
        P_est_a_priori = np.dot(self.A, np.dot(self.P_est, self.A.T))
        P_est_a_priori = P_est_a_priori + self.Q if type(self.Q) != type(None) else P_est_a_priori

        # Define Y and H for the posteriori update
        # X = np.array([[xdot], [x], [ydot], [y], [phi_dot], [phi]])
        H = np.identity(6)

        # innovation / measurement residual
        i = X - np.dot(H, X_est_a_priori)
        S = np.dot(H, np.dot(P_est_a_priori, H.T)) + self.R

        # Calculate the Kalman gain
        K = np.dot(P_est_a_priori, np.dot(H.T, np.linalg.inv(S)))

        # Update the state and covariance matrices with the current sensor data
        X = X_est_a_priori + np.dot(K, i)
        self.P_est = P_est_a_priori - np.dot(K, np.dot(H, P_est_a_priori))
        return X


    def find_thymio(self, img, pixel_to_cm):
        img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)    
        # create a mask based on the threshold
        binary_mask = img_gray > 100
        bin_img = np.zeros_like(img)
        bin_img[binary_mask] = img[binary_mask]
        binary_mask = bin_img != 0
        white = np.ones_like(img)*255
        bin_img[binary_mask] = white[binary_mask]
        result = decode(bin_img)
        if len(result)>=1:
            qr_loc = result[0][3]
            thymio_xy_pix = np.array([int((qr_loc[0].x+qr_loc[1].x+qr_loc[2].x+qr_loc[3].x)/4), 
                                    int((qr_loc[0].y+qr_loc[1].y+qr_loc[2].y+qr_loc[3].y)/4)]) #in pixels
            thymio_xy = thymio_xy_pix*pixel_to_cm # cm

            # Orientation
            if str(result[0][4])=='ZBarOrientation.RIGHT':
                if np.abs(qr_loc[3].y-qr_loc[0].y) < np.abs(qr_loc[1].y-qr_loc[0].y):
                    mid_point_back = np.array([int((qr_loc[0].x+qr_loc[1].x)/2), int((qr_loc[0].y+qr_loc[1].y)/2)])
                else:
                    mid_point_back = np.array([int((qr_loc[0].x+qr_loc[3].x)/2), int((qr_loc[0].y+qr_loc[3].y)/2)])
            
            elif str(result[0][4])=='ZBarOrientation.DOWN':
                if np.abs(qr_loc[0].y-qr_loc[3].y) > np.abs(qr_loc[1].y-qr_loc[0].y):
                    mid_point_back = np.array([int((qr_loc[2].x+qr_loc[3].x)/2), int((qr_loc[2].y+qr_loc[3].y)/2)])
                else:
                    mid_point_back = np.array([int((qr_loc[0].x+qr_loc[3].x)/2), int((qr_loc[0].y+qr_loc[3].y)/2)])

            elif str(result[0][4])=='ZBarOrientation.LEFT':
                if np.abs(qr_loc[0].y-qr_loc[3].y) > np.abs(qr_loc[1].y-qr_loc[0].y):
                    mid_point_back = np.array([int((qr_loc[2].x+qr_loc[1].x)/2), int((qr_loc[2].y+qr_loc[1].y)/2)])
                else:
                    mid_point_back = np.array([int((qr_loc[2].x+qr_loc[3].x)/2), int((qr_loc[2].y+qr_loc[3].y)/2)])      
            
            elif str(result[0][4])=='ZBarOrientation.UP':
                if np.abs(qr_loc[3].y-qr_loc[0].y) > np.abs(qr_loc[1].y-qr_loc[0].y):
                    mid_point_back = np.array([int((qr_loc[0].x+qr_loc[1].x)/2), int((qr_loc[0].y+qr_loc[1].y)/2)])
                else:
                    mid_point_back = np.array([int((qr_loc[2].x+qr_loc[1].x)/2), int((qr_loc[2].y+qr_loc[1].y)/2)])

            thymio_vec = thymio_xy_pix - mid_point_back
            phi = np.arctan(thymio_vec[1]/thymio_vec[0])
            thymio_pose = np.array([thymio_xy[0], thymio_xy[1], phi])
            flag = 1
        else:
            thymio_xy_pix = np.zeros((1,3))
            thymio_pose = np.zeros((1,3))
            qr_loc = np.zeros((1,3))
            flag = 0
        return thymio_xy_pix, thymio_pose, qr_loc, flag


    def motors(self, motor_left, motor_right):
        return {
            "motor.left.target": [int(motor_left)],
            "motor.right.target": [int(motor_right)],
        }



