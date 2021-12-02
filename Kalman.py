import numpy as np

ts = 0.1  # time step
A = np.array([[1, ts, 0, 0],  # state transition matrix
              [0, 1, 0, 0],
              [0, 0, 1, ts],
              [0, 0, 0, 1]])


def covariance(sigmax, sigmaxdot, sigmay, sigmaydot):
    """creates diagonal covariance matrix of errors"""
    cov_matrix = np.array([[sigmax ** 2, 0, 0, 0],
                           [0, sigmaxdot ** 2, 0, 0],
                           [0, 0, sigmay ** 2, 0],
                           [0, 0, 0, sigmaydot ** 2]])
    return cov_matrix


def kalman_filter(X_est_prev, P_est_prev, Q, R,  x, xdot, y, ydot):
    """takes the previous state (position and velocity in x and y) and covariance matrices  and uses the current sensor\
    data to update the state and covariance matrices i.e. get the robots current location"""
    # Use the previous state to predict the new state
    X_est_a_priori = np.dot(A, X_est_prev)

    # Use the previous covariance to predict the new covariance
    P_est_a_priori = np.dot(A, np.dot(P_est_prev, A.T))
    P_est_a_priori = P_est_a_priori + Q if type(Q) != type(None) else P_est_a_priori

    # Define Y and H for the posteriori update
    Y = np.array([[x], [xdot], [y], [ydot]])
    H = np.identity(4)

    # innovation / measurement residual
    i = Y - np.dot(H, X_est_a_priori)
    S = np.dot(H, np.dot(P_est_a_priori, H.T)) + R

    # Calculate the Kalman gain
    K = np.dot(P_est_a_priori, np.dot(H.T, np.linalg.inv(S)))

    # Update the state and covariance matrices with the current sensor data
    X_est = X_est_a_priori + np.dot(K, i)
    P_est = P_est_a_priori - np.dot(K, np.dot(H, P_est_a_priori))

    return X_est, P_est


