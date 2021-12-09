import numpy as np



def covariance(sigmax, sigmaxdot, sigmay, sigmaydot, sigmatheta, sigmathetadot):
    """creates diagonal covariance matrix of errors"""
    cov_matrix = np.array([[sigmax ** 2, 0, 0, 0, 0 ,0],
                           [0, sigmaxdot ** 2, 0, 0, 0, 0],
                           [0, 0, sigmay ** 2, 0, 0, 0],
                           [0, 0, 0, sigmaydot ** 2, 0, 0],
                           [0, 0, 0, 0, sigmatheta ** 2, 0],
                           [0, 0, 0, 0, 0, sigmathetadot ** 2]])
    return cov_matrix


def kalman_filter(X_est_prev, P_est_prev, Q, R,  x, xdot, y, ydot, theta, theta_dot):
    """takes the previous state (position and velocity in x and y) and covariance matrices  and uses the current sensor\
    data to update the state and covariance matrices i.e. get the robots current location"""
    # Use the previous state to predict the new state
    X_est_a_priori = np.dot(A, X_est_prev)

    # Use the previous covariance to predict the new covariance
    P_est_a_priori = np.dot(A, np.dot(P_est_prev, A.T))
    P_est_a_priori = P_est_a_priori + Q if type(Q) != type(None) else P_est_a_priori

    # Define Y and H for the posteriori update
    Y = np.array([[x], [xdot], [y], [ydot], [theta], [theta_dot]])
    H = np.identity(6)

    # innovation / measurement residual
    i = Y - np.dot(H, X_est_a_priori)
    S = np.dot(H, np.dot(P_est_a_priori, H.T)) + R

    # Calculate the Kalman gain
    K = np.dot(P_est_a_priori, np.dot(H.T, np.linalg.inv(S)))

    # Update the state and covariance matrices with the current sensor data
    X_est = X_est_a_priori + np.dot(K, i)
    P_est = P_est_a_priori - np.dot(K, np.dot(H, P_est_a_priori))

    return X_est, P_est


