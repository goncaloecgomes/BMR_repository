import numpy as np
from numpy.linalg import inv

ts = 0.1
A = np.array([[1, ts, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 1, ts],
              [0, 0, 0, 1]])
Q = np.array([[qx, 0, 0, 0],
              [0, qxdot, 0, 0],
              [0, 0, qy, 0],
              [0, 0, 0, qydot]]);


def covariance(sigmax, sigmaxdot, sigmay,sigmaydot):
    cov_matrix = np.array([[sigmax ** 2, 0, 0, 0],
                           [0, sigmaxdot ** 2, 0, 0],
                           [0, 0, sigmay ** 2, 0],
                           [0, 0, 0, sigmaydot ** 2]])
    return cov_matrix

def kalman_filter(X_est_prev, P_est_prev, Q, x, xdot, y, ydot):

    # Use the previous state to predict the new state
    X_est_a_priori = np.dot(A, X_est_prev)
    P_est_a_priori = np.dot(A, np.dot(P_est_prev, A.T))
    P_est_a_priori = P_est_a_priori + Q if type(Q) != type(None) else P_est_a_priori

    # Calculate the Kalman gain
    Y = np.array([[x], [xdot], [y], [ydot]])
    H = np.identity(4)
    R = covariance(rx, rxdot, ry, rydot)


    # innovation/measurement residual
    i = Y - np.dot(H, X_est_a_priori)
    S = np.dot(H, np.dot(P_est_a_priori, H.T)) + R

    # Kalman gain
    K = np.dot(P_est_a_priori, np.dot(H.T, np.linalg.inv(S)))

    # a posteriori estimate
    X_est = X_est_a_priori + np.dot(K, i)
    P_est = P_est_a_priori - np.dot(K, np.dot(H, P_est_a_priori))

    return X_est, P_est



