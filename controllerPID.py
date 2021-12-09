import numpy as np
import kalmanFilter as kl

def PIDcontroller(ref, pose, Kp, Ki, Kd, error_sum, previous_error, R, L, v):
    """ PID controller """

    # Inputs: ref            -> tracking ference
    #         pose           -> current position
    #         Kp             -> Proportional gain
    #         Ki             -> Integral gain 
    #         Kd             -> Derivational gain
    #         error_sum      -> Cumulative error
    #         previous_error -> Previous error

    error = ref - pose
    error_sum += error
    error_dif = error - previous_error
    previous_error = error

    omega = Kp*error + Ki*error_sum + Kd*error_dif

    vr = (2*v+omega*L)/(2*R) # angular velocity of the right wheel
    vl = (2*v-omega*L)/(2*R) # angular velocity of the left wheel

    return vr, vl, error_sum, previous_error

