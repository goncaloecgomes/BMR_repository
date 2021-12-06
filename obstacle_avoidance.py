## ------------------ Module for Obstale Avoidance for Thymio Robot - Basics of Mobile Robotics --------------------##

import tdmclient.notebook
# await tdmclient.notebook.start()

#Commented are bits of code from exercise session 4 for inspiration....

## ------------------ Code 1 -------------##
# Environment : Gray-Scale Gradient paper
# Goal is a dark contrast, Thymio uses sensors to go up the gradient scale (starts in light gray region)

# %%run_python
#
# BASICSPEED = 0
# GAIN       = 0
#
# @onevent
# def prox():
#     global prox_ground_delta, motor_left_target, motor_right_target, BASICSPEED, GAIN
#     diff = prox_ground_delta[1] - prox_ground_delta[0]
#     motor_left_target = BASICSPEED - diff*GAIN
#     motor_right_target = BASICSPEED + diff*GAIN


## ------------------ Code 2 -------------##
# Environment : Gray-Scale Gradient paper
# Goal is same as previous, but now aims to use side sensors to avoid obstacle on sheet

# %% run_python

# speed0 = 100  # nominal speed
# speedGain = 2  # gain used with ground gradient
# obstThrL = 10  # low obstacle threshold to switch state 1->0
# obstThrH = 20  # high obstacle threshold to switch state 0->1
# obstSpeedGain = 5  # /100 (actual gain: 5/100=0.05)
#
# state = 1  # 0=gradient, 1=obstacle avoidance
# obst = [0, 0]  # measurements from left and right prox sensors
#
# timer_period[0] = 10  # 10ms sampling time
#
#
# @onevent
# def timer0():
#     global prox_ground_delta, prox_horizontal, motor_left_target, motor_right_target, state, obst, obstThrH, obstThrL, obstSpeedGain, speed0, speedGain
#     # acquisition from ground sensor for going toward the goal
#     diffDelta = prox_ground_delta[1] - prox_ground_delta[0]
#
#     # acquisition from the proximity sensors to detect obstacles
#     obst = [prox_horizontal[0], prox_horizontal[4]]
#
#     # tdmclient does not support yet multiple and/or in if statements:
#     if state == 0:
#         # switch from goal tracking to obst avoidance if obstacle detected
#         if (obst[0] > obstThrH):
#             state = 1
#         elif (obst[1] > obstThrH):
#             state = 1
#     elif state == 1:
#         if obst[0] < obstThrL:
#             if obst[1] < obstThrL:
#                 # switch from obst avoidance to goal tracking if obstacle got unseen
#                 state = 0
#     if state == 0:
#         # goal tracking: turn toward the goal
#         leds_top = [0, 0, 0]
#         motor_left_target = speed0 - speedGain * diffDelta
#         motor_right_target = speed0 + speedGain * diffDelta
#     else:
#         leds_top = [30, 30, 30]
#         # obstacle avoidance: accelerate wheel near obstacle
#         motor_left_target = speed0 + obstSpeedGain * (obst[0] // 100)
#         motor_right_target = speed0 + obstSpeedGain * (obst[1] // 100)