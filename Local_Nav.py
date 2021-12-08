from tdmclient import ClientAsync


client = ClientAsync()

node = client.aw(client.wait_for_node())
#
class Local_Nav:

    def __init__(self,client,node):#,client,node):#, motor_speed_left, motor_speed_right):
        # self.motor_speed_left = motor_speed_left
        # self.motor_speed_right = motor_speed_right
        self.client = client
        self.node = node #self.client.aw(self.client.wait_for_node())

    def get_sensor_data(self):

        self.client.aw(self.node.wait_for_variables({"prox.horizontal"}))
        prox_sens = self.node.v.prox.horizontal
        return  list(prox_sens)

    def analyse_data(self):

        prox_sens_data = self.get_sensor_data()

        # prox_sens_data = [0, 0, 0, 3434, 0, 0, 0]
        print("in class TEST list :", list(prox_sens_data))
        if prox_sens_data == [0, 0, 0, 0, 0, 0, 0]:
            self.colour(2)
            return True
        else :
            self.colour(1)
            return False
        return

    def obstacle_avoidance(self):
        return

    def motors(self, motor_speed_left, motor_speed_right):
        return {
            "motor.left.target": [motor_speed_left],
            "motor.right.target": [motor_speed_right],
        }

    def colour(self,c):
        leds_top = self.node.v.leds.top
        print("We got some colour man",list(leds_top))
        if c == 1:
            leds_top[0] = 32
            leds_top[1] = 0
            leds_top[2] = 0
            return {"leds.top": [32, 0, 0]}
        elif c == 2:
            leds_top[0] = 0
            leds_top[1] = 32
            leds_top[2] = 0
            return {"leds.top": [0,32,0]}
        elif c == 3:
            leds_top[0] = 0
            leds_top[1] = 0
            leds_top[2] = 32
            return {"leds.top": [0, 0, 32]}

        self.client.aw(node.lock_node())
        self.node.var_to_send
        self.node.flush()


    # async def prog():
    #     with await client.lock() as node:
    #         await node.wait_for_variables({"prox.horizontal"})
    #         while True:
    #             prox_front = node.v.prox.horizontal[2]
    #             speed = -prox_front // 10
    #             node.v.motor.left.target = 100
    #             node.v.motor.right.target = 100
    #             node.flush()
    #             await client.sleep(0.1)

from tdmclient import ClientAsync
from tdmclient.atranspiler import ATranspiler
#
thymio_program_python = r"""
@onevent
def prox():
    global prox_horizontal, motor_left_target, motor_right_target
    prox_front = prox_horizontal[2]
    speed = -prox_front // 10
    motor_left_target = speed
    motor_right_target = speed
"""

avoid_stuff_program = r"""
@onevent
def timer0():
    global prox_ground_delta, prox_horizontal, motor_left_target, motor_right_target, state, obst, obstThrH, obstThrL, obstSpeedGain, speed0, speedGain
    # acquisition from ground sensor for going toward the goal
    diffDelta = prox_ground_delta[1] - prox_ground_delta[0]

    # acquisition from the proximity sensors to detect obstacles
    obst = [prox_horizontal[0], prox_horizontal[4]]

    # tdmclient does not support yet multiple and/or in if statements:
    if state == 0:
        # switch from goal tracking to obst avoidance if obstacle detected
        if (obst[0] > obstThrH):
            state = 1
        elif (obst[1] > obstThrH):
            state = 1
    elif state == 1:
        if obst[0] < obstThrL:
            if obst[1] < obstThrL:
                # switch from obst avoidance to goal tracking if obstacle got unseen
                state = 0
    if state == 0:
        # goal tracking: turn toward the goal
        leds_top = [0, 0, 0]
        motor_left_target = speed0 - speedGain * diffDelta
        motor_right_target = speed0 + speedGain * diffDelta
    else:
        leds_top = [30, 30, 30]
        # obstacle avoidance: accelerate wheel near obstacle
        motor_left_target = speed0 + obstSpeedGain * (obst[0] // 100)
        motor_right_target = speed0 + obstSpeedGain * (obst[1] // 100)

"""

# convert program from Python to Aseba
# thymio_program_aseba = ATranspiler.simple_transpile(thymio_program_python)
#
# with ClientAsync() as client:
#     async def prog():
#         with await client.lock() as node:
#             error = await node.compile(thymio_program_aseba)@onevent
# def prox():
#     global prox_horizontal, motor_left_target, motor_right_target
#     prox_front = prox_horizontal[2]
#     speed = -prox_front // 10
#     motor_left_target = speed
#     motor_right_target = speed
#             error = await node.run()
# client.run_async_program(prog)
