from tdmclient import ClientAsync


# client = ClientAsync()
#
# node = client.aw(client.wait_for_node())
#
class Local_Nav:

    def __init__(self,client,node, motor_speed_left,motor_speed_right):
        self.motor_speed_left = motor_speed_left
        self.motor_speed_right = motor_speed_right
        self.client = client
        self.node = node #self.client.aw(self.client.wait_for_node())
        self.sat_speed = 500
        self.lower_threshold = 2000
        self.middle_threshold = 3000
        self.upper_threshold = 3600

    def get_sensor_data(self):

        self.client.aw(self.node.wait_for_variables({"prox.horizontal"}))
        prox_sens = self.node.v.prox.horizontal
        return  list(prox_sens)

    def analyse_data(self):
        prox_sens_data = self.get_sensor_data()

        if prox_sens_data == [0, 0, 0, 0, 0, 0, 0]:
            self.colour2(2)
            flag = 0
            return flag
        else :
            # print("flag is on")
            self.colour2(1)
            flag = 1
            return flag


    def obstacle_avoidance(self):
        flag = self.analyse_data()
        obstSpeedGain = [6, 4]
        prox_sens_data = self.get_sensor_data()
        extermity_sens = [prox_sens_data[0], prox_sens_data[4]]
        mid_extermity_sens = [prox_sens_data[1], prox_sens_data[3]]
        middle_sensor = prox_sens_data[2]
        gain = [0,0]
        gain_mid = [(mid_extermity_sens[0] + extermity_sens[0]) * obstSpeedGain[1] // 100,
                (mid_extermity_sens[1] + extermity_sens[1]) * obstSpeedGain[1] // 100]
        gain_high = [(mid_extermity_sens[0] + extermity_sens[0]) * obstSpeedGain[0] // 100,
                (mid_extermity_sens[1] + extermity_sens[1]) * obstSpeedGain[0] // 100]
        if flag == 1:
            if extermity_sens[0] or extermity_sens[1] >= self.lower_threshold:
                for i in range(2):
                    if extermity_sens[i] or mid_extermity_sens[i] <= self.middle_threshold:
                        adjust_speed = self.motors(self.motor_speed_left + gain_mid[0],
                                                   self.motor_speed_right + gain_mid[1])
                        self.node.send_set_variables(adjust_speed)
                    elif self.middle_threshold <= extermity_sens[i] or mid_extermity_sens[i] <= self.upper_threshold:
                        adjust_speed = self.motors(self.motor_speed_left + gain_mid[0],
                                                   self.motor_speed_right + gain_mid[1])
                        self.node.send_set_variables(adjust_speed)
                    elif extermity_sens[i] or mid_extermity_sens[i] >= self.upper_threshold:
                        adjust_speed = self.motors(self.motor_speed_left + gain_high[0],
                                                   self.motor_speed_right + gain_high[1])
                        self.node.send_set_variables(adjust_speed)

                    elif extermity_sens[i] and mid_extermity_sens[i] >= self.upper_threshold:
                        if i == 0:
                            adjust_speed = self.motors(self.motor_speed_left + gain_high[0],
                                                       -self.motor_speed_right - gain_high[1])
                        else:
                            adjust_speed = self.motors(-self.motor_speed_left - gain_high[0],
                                                       self.motor_speed_right + gain_high[1])
                        self.node.send_set_variables(adjust_speed)

            if mid_extermity_sens[0] or mid_extermity_sens[1] >= self.lower_threshold:
                for i in range(2):
                    print("hi there")
                    if extermity_sens[i]  <= self.upper_threshold:
                        print("hi there 2", extermity_sens)
                        adjust_speed = self.motors(self.motor_speed_left + gain_mid[0],
                                                   self.motor_speed_right + gain_mid[1])
                        self.node.send_set_variables(adjust_speed)
                    elif extermity_sens[i] >= self.upper_threshold:
                        if i == 0:
                            print("hi there left", extermity_sens)
                            adjust_speed = self.motors(self.motor_speed_left ,
                                                       -self.motor_speed_right - gain_high[1])
                        else:
                            print("hi there right", extermity_sens)
                            adjust_speed = self.motors(-self.motor_speed_left - gain_high[0],
                                                       self.motor_speed_right )
                        self.node.send_set_variables(adjust_speed)
            if self.lower_threshold <= middle_sensor <= self.middle_threshold:
                if mid_extermity_sens[0] <= mid_extermity_sens[1]:
                    adjust_speed = self.motors(-self.motor_speed_left - gain[0],
                                               2*self.motor_speed_right + gain_high[1])
                else:
                    adjust_speed = self.motors(2*self.motor_speed_left + gain[0],
                                                -self.motor_speed_right - gain_high[1])
                self.node.send_set_variables(adjust_speed)
            if middle_sensor >= self.lower_threshold and mid_extermity_sens[0] >= self.lower_threshold:
                    adjust_speed = self.motors(-self.motor_speed_left - gain_high[0],
                                               -2*self.motor_speed_right - 2*gain_high[1])
                    if mid_extermity_sens[0] >= self.upper_threshold :
                        adjust_speed = self.motors(-self.motor_speed_left - gain_high[0],
                                                   -2 * self.motor_speed_right - 2 * gain_high[1])
                    self.node.send_set_variables(adjust_speed)

            if middle_sensor >= self.lower_threshold and mid_extermity_sens[1] >= self.lower_threshold:
                    adjust_speed = self.motors(-2*self.motor_speed_left - 2*gain_high[0],
                                               -self.motor_speed_right - gain_high[1])
                    if mid_extermity_sens[0] >= self.upper_threshold :
                        adjust_speed = self.motors(-self.motor_speed_left - gain_high[0],
                                                   -2 * self.motor_speed_right - 2 * gain_high[1])
                    self.node.send_set_variables(adjust_speed)


    def motors(self, motor_speed_left, motor_speed_right):
        if motor_speed_left >= 500:
            return {
                "motor.left.target": [self.sat_speed],
                "motor.right.target": [motor_speed_right],
            }
        elif motor_speed_right >= 500:
            return {
                "motor.left.target": [motor_speed_left],
                "motor.right.target": [self.sat_speed],
            }
        elif motor_speed_left <= -500:
            return {
                "motor.left.target": [-self.sat_speed],
                "motor.right.target": [motor_speed_right],
            }
        elif motor_speed_right <= -500:
            return {
                "motor.left.target": [motor_speed_left],
                "motor.right.target": [-self.sat_speed],
            }
        else:
            return {
                "motor.left.target": [motor_speed_left],
                "motor.right.target": [motor_speed_right],
            }


    def colour(self,c):
        leds_top = self.node.v.leds.top
        if c == 1: # make it red
            leds_top[0] = 32
            leds_top[1] = 0
            leds_top[2] = 0
            return {"leds.top": [32, 0, 0]}
        elif c == 2: # make it green
            leds_top[0] = 0
            leds_top[1] = 32
            leds_top[2] = 0
            return {"leds.top": [0,32,0]}
        elif c == 3: # make it blue
            leds_top[0] = 0
            leds_top[1] = 0
            leds_top[2] = 32
            return {"leds.top": [0, 0, 32]}

        self.client.aw(self.node.lock_node())
        self.node.var_to_send
        self.node.flush()

    def colour2(self, c):
        if c == 1:  # make it red
            return {"leds.bottom.left": [32, 0, 0] , "leds.bottom.right": [32, 0, 0]}
        elif c == 2:  # make it green
            return {"leds.bottom.left": [0, 32, 0] , "leds.bottom.right": [0, 32, 0]}
        elif c == 3:  # make it blue

            return {"leds.bottom.left": [0, 0, 32] , "leds.bottom.right": [0, 0, 32]}

        self.client.aw(self.node.lock_node())
        self.node.var_to_send
        self.node.flush()

