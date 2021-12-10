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
        self.lower_threshold = 2500
        self.middle_threshold = 3200
        self.upper_threshold = 40000

    def get_sensor_data(self):

        self.client.aw(self.node.wait_for_variables({"prox.horizontal"}))
        prox_sens = self.node.v.prox.horizontal
        return  list(prox_sens)

    def analyse_data(self):
        prox_sens_data = self.get_sensor_data()

        if all(prox_sens_data) <= self.lower_threshold:
            self.colour2(2)
            flag = 0
            return flag
        else :

            self.colour2(1)
            flag = 1
            return flag


    def obstacle_avoidance(self):

        flag = self.analyse_data()
        prox_sens_data = self.get_sensor_data()
        left_sens = prox_sens_data[0:2]
        middle_sens = prox_sens_data[2]
        right_sens = prox_sens_data[3:5]
        back_sens = prox_sens_data[5:7]
        obstSpeedGain = [6, 4, -2]    # /100
        if flag == 1:
            if left_sens[0] or left_sens[1] != 0:
                gain = 0
                for i in range(2):
                    gain += left_sens[i] * obstSpeedGain[i] // 100
                print("gain is ", gain)
                if self.motor_speed_left + gain < self.sat_speed:
                    adjust_speed = self.motors(self.motor_speed_left + gain, self.motor_speed_right - gain)
                else:
                    adjust_speed = self.motors(self.sat_speed, self.motor_speed_right)

                self.node.send_set_variables(adjust_speed)
            elif right_sens[0] or right_sens[1] != 0:
                gain = 0
                for i in range(2):
                    gain += right_sens[i] * obstSpeedGain[i] // 100
                if self.motor_speed_right + gain < self.sat_speed:
                    adjust_speed = self.motors( self.motor_speed_left - gain  , self.motor_speed_right + gain)
                else:
                    adjust_speed = self.motors(self.motor_speed_left, self.sat_speed)

                self.node.send_set_variables(adjust_speed)

            elif middle_sens < 3000:
                print("middle sens is : ", middle_sens)
                gain = 0
                if left_sens[1] != 0:
                    gain = left_sens[1] * obstSpeedGain[1] // 100
                    adjust_speed = self.motors(self.motor_speed_left + 2*gain, self.motor_speed_right)
                    self.node.send_set_variables(adjust_speed)
                elif right_sens[0] != 0:
                    gain = right_sens[1] * obstSpeedGain[0] // 100
                    adjust_speed = self.motors(self.motor_speed_left, self.motor_speed_right + 2*gain)
                    self.node.send_set_variables(adjust_speed)

            elif middle_sens > 3000 :
                adjust_speed = self.motors(-self.sat_speed, -self.sat_speed)
                self.node.send_set_variables(adjust_speed)

            elif middle_sens > 4000 and left_sens[1] < right_sens[0]:
                    gain = middle_sens * obstSpeedGain[2] // 100
                    print("gain sens is : ", gain)
                    adjust_speed = self.motors(-self.motor_speed_left + gain, -self.motor_speed_right + 5*gain)
                    self.node.send_set_variables(adjust_speed)
            elif middle_sens > 4000 and left_sens[1] > right_sens[0]:
                    gain = middle_sens * obstSpeedGain[2] // 100
                    adjust_speed = self.motors(-self.motor_speed_left + 5*gain, -self.motor_speed_right + gain)
                    self.node.send_set_variables(adjust_speed)

            elif back_sens != [0,0]:
                if back_sens[0] != 0:
                    gain = 100
                    adjust_speed = self.motors(self.motor_speed_left + gain, self.motor_speed_right )
                    self.node.send_set_variables(adjust_speed)
                elif back_sens[1] != 0:
                    gain = 100
                    adjust_speed = self.motors(self.motor_speed_left, self.motor_speed_right + gain)
                    self.node.send_set_variables(adjust_speed)
            # elif prox_sens_data[1:6] != [0,0,0,0,0]:
            #         gain = 100
            #         adjust_speed = self.motors(-self.motor_speed_left - gain, - self.motor_speed_right - gain)
            #         self.node.send_set_variables(adjust_speed)

        else :
            pass

            # self.node.send_set_variables(adjust_speed)

    def obstacle_avoidance2(self)
        flag = self.analyse_data()
        obstSpeedGain = [6, 4, 2, -2]
        prox_sens_data = self.get_sensor_data()
        extermity_sens = prox_sens_data[0,4]
        mid_extermity_sens = prox_sens_data[1,3]
        middle_sensor = prox_sens_data[2]
        gain = [0,0]
        if flag == 1:
            for idx, val in enumerate(extermity_sens):
                if val <= self.middle_threshold:
                    gain[idx] = (mid_extermity_sens[idx] + extermity_sens[idx]) * obstSpeedGain[2] // 100
                    adjust_speed = self.motors(self.motor_speed_left + gain[0], self.motor_speed_right + gain[1])
                    self.node.send_set_variables(adjust_speed)
                elif self.middle_threshold <= val <= self.upper_threshold:
                    gain[idx] = (mid_extermity_sens[idx] + extermity_sens[idx]) * obstSpeedGain[1] // 100
                    adjust_speed = self.motors(self.motor_speed_left + gain[0], self.motor_speed_right + gain[1])
                    self.node.send_set_variables(adjust_speed)
                elif val >= self.upper_threshold:
                    gain[idx] = (mid_extermity_sens[idx] + extermity_sens[idx]) * obstSpeedGain[0] // 100
                    adjust_speed = self.motors(self.motor_speed_left + gain[0], self.motor_speed_right + gain[1])
                    self.node.send_set_variables(adjust_speed)
            if middle_sensor <= self.upper_threshold:
                adjust_speed = self.motors(2*self.motor_speed_left + gain[0], self.motor_speed_right + gain[1])
                self.node.send_set_variables(adjust_speed)


    def motors(self, motor_speed_left, motor_speed_right):
        if motor_speed_left and motor_speed_right <= 500:
            return {
                "motor.left.target": [motor_speed_left],
                "motor.right.target": [motor_speed_right],
            }
        elif motor_speed_left >= 500:
            return {
                "motor.left.target": [self.sat_speed],
                "motor.right.target": [motor_speed_right],
            }
        elif motor_speed_right >= 500:
            return {
                "motor.left.target": [motor_speed_left],
                "motor.right.target": [self.sat_speed],
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
        leds_top = self.node.v.leds.bottom
        if c == 1:  # make it red
            leds_top[0] = 32
            leds_top[1] = 0
            leds_top[2] = 0
            return {"leds.top": [32, 0, 0]}
        elif c == 2:  # make it green
            leds_top[0] = 0
            leds_top[1] = 32
            leds_top[2] = 0
            return {"leds.top": [0, 32, 0]}
        elif c == 3:  # make it blue
            leds_top[0] = 0
            leds_top[1] = 0
            leds_top[2] = 32
            return {"leds.top": [0, 0, 32]}

        self.client.aw(self.node.lock_node())
        self.node.var_to_send
        self.node.flush()

