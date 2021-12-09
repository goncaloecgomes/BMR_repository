# ## ------------------ Module for Obstale Avoidance for Thymio Robot - Basics of Mobile Robotics --------------------##

from tdmclient import ClientAsync
import Local_Nav as LN

client = ClientAsync()
node = client.aw(client.wait_for_node())

# set Cruise speed here
cruisin = 200;

motor_speed_left = cruisin
motor_speed_right = cruisin

# Define local_nav object with class attributes
local_nav = LN.Local_Nav(client, node, motor_speed_left, motor_speed_right)


# Store motor speeds in right format to update nodes
client.aw(node.lock_node())
cruise_speed = local_nav.motors(motor_speed_left,motor_speed_right)
stop = local_nav.motors(0,0)

# Test Local Navigation Class
counter = 1
while counter < 300:

    node = client.aw(client.wait_for_node()) #Update states and sensor values at each iteration
    flag = local_nav.analyse_data() # store a flag if obstacle detected (red LEDS)
    node.send_set_variables(cruise_speed)
    if flag == 1:
        task = local_nav.obstacle_avoidance() #trigger task depending on if flag detected
    else:
        pass

    node.flush()
    counter += 1
    print(counter)
colour_switch = local_nav.colour(3)
node.send_set_variables(stop)
node.flush()
#