# ## ------------------ Module for Obstale Avoidance for Thymio Robot - Basics of Mobile Robotics --------------------##

from tdmclient import ClientAsync
import Local_Nav as LN

client = ClientAsync()
node = client.aw(client.wait_for_node())

# set Cruise speed here
cruisin = 100;

motor_speed_left = cruisin
motor_speed_right = cruisin

# Define local_nav object with class attributes
local_nav = LN.Local_Nav(client, node, motor_speed_left, motor_speed_right)


client.aw(node.lock_node())
cruise_speed = local_nav.motors(100,100)
stop = local_nav.motors(0,0)
print("My man, our cruise speed is: ", cruise_speed)

node.flush()

counter = 1
while counter < 1000:

    node = client.aw(client.wait_for_node())
    data = local_nav.get_sensor_data()
    flag = local_nav.analyse_data()

    # client.aw(node.lock_node())
    node.send_set_variables(cruise_speed)
    if flag == 1:
        task = local_nav.obstacle_avoidance()
    else:
        pass

    node.flush()
    counter += 1
    print(counter)
node.send_set_variables(stop)
# #
# async def prog():
#     node = await client.wait_for_node()
#     await node.lock()
#     await node.set_variables(cruise_speed)
#     await client.sleep(2)
#     await node.set_variables(stop)
#     await node.unlock()
#
# client.run_async_program(prog)

# obst_avoid_task = local_nav.obstacle_avoidance()
