# ## ------------------ Module for Obstale Avoidance for Thymio Robot - Basics of Mobile Robotics --------------------##

from tdmclient import ClientAsync
import Local_Nav as LN

client = ClientAsync()
node = client.aw(client.wait_for_node())

local_nav = LN.Local_Nav(client,node)#client,node)
data = local_nav.get_sensor_data()
print("this is my data : ", data)
data_analysis = local_nav.analyse_data()
print("analysed data : ", data_analysis)
# #

cruise_speed = local_nav.motors(100,100)
stop = local_nav.motors(0,0)
print("My man, our cruise speed is: ", cruise_speed)
colour_switch = local_nav.colour(3)
print(colour_switch)
var_dic = cruise_speed
async def prog():
    node = await client.wait_for_node()
    await node.lock()
    await node.set_variables(cruise_speed)
    await client.sleep(2)
    await node.set_variables(stop)
    await node.unlock()

client.run_async_program(prog)


client.aw(node.lock_node())
# node.var_to_send
node.flush()