#%%

from main_digital_world import *

#%%

stepHandler = RobotSteps()
device_count, imgs = aruco.list_cameras()
camera_index = device_count-1
port_list = stepHandler.arduino_list_ports()
stepHandler.set_window(0.20, 0.20, 0.10, 0.05)
stepHandler.connect(camera_index, port_list[0].device)

#%%

for key, step in stepHandler.ordered_steps:
    #%%
    print('key', key)
    if key==5:
        break
    print(step['desc'])
    print(step['func']())
#%%
start_pos_s5 = stepHandler.rob.getl()
#%%
stepHandler.rob.movel(start_pos_s5, stepHandler.a, stepHandler.v)
print('done')
#%%
stepHandler.step5()
#%%
stepHandler.a = stepHandler.a*3
stepHandler.v = stepHandler.v*3
#%%
step
for key, step in stepHandler.ordered_steps:
    #%%
    if key > 5:
        if key==11:
            break
        print('key', key)
        print(step['desc'])
        print(step['func']())
#%%
start_pos_s11 = stepHandler.rob.getl()
#%%
stepHandler.rob.movel(start_pos_s11, stepHandler.a, stepHandler.v)
print('done')
#%%

stepHandler.step11()


#%%
stepHandler.world_handler.setl(stepHandler.rob.getl())
stepHandler.world_handler.plot()
#%%

stepHandler.world_handler.world.align_tcp_to_robot()
stepHandler.world_handler.move_tcp_to_plane(-0.50)
stepHandler.world_handler.plot()

# self.movel( world.getl(), acc, vel )
#%%
# world.move_tcp( (-safe_move, 0, 0) )
# world.plot()
# world.world.rotate_z_tcp( -0.75 * sympy.pi )
# world.plot()
# self.movel( world.getl(), acc, vel )
