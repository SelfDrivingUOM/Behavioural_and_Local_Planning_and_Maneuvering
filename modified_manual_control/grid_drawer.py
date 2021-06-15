# Drawing a 2D grid in CARLA world for actor spawning

import sys
import glob
import os
import numpy as np
import helper_functions as hf

# my_parser = argparse.ArgumentParser("Spawning a pedestrian in the CARLA environment."+\
#     "By default, spawn point is the center of the specified cell. Optional offsets can be given")
# my_parser.add_argument('--xcell','-x', action='store', type=int, required=True, help="desired x cell no.")
# my_parser.add_argument('--ycell','-y', action='store', type=int, required=True, help="desired y cell no.")
# my_parser.add_argument('--psi','-p', action='store', type=float, required=False, default=0,
#                         help="optional yaw angle, default 0")
# my_parser.add_argument('--offsets','-o', action='store', type=float, nargs=2, required=False, default=[0,0],
#                         help="optional offsets along x and y axes, default [0, 0]")


# args=my_parser.parse_args()
# pedestrian_id = "return the walker's ID"
try:
    sys.path.append("../Behavioural_and_Local_Planning_and_Maneuvering")

except IndexError:
    pass

from os_carla import *

if WINDOWS:
    try:
        sys.path.append(glob.glob('C:/Carla0.99/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])

    except IndexError:
        pass

elif YASINTHA_WINDOWS:
    try:
        sys.path.append(glob.glob('C:/Users/4Axis/Desktop/Project/Carla/WindowsNoEditor/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])
    except IndexError:
        pass

else:
    try:
        sys.path.append(glob.glob('/home/selfdriving/carla-precompiled/CARLA_0.9.9/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg' )[0])
    except IndexError:
        pass


import carla

actor_list = []
client = carla.Client('127.0.0.1', 2000)
client.set_timeout(10.0)

try:
    sync = False
    synchronous_master = False
    world = client.get_world()

    if sync:
        settings = world.get_settings()
        if not settings.synchronous_mode:
            synchronous_master = True
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)
        else:
            synchronous_master = False
    
    for i in range(10):
        if sync and synchronous_master:
            world.tick()
        else:
            world.wait_for_tick()
    

    hf.draw_axes(world, carla.Location(x=0,y=0,z=0.2), 4)
    hf.draw_lines_ver(world, hf.grid_resolution, hf.x_start, hf.x_end, hf.y_start, hf.y_end, line_width=0.3, big_res=hf.res_main)
    hf.draw_lines_hor(world, hf.grid_resolution, hf.x_start, hf.x_end, hf.y_start, hf.y_end, line_width=0.3, big_res=hf.res_main)

    print("done printing grid")

    # while 1:
    #     pass

finally:
    print("Bye!")

