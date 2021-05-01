# example CLI to spawn a pedestrian in CARLA env
# returns id of the spawned pedestrian
#https://stackoverflow.com/questions/24072790/detect-key-press-in-python
#https://realpython.com/command-line-interfaces-python-argparse/

import glob
import os
import numpy as np
import matplotlib.pyplot as plt
from helper_functions import *
import data_reader
import argparse
from config import carla_path

my_parser = argparse.ArgumentParser("Spawning a pedestrian in the CARLA environment."+\
    "By default, spawn point is the center of the specified cell. Optional offsets can be given")
my_parser.add_argument('--xcell','-x', action='store', type=int, required=True, help="desired x cell no.")
my_parser.add_argument('--ycell','-y', action='store', type=int, required=True, help="desired y cell no.")
my_parser.add_argument('--psi','-p', action='store', type=float, required=False, default=0,
                        help="optional yaw angle, default 0")
my_parser.add_argument('--offsets','-o', action='store', type=float, nargs=2, required=False, default=[0,0],
                        help="optional offsets along x and y axes, default [0, 0]")


args=my_parser.parse_args()
pedestrian_id = "return the walker's ID"

try:
    sys.path.append(glob.glob(carla_path)[0])
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
    

    res=5

    walker_blueprints = world.get_blueprint_library().filter("walker")

    walker_bp = random.choice(walker_blueprints)
    vertical_line = args.xcell  # x cell number
    horizontal_line = args.ycell # y cell number
    walker_yaw = args.psi
    walker_offset = (args.offsets[0], args.offsets[1])
    custom_spawn_pedestrian(world, actor_list, walker_bp, vertical_line, horizontal_line, walker_yaw, res, offsets=walker_offset)

    print("done spawning")

    # while 1:
    #     pass

finally:
    print("Bye!")

    # if sync and synchronous_master:
    #     settings = world.get_settings()
    #     settings.synchronous_mode = False
    #     settings.fixed_delta_seconds = None
    #     world.apply_settings(settings)

    # print('\ndestroying %d spawned walkers' % len(actor_list))
    # client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
