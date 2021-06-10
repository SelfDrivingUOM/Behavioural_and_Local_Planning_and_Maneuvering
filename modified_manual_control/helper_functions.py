import glob
import sys
import numpy as np
import random
try:
    sys.path.append("../Behavioural_and_Local_Planning_and_Maneuvering")

except IndexError:
    pass

from os_carla import *

if WINDOWS:
    try:
        sys.path.append(glob.glob('C:/Carla0.99/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])
        sys.path.append("../Behavioural_and_Local_Planning_and_Maneuvering")

    except IndexError:
        pass

elif YASINTHA_WINDOWS:
    try:
        sys.path.append(glob.glob('C:/Users/4Axis/Desktop/Project/Carla/WindowsNoEditor/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])
    except IndexError:
        pass

elif GERSHOM_WINDOWS:
    try:
        sys.path.append(glob.glob('D:/WindowsNoEditor/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])
    except IndexError:
        pass
    
else:
    try:
        sys.path.append(glob.glob('/home/selfdriving/carla-precompiled/CARLA_0.9.9/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg' )[0])
    except IndexError:
        pass


import carla

lt = 60 # lifetime of all drawings
grid_resolution = 5 
x_start = -500
x_end = 500
y_start = -500
y_end = 500
res_main = 5

def draw_axes(world, origin, length, size=0.1):
    # draws the coord frame
    # origin - carla.Location
    # x-axis - red, y-axis - green

    x_end = origin + carla.Location(x=length,y=0,z=0.5)
    y_end = origin + carla.Location(x=0,y=length,z=0)

    world.debug.draw_string(origin, 'O', draw_shadow=False,color=carla.Color(r=255, g=255, b=255), life_time=lt,persistent_lines=True)

    world.debug.draw_arrow(origin, x_end, arrow_size=size, life_time=lt, color=carla.Color(r=255, g=0, b=0)) # x axis
    world.debug.draw_string(x_end, 'x+', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=lt,persistent_lines=True)
    
    world.debug.draw_arrow(origin, y_end, arrow_size=size, life_time=lt, color=carla.Color(r=0, g=255, b=0)) # y axis
    world.debug.draw_string(y_end, 'y+', draw_shadow=False,color=carla.Color(r=0, g=255, b=0), life_time=lt,persistent_lines=True)

def draw_lines_ver(world, res, x_start, x_end, y_start, y_end, line_width=0.3, big_res=5):
    # drawing lines parallel to y axis
    # [x_start,...,x_end] - x values of lines
    assert x_start < 0 and x_end > 0 and y_start < 0 and y_end > 0 and res>0

    x_vals_pos = np.arange(0, x_end, res)
    x_vals_neg = np.arange(0, -x_start, res)*(-1)

    for x_vals_list in [x_vals_pos, x_vals_neg]: 
        for ind,x_val in enumerate(x_vals_list):
            pt1 = carla.Location(x=float(x_val), y=float(y_start), z=float(0.5))
            pt2 = carla.Location(x=float(x_val), y=float(y_end), z=float(0.5))

            if ind!=0 and ind%big_res == 0:
                line_color = carla.Color(r=255, g=0, b=0)
                width = line_width*2
                is_main_line = True
            else:
                line_color = carla.Color(r=0, g=0, b=255)
                width = line_width
                is_main_line = False

            # drawing line
            world.debug.draw_line(pt1, pt2, thickness=width, color=line_color, life_time=lt)
            # drawing line no.
            if is_main_line:
                if x_val > 0:
                    str_out = ("x=%i"% ind)
                else:
                    str_out = ("x=%i"% -ind)
                world.debug.draw_string(carla.Location(x=float(x_val), y=float(0), z=float(0.5)), str_out, draw_shadow=False,color=line_color,
                                                     life_time=lt,persistent_lines=True)

def draw_lines_hor(world, res, x_start, x_end, y_start, y_end, line_width=0.3, big_res=5):
    # drawing lines parallel to x axis
    # [y_start,...,y_end] - y values of lines
    assert x_start < 0 and x_end > 0 and y_start < 0 and y_end > 0 and res>0

    y_vals_pos = np.arange(0, y_end, res)
    y_vals_neg = np.arange(0, -y_start, res)*(-1)

    for y_vals_list in [y_vals_pos, y_vals_neg]: 
        for ind,y_val in enumerate(y_vals_list):
            pt1 = carla.Location(x=float(x_start), y=float(y_val), z=float(0.5))
            pt2 = carla.Location(x=float(x_end), y=float(y_val), z=float(0.5))

            if ind!=0 and ind%big_res == 0:
                line_color = carla.Color(r=255, g=0, b=0)
                width = line_width*2
                is_main_line = True
            else:
                line_color = carla.Color(r=0, g=0, b=255)
                width = line_width
                is_main_line = False

            # drawing line
            world.debug.draw_line(pt1, pt2, thickness=width, color=line_color, life_time=lt)
            # drawing line no.
            if is_main_line:
                if y_val > 0:
                    str_out = ("y=%i"% ind)
                else:
                    str_out = ("y=%i"% -ind)
                world.debug.draw_string(carla.Location(x=float(0), y=float(y_val), z=float(0.5)), str_out, draw_shadow=False,color=line_color,
                                                     life_time=lt,persistent_lines=True)


def custom_spawn_actor(world, actor_list, actor_bp, vertical_line, horizontal_line, actor_yaw, res, offsets=(0.0, 0.0)):
    assert offsets[0] <= res/2 and offsets[1] <= res/2
    # spawning a actor in the middle of a cell
    # requires a CARLA transform
    # offsets - x/y offsets from the center of the cell
    actor_x = (vertical_line-1)*res +res/2 + offsets[0]
    actor_y = (horizontal_line-1)*res +res/2 + offsets[1]
    actor_loc = carla.Location(x=float(actor_x), y=float(actor_y), z=0.5)
    actor_rot = carla.Rotation(roll=0, pitch=0, yaw=actor_yaw)
    actor_transform = carla.Transform(actor_loc, actor_rot)
    
    # spawning
    actor = world.try_spawn_actor(actor_bp, actor_transform)

    return actor

