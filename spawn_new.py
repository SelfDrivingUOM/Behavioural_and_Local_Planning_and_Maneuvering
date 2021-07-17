# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import glob
import os
import sys
import time
import random
import math
import numpy as np
from gekko import GEKKO
import matplotlib.pyplot as plt
from os_carla import WINDOWS,YASINTHA_WINDOWS,GERSHOM_WINDOWS


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================
if WINDOWS:
    try:
        sys.path.append(glob.glob('C:/Carla0.99/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])
    except IndexError:
        pass

    try:
        sys.path.append('C:/Carla0.99/PythonAPI/carla/')

    except IndexError:
        pass

    try:
        sys.path.append('C:/Carla0.99/PythonAPI/carla/agents/navigation')

    except IndexError:
        pass

elif GERSHOM_WINDOWS:
    try:
        sys.path.append(glob.glob('D:/WindowsNoEditor/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])
    except IndexError:
        pass
    
elif YASINTHA_WINDOWS:
    try:
        sys.path.append(glob.glob('C:/Users/4Axis/Desktop/Project/Carla/WindowsNoEditor/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])
    except IndexError:
        pass

    try:
            sys.path.append('C:/Users/4Axis/Desktop/Project/Carla/WindowsNoEditor/PythonAPI/carla/')

    except IndexError:
        pass

    try:
        sys.path.append('C:/Users/4Axis/Desktop/Project/Carla/WindowsNoEditor/PythonAPI/carla/agents/navigation')

    except IndexError:
        pass

elif SAUMYA_UBUNTU:
    try:
        sys.path.append(glob.glob('/home/pq-saumya/Documents/CARLA_0.9.9/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg' )[0])
    except IndexError:
        pass

    try:
        sys.path.append('/home/pq-saumya/Documents/CARLA_0.9.9/PythonAPI/carla/')

    except IndexError:
        pass

    try:
        sys.path.append('/home/pq-saumya/Documents/CARLA_0.9.9/PythonAPI/carla/agents/navigation')

    except IndexError:
        pass



else:
    try:
        sys.path.append(glob.glob('/home/selfdriving/carla-precompiled/CARLA_0.9.9/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg' )[0])
    except IndexError:
        pass

    try:
        sys.path.append('/home/selfdriving/carla-precompiled/CARLA_0.9.9/PythonAPI/carla/')

    except IndexError:
        pass

    try:
        sys.path.append('/home/selfdriving/carla-precompiled/CARLA_0.9.9/PythonAPI/carla/agents/navigation')

    except IndexError:
        pass


# ==============================================================================
# -- file imports --------------------------------------------------------------
# ==============================================================================

import carla
from global_route_planner import GlobalRoutePlanner
from global_route_planner_dao import GlobalRoutePlannerDAO
from controller import VehiclePIDController
from basic_agent import BasicAgent
from carla import VehicleControl


# ==============================================================================
# -- Global variables   --------------------------------------------------------
# ==============================================================================

actor_list=[]

# ==============================================================================
# -- Local functions   ---------------------------------------------------------
# ==============================================================================

def get_transform(vehicle_location, angle, d=6.4):
    a = math.radians(angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
    return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))


def get_current_pose(location):
    x = location.location.x
    y = location.location.y
    yaw = np.deg2rad(location.rotation.yaw)

    return (x,y,yaw)

def send_control_command(vehicle, throttle, steer, brake, 
                         hand_brake=False, reverse=False,manual_gear_shift = False):

    control = VehicleControl()
    # Clamp all values within their limits
    steer = np.fmax(np.fmin(steer, 1.0), -1.0)
    throttle = np.fmax(np.fmin(throttle, 1.0), 0)
    brake = np.fmax(np.fmin(brake, 1.0), 0)

    #print(brake,steer,throttle)

    vehicle.apply_control(carla.VehicleControl(brake = brake , steer = steer,throttle = throttle))


# ==============================================================================
# -- Main function  ------------------------------------------------------------
# ==============================================================================

def spawn_new(world,vehicles):

	try:

		blueprint_library = world.get_blueprint_library()
		vehicle_bp=blueprint_library.filter("model3")[0]
		wmap = world.get_map()
		start_point=wmap.get_spawn_points()[50]
		end_point = wmap.get_spawn_points()[80]
		end = [end_point.location.x,end_point.location.y,end_point.location.z]
		vehicle = world.spawn_actor(vehicle_bp, start_point)


		
		Agent=BasicAgent(vehicle,20)
		Agent.set_destination(end)

		debug=world.debug

		all_actor_list = world.get_actors()
		vehicle_list = all_actor_list.filter("*vehicle*")
		
		while(True):
			cmd=Agent.run_step(True)
			send_control_command(vehicle,cmd.throttle,cmd.steer,cmd.brake, hand_brake=False, reverse=False,manual_gear_shift = False)