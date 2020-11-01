#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" Module with auxiliary functions. """

import math

import numpy as np


import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('/home/selfdriving/carla-precompiled/CARLA_0.9.9/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg' )[0])
except IndexError:
    pass

try:
    sys.path.append('/home/selfdriving/yasintha/Path_planner_6/')

except IndexError:
    pass


import carla


def draw_waypoints(world, waypoints, z=0.5):
    """
    Draw a list of waypoints at a certain height given in z.

    :param world: carla.world object
    :param waypoints: list or iterable container with the waypoints to draw
    :param z: height in meters
    :return:
    """
    for w in waypoints:
        t = w.transform
        begin = t.location + carla.Location(z=z)
        angle = math.radians(t.rotation.yaw)
        end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
        world.debug.draw_arrow(begin, end, arrow_size=0.3, life_time=1.0)


def get_speed(vehicle):
    """
    Compute speed of a vehicle in Kmh
    :param vehicle: the vehicle for which speed is calculated
    :return: speed as a float in Kmh
    """
    vel = vehicle.get_velocity()
    return math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)


def is_within_distance_ahead(target_transform, current_transform, max_distance):
    """
    Check if a target object is within a certain distance in front of a reference object.

    :param target_transform: location of the target object
    :param current_transform: location of the reference object
    :param orientation: orientation of the reference object
    :param max_distance: maximum allowed distance
    :return: True if target object is within max_distance ahead of the reference object
    """
    target_vector = np.array([target_transform.location.x - current_transform.location.x, target_transform.location.y - current_transform.location.y])
    norm_target = np.linalg.norm(target_vector)

    # If the vector is too short, we can simply stop here
    if norm_target < 0.001:
        return True

    if norm_target > max_distance:
        return False

    fwd = current_transform.get_forward_vector()
    forward_vector = np.array([fwd.x, fwd.y])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return d_angle < 90.0


def compute_magnitude_angle(target_location, current_location, orientation):
    """
    Compute relative angle and distance between a target_location and a current_location

    :param target_location: location of the target object
    :param current_location: location of the reference object
    :param orientation: orientation of the reference object
    :return: a tuple composed by the distance to the object and the angle between both objects
    """
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return (norm_target, d_angle)


def distance_vehicle(waypoint, vehicle_transform):
    loc = vehicle_transform.location
    dx = waypoint.transform.location.x - loc.x
    dy = waypoint.transform.location.y - loc.y

    return math.sqrt(dx * dx + dy * dy)


def vector(location_1, location_2):
    """
    Returns the unit vector from location_1 to location_2
    location_1, location_2:   carla.Location objects
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps

    return [x / norm, y / norm, z / norm]

def debug_print(paths,world,best_index):
	for path in paths:
		le=len(path[0])
		if best_index==None:
			best_index=int(le/2)
		if best_index>=paths.shape[0]:
			best_index=paths.shape[0]-1
		for i in range(le):
            
			if np.all(path==paths[best_index]):
				x=path[0][i]
				y=path[1][i]
				t=path[2][i]

				loc=carla.Location(x=x , y=y,z=0)
				#print(loc)
				world.debug.draw_string(loc, 'X', draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=0.1,persistent_lines=True)
			else:
				x=path[0][i]
				y=path[1][i]
				t=path[2][i]

				loc=carla.Location(x=x , y=y,z=0)
				#print(loc)
				world.debug.draw_string(loc, 'X', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=0.1,persistent_lines=True)

def draw_bound_box(obstacle_actors,world):
    for vehi in obstacle_actors:
        transform = vehi.get_transform()
        bounding_box = vehi.bounding_box
        bounding_box.location += transform.location
        #world.debug.draw_box(bounding_box, transform.rotation,life_time=-1.0000, persistent_lines=True)
        world.debug.draw_box(bounding_box,transform.rotation,0.05, carla.Color(255,0,0,0),0.001)