#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" Module with auxiliary functions. """

import math

import numpy as np
import hull
import glob
import os
import sys
import time
from os_carla import WINDOWS
from os_carla import YASINTHA_WINDOWS,GERSHOM_WINDOWS

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
    try:
        sys.path.append('/home/selfdriving/BP_with_git/Behavioural_and_Local_Planning_and_Maneuvering')
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

    
    return d_angle < 10.0


def compute_magnitude_angle(target_location, current_location, orientation):
    """
    Compute relative angle and distance between a target_location and a current_location

    :param target_location: location of the target object
    :param current_location: location of the reference object
    :param orientation: orientation of the reference object
    :return: a tuple composed by the distance to the object and the angle between both objects
    """

    # print(orientation)
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    forward_vector = np.array([math.cos(orientation), math.sin(orientation)])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    dir_ = np.cross(forward_vector, target_vector) 
    if(dir_>0):
        dir_ = +1
    else:
        dir_ = -1 
    return (norm_target, dir_*d_angle)


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

def debug_print(paths,world,best_index,life = 0.1):
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
				world.debug.draw_string(loc, '*', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=life,persistent_lines=True)
			else:
				x=path[0][i]
				y=path[1][i]
				t=path[2][i]

				loc=carla.Location(x=x , y=y,z=0)
				#print(loc)
				world.debug.draw_string(loc, '*', draw_shadow=False,color=carla.Color(r=0, g=255, b=255), life_time=life,persistent_lines=True)

def draw_bound_box(obstacle_actors,world,r,g,b):
    for vehi in obstacle_actors:
        transform = vehi.get_transform()
        bounding_box = vehi.bounding_box
        bounding_box.location += transform.location
        #world.debug.draw_box(bounding_box, transform.rotation,life_time=-1.0000, persistent_lines=True)
        world.debug.draw_box(bounding_box,transform.rotation,0.1, carla.Color(r,g,b,0),0.001)

def draw_bound_box_actor(obstacle_actor,world, r, g, b):
    if (obstacle_actor!=None):
        transform = obstacle_actor.get_transform()
        bounding_box = obstacle_actor.bounding_box
        bounding_box.location += transform.location
        world.debug.draw_box(bounding_box,transform.rotation,0.1,carla.Color(r=r, g=g,b=b),0.03)

def draw_bound_box_actor_emerg(obstacle_actor,world, r, g, b):
    if (obstacle_actor!=None):
        transform = obstacle_actor.get_transform()
        bounding_box = obstacle_actor.bounding_box
        bounding_box.location += transform.location
        world.debug.draw_box(bounding_box,transform.rotation,0.1,carla.Color(r=r, g=g,b=b),0.03)

def draw_emergency_box(obstacle_actor,world, r, g, b,emerg_loc,emerg_yaw):
    if (obstacle_actor!=None):
        transform = obstacle_actor.get_transform()
        bounding_box = obstacle_actor.bounding_box
        bounding_box.location += emerg_loc
        rot= carla.Rotation(pitch=transform.rotation.pitch,yaw = emerg_yaw, roll= transform.rotation.roll )
        world.debug.draw_box(bounding_box,rot,0.1,carla.Color(r=r, g=g,b=b),0.01)

def spawn_pts_print(world_map,world):

    spawn_pts=world_map.get_spawn_points()
    for i in range (len(spawn_pts)):
        p = world_map.get_spawn_points()[i]
        world.debug.draw_string(p.location, str(i), draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)
            


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================

def get_line(point_list):
    # print(point_list)
    n_ = point_list.shape[0]
    line_info = np.zeros((n_-1,2))
    

    grad_vals = point_list[1:,:] - point_list[:-1,:]
    # print(grad_vals)
    crit = (grad_vals[:,0] ==0)
    crit_ = np.logical_not(crit)
    grad_vals_ = grad_vals[crit_][:,1] /grad_vals[crit_][:,0]
    c_vals = point_list[:-1,1][crit_] - grad_vals_*point_list[:-1,0][crit_]
    
    # print(grad_vals_,c_vals)
    grad_vals_ = grad_vals_.reshape((grad_vals_.shape[0],1))
    c_vals = c_vals.reshape((grad_vals_.shape[0],1))
    line_info[crit_] = np.hstack((grad_vals_,c_vals))

    if(np.any(crit)):
        line_info[crit] = np.array([None,point_list[:-1][crit][0,0]])
    

    # print(line_info)
    # for i in range(len(point_list)-1):

    #     if((point_list[i+1][0]-point_list[i][0]) != 0 ):
    #         m = (point_list[i+1][1]-point_list[i][1])/(point_list[i+1][0]-point_list[i][0])
    #         c = point_list[i][1] - m*point_list[i][0] 
    #     else:
    #         m = None
    #         c = point_list[i][0]

    #     line_info.append((m,c))
    temp_ = point_list[0] - point_list[-1]
    if(temp_[0] ==0):
        line_info = np.append(line_info,[[None,temp_[0]]],axis = 0)
    else:
        m_ = temp_[1]/temp_[0]

        line_info = np.append(line_info,[[m_,point_list[0][1]-m_*point_list[0][0]]],axis = 0)

    # line_info  
    # print(line_info)
    return line_info
"""
@breif: solve_lines
This function solves the lines obtained from the convex hull 
of the intersection and returns the points.
@param: lines
lines: this is a numpy array with (x,y) coordinates of the convex hull
@return: intersection
intersection: this is a numpu array with the (x,y) coordinates of the 
              intersection points of the lines o0f the convex hull

"""
def solve_lines(lines):
    intersections = np.empty((0,2))

    for i in range(-2,lines.shape[0]-2,1):

        if(np.isnan(lines[i+2][0])): 
            x_ = lines[i+2][1]
            y_ = lines[i][0]*x_+ lines[i][1]
        
        elif(np.isnan(lines[i][0])):
            x_ = lines[i][1]
            y_ = lines[i+2][0]*x_+ lines[i+2][1]
            
        else:
            x_ = (lines[i+2][1]-lines[i][1])/(lines[i][0]-lines[i+2][0])
            y_ = lines[i][0]*x_ + lines[i][1]
        intersections = np.append(intersections,[[x_,y_]],axis =0)
    # print(intersections)
    return intersections

def draw_hex(world,line_info):

    for i in range(line_info.shape[0]):

        if(np.isnan(line_info[i][0])):
            point_1 = [line_info[i][1],-200]
            point_2 = [line_info[i][1],200]
        else:
            point_1 = [-200,-200*line_info[i][0]+line_info[i][1]]
            point_2 = [200,200*line_info[i][0]+line_info[i][1]]
        world.world.debug.draw_line(carla.Location(x= point_1[0],y = point_1[1],z = 0),carla.Location(x= point_2[0],y = point_2[1],z = 1.843), thickness=0.2, color=carla.Color(r=255, g=0, b=0), life_time=-1.)

def get_box(world_map,intersections):

    count=0

    loc  = carla.Location(x = intersections[0][0],y = intersections[0][1],z = 0)
    wayp_= world_map.get_waypoint(loc,project_to_road = True,lane_type = (carla.LaneType.Driving|carla.LaneType.Sidewalk))
    
    var = wayp_.lane_type
    # print(var,type(var))
    # point_info.append(var)

    if(var == carla.LaneType.Sidewalk):
        count+=1


    crit = np.arange(0,8,1)%2
    if(count%2 ==0):

        return(intersections[crit==1])

    else:
        return(intersections[crit==0])
        # print(var)

def within_polygon(points,location):
    within = False
    sign = None
    for i in range(points.shape[0]-1):
        cross = np.cross(points[i+1]-points[i],location-points[i])
        if sign == None:
            if(cross>0):
                sign = 1
            else:
                sign = -1

        else:
            if(sign*cross>0):
                continue
            else:
                break
    else:
        within = True

    return(within)


def print_junction(world,waypoint):

    ######CONVERT THIS TO NUMPY
    L = waypoint.get_junction().get_waypoints(carla.LaneType.Driving)
    # print(len(L))

    temp_list = []
    for i in L:

        # rand_r = random.randint(0,255)
        # rand_g = random.randint(0,255)
        # rand_b = random.randint(0,255)
        for j in i:
            # print(j.lane_id,j.road_id,j.section_id,j.lane_type)
            temp_list.append((j.transform.location.x,j.transform.location.y))
            # world.world.debug.draw_string(j.transform.location,"A", draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)
            
    hull_ = hull.GrahamScan(temp_list)

    # print(len(hull_))
    final_hull = []
    curr_ = None
    for i in range(len(hull_)-1):

        # world.world.debug.draw_line(carla.Location(x= hull_[i][0],y = hull_[i][1],z = 1.843),carla.Location(x= hull_[i+1][0],y = hull_[i+1][1],z = 1.843), thickness=0.2, color=carla.Color(r=255, g=0, b=0), life_time=-1.)
        
        angle = np.degrees(np.arctan2(hull_[i+1][1]-hull_[i][1],hull_[i+1][0]-hull_[i][0]))
        # print(angle)

        if(i == 0):
            final_hull.append(hull_[0])
            final_hull.append(hull_[1])
            curr_ = angle
            # world.world.debug.draw_line(carla.Location(x= hull_[0][0],y = hull_[0][1],z = 1.843),carla.Location(x= hull_[1][0],y = hull_[0][1],z = 1.843), thickness=0.2, color=carla.Color(r=255, g=0, b=0), life_time=-1.)

        else:
            if(abs(angle - curr_)<5):
                curr_ = angle
                
            else:
                if(np.linalg.norm(hull_[i+1]-hull_[i])>0.75):
                    curr_ = angle
                    final_hull.append(hull_[i+1])
                else:
                    continue
                # world.world.debug.draw_line(carla.Location(x= hull_[i][0],y = hull_[i][1],z = 1.843),carla.Location(x= hull_[i+1][0],y = hull_[i+1][1],z = 1.843), thickness=0.2, color=carla.Color(r=255, g=0, b=0), life_time=-1.)
    
    ####only for 4 intersections


    if(len(final_hull) == 9):
        final_hull=final_hull[1:]

    # print(len(final_hull))

    # j = 0
    # for i in final_hull:

    #     world.world.debug.draw_string(carla.Location(x=i[0],y = i[1],z = 1),chr(65+j), draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)
    #     j+=1
    return np.array(final_hull)
    
    # for i in final_hull:

    #     world.world.debug.draw_string(carla.Location(x=i[0],y = i[1],z = 1),"A", draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)
