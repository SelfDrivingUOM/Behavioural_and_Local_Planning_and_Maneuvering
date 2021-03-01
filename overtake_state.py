import numpy as np
from tools.misc import get_speed
from os_carla import WINDOWS
import sys
import glob
import time

if WINDOWS:
    try:
        sys.path.append(glob.glob('C:/Carla0.99/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])

    except IndexError:
        pass

else:
    try:
        sys.path.append(glob.glob('/home/selfdriving/carla-precompiled/CARLA_0.9.9/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg' )[0])
    except IndexError:
        pass

import carla
#### --------------------------------- ####
# Functions used in the OVERTAKE state ####
#### --------------------------------- ####
ovetakeAllowed = [carla.LaneMarkingType.BrokenSolid, carla.LaneMarkingType.BrokenBroken, carla.LaneMarkingType.Broken]
def check_lane_closest(closest_vehicle, ego_vehicle,_map):
    ego_wpt = _map.get_waypoint(ego_vehicle.get_transform().location,project_to_road=True,lane_type=carla.LaneType.Driving)   
    cst_veh_wpt = _map.get_waypoint(closest_vehicle.get_transform().location,project_to_road=True,lane_type=carla.LaneType.Driving)  
    return ego_wpt.lane_id == cst_veh_wpt.lane_id
    
def can_we_overtake(ego_vehicle, closest_vehicle,_map,_world,glb_wpts,desired_speed, environment,speedFactor):
    canOveretake = False
    rear_buffer = 5   # 5
    closest_veh_speed = get_speed(closest_vehicle)
    ##############################################################################
    # Find the distance to closest vehicle using the global wapypoint 
    _, ego_index = get_closest_index(np.array([ego_vehicle.get_transform().location.x, ego_vehicle.get_transform().location.y]),glb_wpts)
    _, veh_index = get_closest_index(np.array([closest_vehicle.get_transform().location.x, closest_vehicle.get_transform().location.y]),glb_wpts)
    # print('index:',ego_index,veh_index)
    to_closest = 0
    for i in range(ego_index,veh_index-1):
        to_closest += np.linalg.norm(glb_wpts[i,:2]-glb_wpts[i+1,:2])
    ##############################################################################

    time_to_pass =  (to_closest + ego_vehicle.bounding_box.extent.y + closest_vehicle.bounding_box.extent.y + 3.5 + 3)/desired_speed
    time_to_global  = (ego_vehicle.bounding_box.extent.y + 15)/(desired_speed-((1-speedFactor)*closest_veh_speed))

    # print("Speed lead",get_speed(closest_vehicle))
    frwd_buffer_ego = (time_to_pass*(desired_speed + closest_veh_speed)) + (time_to_global*(desired_speed + (speedFactor*closest_veh_speed)))
    frwd_buffer = frwd_buffer_ego
    roadRulesOK, rear_buffer_wpts, frwd_buffer_wpts, frwd_buffer_ego_wpts  = road_rules_ok(ego_vehicle, to_closest, rear_buffer, frwd_buffer, frwd_buffer_ego, _map,_world)
    # print("condition2")
    # print(roadRulesOK)
    # print("end of condition2")
    if roadRulesOK and ( not is_overtake_collisions(rear_buffer_wpts, frwd_buffer_wpts, frwd_buffer_ego_wpts,_world, environment, ego_vehicle)):
        canOveretake = True
    # print("Can Overtake",canOveretake)
    return canOveretake, frwd_buffer_wpts, frwd_buffer_ego_wpts

def road_rules_ok(ego_vehicle, to_closest, rear_buffer, frwd_buffer, frwd_buffer_ego, _map, _world):
    is_ok = True    # this indicates that we can perform the overtaking maneuver
    ego_waypoint=_map.get_waypoint(ego_vehicle.get_transform().location,project_to_road=True,lane_type=carla.LaneType.Driving)
    rear_buffer_wpts = None
    frwd_buffer_wpts = None
    frwd_buffer_ego_wpts = None
    
    # Get the waypoint in the left lane (backward) wrt to the ego vehicle and check if  lane change is allowed
    wpt = ego_waypoint.get_left_lane()
    # _world.debug.draw_string(wpt.transform.location, 'X', draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=100,persistent_lines=True)


    if wpt is None:
        is_ok = False
    else:
        if (wpt.lane_id*ego_waypoint.lane_id)<0:
            # print("overtaking  using oppsite direction lane")
            rear_buffer, frwd_buffer = frwd_buffer, rear_buffer
        dist = 0
        rear_buffer_wpts = np.array([[wpt.transform.location.x, wpt.transform.location.y]])
        # _world.debug.draw_string(wpt.transform.location, 'X', draw_shadow=False,color=carla.Color(r=0, g=255, b=0), life_time=5,persistent_lines=True)
        while dist < rear_buffer:
            prev_wpt = wpt.previous(0.2)
            if len(prev_wpt)==1:
                rear_buffer_wpts = np.append(rear_buffer_wpts,np.array([[prev_wpt[0].transform.location.x,prev_wpt[0].transform.location.y]]),axis=0)
                # _world.debug.draw_string(prev_wpt[0].transform.location, 'X', draw_shadow=False,color=carla.Color(r=0, g=255, b=0), life_time=5,persistent_lines=True)
                dist += np.linalg.norm(rear_buffer_wpts[-1] - rear_buffer_wpts[-2])
                wpt = prev_wpt[0]
            else:
                # for wpt_ in prev_wpt:
                # _world.debug.draw_string(prev_wpt[0].transform.location, 'X', draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=60,persistent_lines=True)
                # _world.debug.draw_string(prev_wpt[1].transform.location, 'X', draw_shadow=False,color=carla.Color(r=0, g=255, b=0), life_time=40,persistent_lines=True)
                # while True:
                #     pass
                is_ok = False
                # print("Len not equal 1 rear")
                # print("dist",prev_wpt)
                # print("left lane backward waypoint is none")
                # raise Exception
                break
        # if is_ok:
        #     # print('rear buffer ok')
        # else:
        #     print('rear buffer not ok')
            
    
    # getting the waypoints in the left lane (forward dirn)
    if is_ok:
        dist = 0
        next_wpt = ego_waypoint.get_left_lane()
        frwd_buffer_wpts = np.array([[next_wpt.transform.location.x, next_wpt.transform.location.y]])

        while dist < frwd_buffer:
            next_wpt = next_wpt.next(0.2)          
            if len(next_wpt) == 1 and  (next_wpt[0].lane_change.Both or next_wpt[0].lane_change.Right):
                frwd_buffer_wpts = np.append(frwd_buffer_wpts,np.array([[next_wpt[0].transform.location.x, next_wpt[0].transform.location.y]]),axis=0)
                # _world.debug.draw_string(next_wpt[0].transform.location, 'X', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=500,persistent_lines=True)
                dist += np.linalg.norm(frwd_buffer_wpts[-1] - frwd_buffer_wpts[-2])
                next_wpt = next_wpt[0]
            else:
                is_ok = False
                # print("Len not equal 1 Forward")
                break
        # if is_ok:
        #     print('forwd next buffer ok')
        # else:
        #     print('forwd next buffer not ok')

        
    # getting the waypoints in the ego lane (forward dirn)
    if is_ok:
        dist = 0
        next_wpt = ego_waypoint
        frwd_buffer_ego_wpts = np.array([[next_wpt.transform.location.x, next_wpt.transform.location.y]])

        while dist < frwd_buffer_ego:   # -------------------------------- define a distance
            next_wpt = next_wpt.next(0.2)
            if len(next_wpt) == 1 and  (next_wpt[0].lane_change.Both or next_wpt[0].lane_change.Left) and (next_wpt[0].left_lane_marking.type in ovetakeAllowed):
                frwd_buffer_ego_wpts = np.append(frwd_buffer_ego_wpts,np.array([[next_wpt[0].transform.location.x, next_wpt[0].transform.location.y]]),axis=0)
                # _world.debug.draw_string(next_wpt[0].transform.location, 'X', draw_shadow=False,color=carla.Color(r=255, g=255, b=255), life_time=500,persistent_lines=True)
                dist += np.linalg.norm(frwd_buffer_ego_wpts[-1] - frwd_buffer_ego_wpts[-2])
                next_wpt = next_wpt[0]
            else:
                is_ok = False
                # print("Len not equal 1 Ego")
                break
        # if is_ok:
        #     print('forwd ego buffer ok')
        # else:
        #     print('forwd ego buffer not ok')

    if  not is_ok:
        rear_buffer_wpts = None
        frwd_buffer_wpts = None
        frwd_buffer_ego_wpts = None

    return is_ok, rear_buffer_wpts, frwd_buffer_wpts, frwd_buffer_ego_wpts 

def is_overtake_collisions(rear_buffer_wpts, frwd_buffer_wpts, frwd_buffer_ego_wpts,_world, environment, ego_vehicle):
    is_collision = True
    tresh_dist_sq = (3.7/2)**2  #half the width of a lane

    buffer_wpts = np.concatenate((rear_buffer_wpts,frwd_buffer_wpts,frwd_buffer_ego_wpts))
    # for b in rear_buffer_wpts:#[rear_buffer_wpts[0],rear_buffer_wpts[-1]]:
    #     _world.debug.draw_string(carla.Location(x=b[0],y=b[1],z=1.843), 'X', draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=500,persistent_lines=True)
    # for b in frwd_buffer_wpts:#[frwd_buffer_wpts[0],frwd_buffer_wpts[-1]]:
    #     _world.debug.draw_string(carla.Location(x=b[0],y=b[1],z=1.843), 'X', draw_shadow=False,color=carla.Color(r=0, g=255, b=0), life_time=500,persistent_lines=True)
    # for b in frwd_buffer_ego_wpts:#[frwd_buffer_ego_wpts[0],frwd_buffer_ego_wpts[-1]]:
    #     _world.debug.draw_string(carla.Location(x=b[0],y=b[1],z=1.843), 'X', draw_shadow=False,color=carla.Color(r=255, g=255, b=255), life_time=500,persistent_lines=True)
    
    vehicles, walkers = environment.get_overtake_actors(buffer_wpts, tresh_dist_sq, ego_vehicle)
    # print("vehicles no",len(vehicles))
    # print(vehicles)

    if (len(vehicles+walkers)>1):
        is_collision = True
    else:
        is_collision = False

    # print("is collision",is_collision)
    return is_collision

def get_closest_index(ego_state, _waypoints):
    """
    Gets closest index a given list of waypoints to the vehicle position.
    """

    waypoint_dists = np.sqrt(np.square(_waypoints[:,0] - ego_state[0]) + np.square(_waypoints[:,1] - ego_state[1]))
    closest_len = np.amin(waypoint_dists)
    closest_index = np.where((waypoint_dists==closest_len))[0][0]
    return closest_len, closest_index

# [[carla.Location(x=  17.250000, y= 177.399994, z= 0.000000),carla.Location(x=  15.650000, y= 197.949997, z=0.000000),carla.Location(x=  40.849998, y= 216.349991, z=0.000000)],
#  [carla.Location(x=  19.150000, y= 101.699997, z= 0.000000),carla.Location(x=  18.600000, y=  79.750000, z=0.000000),carla.Location(x=  42.549999, y= 100.199997, z=0.000000),carla.Location(x=  40.250000, y=  78.599998, z=0.000000)],
#  [carla.Location(x=  40.899998, y=  10.500000, z= 0.000000),carla.Location(x=  40.250000, y= -11.099999, z=0.000000),carla.Location(x=  18.600000, y=  -9.950000, z=0.000000),carla.Location(x=  19.150000, y=  12.000000, z=0.000000)],
#  [carla.Location(x=  41.399998, y= -78.099998, z= 0.000000),carla.Location(x=  41.849998, y=-100.199997, z=0.000000),carla.Location(x=  20.250000, y= -99.900002, z=0.000000),carla.Location(x=  21.250000, y= -78.849998, z=0.000000)],
#  [carla.Location(x=  43.899998, y=-158.199997, z= 0.200000),carla.Location(x=  23.327223, y=-154.399994, z=0.100000)],
#  [carla.Location(x=  46.349998, y=-178.500000, z= 0.000000),carla.Location(x=  47.950001, y=-197.349991, z=0.000000),carla.Location(x=  22.750000, y=-215.149994, z=0.000000)],
#  [carla.Location(x= -38.649998, y= 100.049995, z= 0.000000),carla.Location(x= -37.450001, y=  78.000000, z=0.000000),carla.Location(x= -61.250000, y=  76.699997, z=0.000000),carla.Location(x= -61.349998, y= 101.250000, z=0.000000)],
#  [carla.Location(x= -38.549999, y=  11.599999, z= 0.000000),carla.Location(x= -38.599998, y=  -9.650000, z=0.000000),carla.Location(x= -60.699997, y=  -9.500000, z=0.000000),carla.Location(x= -59.599998, y=  12.450000, z=0.000000)],
#  [carla.Location(x= -39.599998, y= -78.099998, z= 0.000000),carla.Location(x= -38.849998, y=-101.299995, z=0.000000),carla.Location(x= -62.799999, y=-101.799995, z=0.000000),carla.Location(x= -61.149998, y= -78.099998, z=0.000000)],
#  [carla.Location(x=-112.299995, y= 159.399994, z= 0.000000),carla.Location(x=-138.500000, y= 137.550003, z=0.000000),carla.Location(x=-138.800003, y= 159.550003, z=0.000000)],
#  [carla.Location(x=-112.299995, y= 101.099998, z= 0.000000),carla.Location(x=-113.399994, y=  78.000000, z=0.000000),carla.Location(x=-139.800003, y=  77.849998, z=0.000000),carla.Location(x=-138.800003, y= 100.049995, z=0.000000)],
#  [carla.Location(x=-113.149994, y=  11.599999, z= 0.000000),carla.Location(x=-113.899994, y=  -9.650000, z=0.000000),carla.Location(x=-139.349991, y= -10.300000, z=0.000000),carla.Location(x=-140.399994, y=  12.450000, z=0.000000)],
#  [carla.Location(x=-113.149994, y= -78.949997, z= 0.000000),carla.Location(x=-112.799995, y=-101.299995, z=0.000000),carla.Location(x=-139.349991, y=-100.849998, z=0.000000),carla.Location(x=-140.399994, y= -78.099998, z=0.000000)],
#  [carla.Location(x=-112.966469, y=-128.849045, z=-0.000005),carla.Location(x=-112.765907, y=-150.949997, z=0.000000),carla.Location(x=-143.038879, y=-150.019562, z=0.000000)],
#  [carla.Location(x=-179.349991, y=  12.450000, z= 0.000000),carla.Location(x=-179.250000, y=  -9.700000, z=0.000000),carla.Location(x=-201.399994, y= -10.300000, z=0.000000),carla.Location(x=-201.550003, y=  12.450000, z=0.000000)]]

#  [[191,192,189],
#  [190,181,147,169],
#  [198,197,196,195],
#  [148,149,150,152],
#  [146,145],
#  [193,194,151],
#  [170,171,172,173],
#  [165,166,167,168],
#  [154,153,156,155],
#  [184,186,185],
#  [174,177,175,176],
#  [161,164,162,163],
#  [157,160,158,159],
#  [187,188,180],
#  [178,183,179,182]]

