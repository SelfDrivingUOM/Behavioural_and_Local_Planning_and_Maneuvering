import numpy as np
import carla

#### --------------------------------- ####
# Functions used in the OVERTAKE state ####
#### --------------------------------- ####
ovetakeAllowed = [carla.LaneMarkingType.BrokenSolid, carla.LaneMarkingType.BrokenBroken, carla.LaneMarkingType.Broken]
def check_lane_closest(closest_vehicle, ego_vehicle,_map):
    ego_wpt = _map.get_waypoint(ego_vehicle.get_transform().location,project_to_road=True,lane_type=carla.LaneType.Driving)   
    cst_veh_wpt = _map.get_waypoint(closest_vehicle.get_transform().location,project_to_road=True,lane_type=carla.LaneType.Driving)  
    return ego_wpt.lane_id == cst_veh_wpt.lane_id
    
def can_we_overtake(ego_vehicle, vehicles_static, vehicles_dynamic, walkers, closest_vehicle, x_vec, y_vec, walkers_y, walkers_x, current_speed,_map,_world,glb_wpts):
    canOveretake = False
    rear_buffer = 8   # 5

    ##############################################################################
    # Find the distance to closest vehicle using the global wapypoint 
    _, ego_index = get_closest_index(np.array([ego_vehicle.get_transform().location.x, ego_vehicle.get_transform().location.y]),glb_wpts)
    _, veh_index = get_closest_index(np.array([closest_vehicle.get_transform().location.x, closest_vehicle.get_transform().location.y]),glb_wpts)
    print('index:',ego_index,veh_index)
    to_closest = 0
    for i in range(ego_index,veh_index-1):
        to_closest += np.linalg.norm(glb_wpts[i,:2]-glb_wpts[i+1,:2])
    ##############################################################################

    frwd_buffer = to_closest + 10 #15
    frwd_buffer_ego = to_closest + 10 #15
    roadRulesOK, rear_buffer_wpts, frwd_buffer_wpts, frwd_buffer_ego_wpts  = road_rules_ok(ego_vehicle, to_closest, rear_buffer, frwd_buffer, frwd_buffer_ego, _map,_world)
    # print("condition2")
    # print(roadRulesOK)
    # print("end of condition2")
    if roadRulesOK and ( not is_overtake_collisions((vehicles_static+vehicles_dynamic+walkers), closest_vehicle, rear_buffer_wpts, frwd_buffer_wpts, frwd_buffer_ego_wpts,_world)):
        canOveretake = True
    print("Can Overtake",canOveretake)
    return canOveretake

    # if is_prohibit_lane_markings() or is_on_intersection(ego_vehicle) or is_on_zebra_crossing(ego_vehicle):
    #     return False
    
    # pass

def road_rules_ok(ego_vehicle, to_closest, rear_buffer, frwd_buffer, frwd_buffer_ego, _map, _world):
    is_ok = True    # this indicates that we can perform the overtaking maneuver
    ego_waypoint=_map.get_waypoint(ego_vehicle.get_transform().location,project_to_road=True,lane_type=carla.LaneType.Driving)
    rear_buffer_wpts = None
    frwd_buffer_wpts = None
    frwd_buffer_ego_wpts = None
    
    # Get the waypoint in the left lane (backward) wrt to the ego vehicle and check if  lane change is allowed
    wpt = ego_waypoint.get_left_lane()
    # _world.debug.draw_string(wpt.transform.location, 'X', draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=100,persistent_lines=True)

    if (wpt.lane_id*ego_waypoint.lane_id)<0:
        rear_buffer, frwd_buffer = frwd_buffer, rear_buffer

    if wpt is None:
        is_ok = False
    else:
        dist = 0
        rear_buffer_wpts = np.array([[wpt.transform.location.x, wpt.transform.location.y]])
        while dist < rear_buffer:
            prev_wpt = wpt.previous(0.2)
            if len(prev_wpt)==1:
                rear_buffer_wpts = np.append(rear_buffer_wpts,np.array([[prev_wpt[0].transform.location.x,prev_wpt[0].transform.location.y]]),axis=0)
                dist += np.linalg.norm(rear_buffer_wpts[-1] - rear_buffer_wpts[-2])
                wpt = prev_wpt[0]
            else:
                # for wpt_ in prev_wpt:
                # _world.debug.draw_string(prev_wpt[0].transform.location, 'X', draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=60,persistent_lines=True)
                # _world.debug.draw_string(prev_wpt[1].transform.location, 'X', draw_shadow=False,color=carla.Color(r=0, g=255, b=0), life_time=40,persistent_lines=True)
                # while True:
                #     pass
                is_ok = False
                # print("dist",prev_wpt)
                # print("left lane backward waypoint is none")
                # raise Exception
                break
        # if is_ok:
        #     print('rear buffer ok')
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
                dist += np.linalg.norm(frwd_buffer_wpts[-1] - frwd_buffer_wpts[-2])
                next_wpt = next_wpt[0]
            else:
                is_ok = False
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
                dist += np.linalg.norm(frwd_buffer_ego_wpts[-1] - frwd_buffer_ego_wpts[-2])
                next_wpt = next_wpt[0]
            else:
                is_ok = False
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

def is_overtake_collisions(actor_list, closest_vehicle, rear_buffer_wpts, frwd_buffer_wpts, frwd_buffer_ego_wpts,_world):
    is_collision = True
    tresh_dist_sq = (3.7/2)**2  #half the width of a lane
    actor_loc = np.empty((0,2),dtype=np.float32)

    for actor in actor_list:
        if (actor!=closest_vehicle):
            actor_loc = np.append(actor_loc,np.array([[actor.get_transform().location.x,actor.get_transform().location.y]]),axis=0)
    buffer_wpts = np.concatenate((rear_buffer_wpts,frwd_buffer_wpts,frwd_buffer_ego_wpts))
    for b in rear_buffer_wpts:
        _world.debug.draw_string(carla.Location(x=b[0],y=b[1],z=1.843), 'X', draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=500,persistent_lines=True)
    for b in frwd_buffer_wpts:
        _world.debug.draw_string(carla.Location(x=b[0],y=b[1],z=1.843), 'X', draw_shadow=False,color=carla.Color(r=0, g=255, b=0), life_time=500,persistent_lines=True)
    for b in frwd_buffer_ego_wpts:
        _world.debug.draw_string(carla.Location(x=b[0],y=b[1],z=1.843), 'X', draw_shadow=False,color=carla.Color(r=255, g=255, b=255), life_time=500,persistent_lines=True)
    
    # print(actor_loc)
    # print(actor_loc[:,0].reshape((1,-1)))


    if (len(actor_loc)!=0):
        closest_dist = np.amin(np.square(buffer_wpts[:,0].reshape((-1,1)) - actor_loc[:,0].reshape((1,-1))) + np.square(buffer_wpts[:,1].reshape((-1,1)) - actor_loc[:,1].reshape((1,-1))))
        # print(np.square(buffer_wpts[:,0].reshape((-1,1)) - actor_loc[:,0].reshape((1,-1))) + np.square(buffer_wpts[:,1].reshape((-1,1)) - actor_loc[:,1].reshape((1,-1))))
        # print(tresh_dist_sq)
        # print("closest_dist",closest_dist)

        if closest_dist > tresh_dist_sq:
            is_collision = False
    else:
        is_collision = False

    # print("is collision",is_collision)
    return is_collision

def get_closest_index(ego_state, _waypoints):
    """
    Gets closest index a given list of waypoints to the vehicle position.
    """
    closest_len = float('Inf')
    closest_index = 0

    waypoint_dists = np.sqrt(np.square(_waypoints[:,0] - ego_state[0]) + np.square(_waypoints[:,1] - ego_state[1]))
    closest_len = np.amin(waypoint_dists)
    closest_index = np.where((waypoint_dists==closest_len))[0][0]
    #print(closest_len,closest_index)
    return closest_len, closest_index