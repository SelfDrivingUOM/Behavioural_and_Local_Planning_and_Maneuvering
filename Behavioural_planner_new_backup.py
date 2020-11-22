import numpy as np
import glob
import os
import sys
import time
import copy
import local_planner
from tools.misc import get_speed
from tools.misc import debug_print
from tools.misc import draw_bound_box
import carla


BP_LOOKAHEAD_BASE              = 10.0       # m
BP_LOOKAHEAD_TIME              = 1.0        # s
INTERSECTION_APPROACH_DISTANCE = 5
WALKER_THRESHOLD = 0.1

#states
FOLLOW_LANE            = 0
DECELERATE_TO_STOP     = 1
STAY_STOPPED           = 2
OVERTAKE               = 3
FOLLOW_LEAD_VEHICLE    = 4
INTERSECTION           = 5


dict_ = ["FOLLOW_LANE","DECELERATE_TO_STOP","STAY_STOPPED"]
##initial state

"""
    State Offsets

Follow Lane Offset  - 10cm


"""

FOLLOW_LANE_OFFSET = 0.1
DECELERATE_OFFSET = 0.1


class BehaviouralPlanner:
    def __init__(self, lp, waypoints,environment, world,HOP_RESOLUTION,world_map,ego_vehicle,resolution):
        self._lp            = lp
        self._waypoints     = waypoints
        self._goal_state    = [0.0, 0.0, 0.0]
        self._goal_index    = 0
        self._environment   = environment
        self._hop_resolution= HOP_RESOLUTION
        self._world         = world
        self._state         = FOLLOW_LANE
        self._lookahead     = BP_LOOKAHEAD_BASE
        self.paths = None
        self._goal_state_next =None
        self.min_collision = None
        self._collission_index = 48
        self._collission_actor = None
        self._map = world_map
        self._not_passed = False
        self.ego_vehicle = ego_vehicle
        self.started_decel = False
        self.stop_threshold = 0.01
        self.num_layers = None
        self.resolution = resolution


    def state_machine(self, ego_state, current_timestamp, prev_timestamp,current_speed):
        # print(dict_[self._state])
        open_loop_speed = self._lp._velocity_planner.get_open_loop_speed(current_timestamp - prev_timestamp)
        self._lookahead= BP_LOOKAHEAD_BASE + BP_LOOKAHEAD_TIME * open_loop_speed 
        vehicles_static, vehicles_dynamic, walkers,closest_vehicle,x_vec,walkers_y = self._environment.get_actors(max(20,self._lookahead))
        
        vehicles_static = list(vehicles_static)
        vehicles_dynamic = list(vehicles_dynamic)
        walkers = list(walkers)

        # print(vehicles_static,vehicles_dynamic,walkers)
        obstacle_actors = vehicles_static + vehicles_dynamic 
        
        # draw_bound_box(obstacle_actors,self._world)
        
        if (self._state   == FOLLOW_LANE):

            
            # First, find the closest index to the ego vehicle.
            closest_len, closest_index = self.get_closest_index(ego_state)
            # Next, find the goal index that lies within the lookahead distance
            # along the waypoints.
            goal_index = self.get_goal_index(ego_state, closest_len, closest_index)


            
            self._goal_index = goal_index
            # self._goal_state = self._waypoints[goal_index]
            self.num_layers = ((goal_index - closest_index)//(self.resolution+1))+1

            # print(self.num_layers)

            # print(do_stop,intersection)

            # if goal_index < (self._waypoints.shape[0]-1):
            #     point_1 = self._waypoints[goal_index+1]
            #     point_2 = self._waypoints[goal_index]

            # else: 
            #     point_1 = self._waypoints[goal_index]
            #     point_2 = self._waypoints[goal_index-1]

            # Compute the goal state set from the behavioural planner's computed goal state.
            # goal_set, goal_index_set =  self._lp.lattice_layer_stations(self._goal_state , self._waypoints, ego_state)
            # goal_state = goal_set[0]
            # goal_state_set = self._lp.get_goal_state_set(0,12,ego_state,FOLLOW_LANE_OFFSET,self.num_layers,self.resolution)
            goal_state_set = self._lp.get_goal_state_set(closest_index,goal_index,ego_state,FOLLOW_LANE_OFFSET,self.num_layers,self.resolution)
            # print(goal_state_set)
            # Calculate planned paths in the local frame.
            paths, path_validity,mid_path_len = self._lp.plan_paths(goal_state_set)

            # Transform those paths back to the global frame.
            paths = local_planner.transform_paths(paths, ego_state)

            collision_check_array,min_collision = self._lp._collision_checker.collision_check_static(paths, obstacle_actors,self._world)
            # print(min_collision,closest_vehicle)
            best_index = self._lp._collision_checker.select_best_path_index(paths, collision_check_array, self._goal_state,self._waypoints,ego_state)

            ego_waypoint = self._map.get_waypoint(carla.Location(x=ego_state[0], y=ego_state[1], z= 1.843102 ),project_to_road=True)
            ego_road = ego_waypoint.road_id
            ego_lane = ego_waypoint.lane_id
            ego_section = ego_waypoint.section_id

            # print(self.is_approaching_intersection(self._waypoints,closest_index,ego_state))

            # if((abs(best_index - self._lp._num_paths//2))>= 3):
        
            #     self._state   = DECELERATE_TO_STOP
            #     self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
            #     self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
            #     self._goal_state[2] = 0
            # print(self._map,walkers,ego_state,closest_index,self._waypoints,paths,best_index,x_vec,min_collision,walkers_y,mid_path_len)
            walker_collide,col_walker,min_collision = self.check_walkers(self._map,walkers,ego_state,paths,best_index,x_vec,min_collision,walkers_y,mid_path_len)
            # print(walker_collide,col_walker,min_collision )
            if(walker_collide):
                self._collission_actor = col_walker
                self._state   = DECELERATE_TO_STOP
                self._collission_index = min_collision

            
            elif(self.lane_paths_blocked(best_index)): ##LANE PATHS BLOCKED
                # print(" ")
                if(self.can_overtake()):

                    ##### FIX GOAL STATE PROPERLY
                    self._state   = OVERTAKE
                    self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
                    self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
                    self._goal_state[2] = 0
                
                
                elif(self.need_to_stop(closest_vehicle,closest_index,ego_state)):
                    # print("LOL")
                    self._collission_actor = closest_vehicle
                    self._state   = DECELERATE_TO_STOP
                    self._collission_index = min_collision
                    

                else:

                    self._state   = FOLLOW_LEAD_VEHICLE
                    self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
                    self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
                    self._goal_state[2] = 0

            # elif(self.is_approaching_intersection(self._waypoints,closest_index,ego_state)):

            #     self._state   = INTERSECTION
            #     self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
            #     self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
            #     self._goal_state[2] = 0
            else:
                # print( " ")
                self._state   = FOLLOW_LANE

            # best_index = 5

            if(best_index == None):
                best_index = self._lp._num_paths//2
            
            best_path = paths[best_index]
        
            debug_print(paths,self._world,best_index)
            local_waypoints = self._lp._velocity_planner.nominal_profile(best_path, open_loop_speed, self._goal_state[2])

            self.paths = paths
            # self._collission_index = min_collision
            return local_waypoints

######################################################################################################################################
#************************************************************************************************************************************#
######################################################################################################################################

        elif (self._state == DECELERATE_TO_STOP):
            
            # print()
            # vehicles_static, vehicles_dynamic, walkers,closest_vehicle,x_vec,walkers_y = self._environment.get_actors(max(20,self._lookahead))
            # print(self._collission_index,self.started_decel)
            self._goal_state = self.paths[self._lp._num_paths//2,:,max(self._collission_index,1)]
            self._goal_state_next = self.paths[self._lp._num_paths//2,:,max(self._collission_index-1,0)]
            self._goal_state[2] = 0
            
            goal_state_set = self._lp.get_goal_state_set(self._goal_state,self._goal_state_next, self._goal_state, ego_state,DECELERATE_OFFSET)

            paths, path_validity,mid_path_len = self._lp.plan_paths(goal_state_set)
            # Transform those paths back to the global frame.
            paths = local_planner.transform_paths(paths, ego_state)

            collision_check_array,min_collision = self._lp._collision_checker.collision_check_static(paths, obstacle_actors,self._world)   
            best_index = self._lp._collision_checker.select_best_path_index(paths, collision_check_array, self._goal_state,self._waypoints,ego_state)
            best_index = self._lp._num_paths//2

            debug_print(paths,self._world,best_index)
            # print("a")

            
            # print(local_waypoints)
            walker_collide,col_walker,min_collision = self.check_walkers(self._map,walkers,ego_state,paths,best_index,x_vec,min_collision,walkers_y,mid_path_len)
            local_waypoints = self._lp._velocity_planner.decelerate_profile(paths[best_index],current_speed,min_collision)
            if(self.is_ego_less()):
                # print("A")
                self._state = STAY_STOPPED


            elif(walker_collide):
                # print("B")
                self._collission_actor = col_walker
                self._state   = DECELERATE_TO_STOP
                self._collission_index = min_collision

            
            
            elif(self.lane_paths_blocked(best_index)): ##LANE PATHS BLOCKED

                # print(" ")
                need_to_stop,intersection = self.need_to_stop(closest_vehicle,closest_index,ego_state)
                if(need_to_stop):
                    # print("C")
                    self._collission_actor = closest_vehicle
                    self._state   = DECELERATE_TO_STOP
                    self._collission_index = min_collision
                    

                else:
                    # print("D")
                    self._state   = FOLLOW_LEAD_VEHICLE
                    self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
                    self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
                    self._collission_index = min_collision
                    self._goal_state[2] = 0

            # elif(self.is_approaching_intersection(self._waypoints,closest_index,ego_state)):

            #     self._state   = INTERSECTION
            #     self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
            #     self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
            #     self._goal_state[2] = 0
            else:
                # print(" ")
                # print("E")
                self._state   = FOLLOW_LANE
                self._collission_index = min_collision


            # raise Exception

        
            
            if(not self.started_decel):

                # print(min_collision)
                self.paths = paths
                self.started_decel = True

                # debug_print(paths,self._world,best_index,life = 1000)
                # raise Exception

            return local_waypoints

        elif (self._state   == STAY_STOPPED):
            pass

        elif(self._state == OVERTAKE):

            pass
            

    def get_closest_index(self, ego_state):
        """
        Gets closest index a given list of waypoints to the vehicle position.
        """
        closest_len = float('Inf')
        closest_index = 0

        waypoint_dists = np.sqrt(np.square(self._waypoints[:,0] - ego_state[0]) + np.square(self._waypoints[:,1] - ego_state[1]))
        closest_len = np.amin(waypoint_dists)
        closest_index = np.where((waypoint_dists==closest_len))[0][0]
        return closest_len, closest_index

    def get_goal_index(self, ego_state, closest_len, closest_index):
        """
        Gets the goal index for the vehicle.

        Find the farthest point along the path that is within the lookahead 
        distance of the ego vehicle. Take the distance from the ego vehicle
        to the closest waypoint into consideration.
        """
        arc_length = closest_len
        wp_index = closest_index
        
        # In this case, reaching the closest waypoint is already far enough 
        # for the planner. No need to check additional waypoints.
        if arc_length > self._lookahead:
            return wp_index

        # We are already at the end of the path.
        if wp_index == (self._waypoints.shape[0] - 1):
            return wp_index

        # Otherwise, find our next waypoint.
        while wp_index < self._waypoints.shape[0] - 1:
            arc_length += np.sqrt((self._waypoints[wp_index][0] - self._waypoints[wp_index+1][0])**2 + (self._waypoints[wp_index][1] - self._waypoints[wp_index+1][1])**2)
            wp_index += 1
            if arc_length > self._lookahead: break

        return wp_index


    def is_approaching_intersection(self, waypoints, closest_index,ego_state):

        index_length_for_approaching = int(INTERSECTION_APPROACH_DISTANCE/self._hop_resolution)
        checking_waypoint = waypoints[closest_index+index_length_for_approaching]
        loc=carla.Location(x=checking_waypoint[0] , y=checking_waypoint[1],z = 1.843102)

        exwp = self._map.get_waypoint(loc)
        ego_wayp =  self._map.get_waypoint(carla.Location(x=ego_state[0] , y=ego_state[1],z= 1.843102))

        #####

        out=exwp.is_junction
        intersection = False
        
        if(out):
            self._not_passed = True
            intersection = True
            
            # return out
    
        elif(ego_wayp.is_junction):
            self._not_passed = False
            intersection = True
            # return True

        elif(self._not_passed):
            intersection = True
            # return True

        else:
            intersection = False
            # return False
                # return True
                # self._within_int = False
                # return True
            # else:
            #     return False
        if(intersection):
            
        else:
            return intersection,None

        # print(out)

    def lane_paths_blocked(self,best_index):
        
        
        if(best_index == None):
            return True

        # elif()
        elif(self._collission_actor != None):
            dist_closest = np.sum(np.square(np.array([self._collission_actor.get_location().x,self._collission_actor.get_location().y]) \
                             - np.array([self.ego_vehicle.get_location().x,self.ego_vehicle.get_location().y])))
            if(dist_closest< self._lookahead):
                return True

            else:
                return False
        else:
            return False

    def is_ego_less(self):
        velocity = self.ego_vehicle.get_velocity()
        if(np.linalg.norm(np.array([velocity.x,velocity.y,velocity.z]))<self.stop_threshold):
            return True

        else:
            return False


    def check_walkers(self,world_map,walkers,ego_state,paths,best_index,vec_rd,min_collision,walkers_y,mid_path_len):

            if(best_index == None):
                print("brrrr")
                best_index = self._lp._num_paths//2

            best_path = paths[best_index]

            ego_waypoint = world_map.get_waypoint(carla.Location(x=ego_state[0], y=ego_state[1], z= 1.843102 ),project_to_road=True,lane_type = carla.LaneType.Driving)
            ego_road = ego_waypoint.road_id
            ego_lane = ego_waypoint.lane_id
            ego_section = ego_waypoint.section_id
            # print(best_path.shape, ego_section)
            dest_waypoint = world_map.get_waypoint(carla.Location(x=best_path[0][-1], y=best_path[1][-1], z= 1.843102 ),project_to_road = True, lane_type =carla.LaneType.Driving)
            dest_road = dest_waypoint.road_id
            dest_lane = dest_waypoint.lane_id
            dest_section= dest_waypoint.section_id

            # vec_x = waypoints[closest_index+1][0] - ego_state[0]
            # vec_y = waypoints[closest_index+1][1] - ego_state[1]
            # vec = np.array([vec_x,vec_y])
        
            # # print(vec,)
            # # head = np.array([np.cos(np.radians(ego_state[2])),np.sin(np.radians(ego_state[2]))])
            # rot_mat = np.array([[np.cos(np.pi/2),-np.sin(np.pi/2)],
            #                     [np.sin(np.pi/2),np.cos(np.pi/2)]])
            # vec_rd = np.matmul(rot_mat,vec)
            # vec_rd = vec_rd / (np.sqrt(np.sum(np.square(vec_rd))))
            counter = -1
            if walkers:
                print(walkers)
                for person in walkers:
                    counter+=1
                    walker_loc= person.get_location()
                    walker_waypoint=world_map.get_waypoint(carla.Location(x=walker_loc.x, y=walker_loc.y, z= walker_loc.z ),project_to_road=True,lane_type = ( carla.LaneType.Driving | carla.LaneType.Sidewalk ))
                    # print(walker_waypoint.lane_type)
                    w_lane = walker_waypoint.lane_id
                    # vec_wx = walker_loc.x  - ego_state[0]
                    # vec_wy = walker_loc.y  - ego_state[1]
                    # vec_w = np.array([vec_wx,vec_wy])
                    # cross_ = np.cross(vec_rd,vec_w)
                    
                    # print(cross_,ego_lane,ped_cross)
                    # print(vec,vec_w)
                    
                    # if (cross>0):
                    walker_pt1 = np.array([walker_loc.x,walker_loc.y]) + vec_rd
                    walker_pt2 = np.array([walker_loc.x,walker_loc.y]) - vec_rd


                    # print(walker_pt1.shape,vec_rd)
                    walker_way_pt1=world_map.get_waypoint(carla.Location(x=walker_pt1[0], y=walker_pt1[1], z= walker_loc.z ),project_to_road=True, lane_type = ( carla.LaneType.Driving | carla.LaneType.Sidewalk ))
                    walker_way_pt2=world_map.get_waypoint(carla.Location(x=walker_pt2[0], y=walker_pt2[1], z= walker_loc.z),project_to_road=True, lane_type = ( carla.LaneType.Driving | carla.LaneType.Sidewalk ))
                    w_pt1_lane = walker_way_pt1.lane_id
                    w_pt2_lane = walker_way_pt2.lane_id
                    w_speed = person.get_control().speed
                    
                    # walker_waypoint=world_map.get_waypoint(carla.Location(x=walker_loc.x, y=walker_loc.y, z= 1.843102 ),project_to_road=True)
                    w_road = walker_waypoint.road_id
                    w_section = walker_waypoint.section_id
                    # print(walker_waypoint.lane_type,walker_waypoint.is_junction,dest_waypoint.is_junction)


                    # self._world.debug.draw_string(walker_waypoint.transform.location, 'X', draw_shadow=False,color=carla.Color(r=0, g=255, b=0), life_time=10000,persistent_lines=True)



                    if ((dest_waypoint.is_junction) and (walker_waypoint.is_junction)):
                        print("X")
                        # print(walkers_y[counter],mid_path_len,(mid_path_len/49)*(min_collision+1))
                        if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                            # print("XX")
                            min_collision = int(walkers_y[counter]//(mid_path_len/49))
                            # print(min_collision)
                        # print(min_collision)
                        return True,person,min_collision
                    
                    elif (ego_section==w_section):
                        if (ego_lane==w_lane):
                            print("A")
                            if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                min_collision = int(walkers_y[counter]//(mid_path_len/49))


                            return True,person,min_collision      # decelarate to stop

                        elif (ego_lane-1==0):
                            if ((w_lane==ego_lane-2) or (w_lane==ego_lane-1)or (w_lane==ego_lane+1)):
                                if (((ego_lane==w_pt1_lane) or (ego_lane==w_pt2_lane)) and (w_speed>WALKER_THRESHOLD)):
                                    if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                        min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                    print("B")
                                    return True,person,min_collision       # check velocity
                                else:
                                    return False,None,min_collision

                            else:
                                return False,None,min_collision

                        elif (ego_lane+1==0):
                            if ((w_lane==ego_lane+2) or (w_lane==ego_lane+1) or (w_lane==ego_lane-1)):
                                if (((ego_lane==w_pt1_lane) or (ego_lane==w_pt2_lane)) and (w_speed>WALKER_THRESHOLD)):
                                    print("C")                        
                                    if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                        min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                    return True,person,min_collision      # check velocity
                                else:
                                    return False,None,min_collision
                            else:
                                return False,None,min_collision
                                
                        elif ((w_lane==ego_lane-1) or (w_lane==ego_lane+1)):
                            if (((ego_lane==w_pt1_lane) or (ego_lane==w_pt2_lane)) and (w_speed>WALKER_THRESHOLD)):
                                    print("D")                        
                                    if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                        min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                    return True,person,min_collision       # check velocity
                            else:
                                return False,None,min_collision
                        else:
                            return False,None,min_collision
                    
                    else:
                        if (dest_section==w_section) :
                            if (dest_lane==w_lane):
                                print("E")                        
                                if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                        min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                return True,person,min_collision       # decelarate to stop

                            elif (dest_lane-1==0):
                                if ((w_lane==dest_lane-2) or (w_lane==dest_lane-1) or (w_lane==dest_lane+1)):
                                    if (((dest_lane==w_pt1_lane) or (dest_lane==w_pt2_lane)) and (w_speed>WALKER_THRESHOLD)):
                                        print("F")                        
                                        if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                            min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                        return True,person,min_collision       # check velocity
                                    else:
                                        return False,None,min_collision
                                else:
                                    return False,None,min_collision

                            elif (dest_lane+1==0):
                                if ((w_lane==dest_lane+2) or (w_lane==dest_lane+1) or (w_lane==dest_lane-1)):
                                    if (((dest_lane==w_pt1_lane) or (dest_lane==w_pt2_lane)) and (w_speed>WALKER_THRESHOLD)):
                                        print("G")                        
                                        if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                            min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                        return True,person,min_collision       # check velocity
                                    else:
                                        return False,None,min_collision     # check velocity
                                else:
                                    return False,None,min_collision
                                    
                            elif ((w_lane==dest_lane-1) or (w_lane==dest_lane+1)):
                                if (((dest_lane==w_pt1_lane) or (dest_lane==w_pt2_lane)) and (w_speed>WALKER_THRESHOLD)):
                                    print("H")                        
                                    if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                        min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                    return True,person,min_collision       # check velocity
                                else:
                                    return False,None,min_collision             # check velocity
                            else:
                                return False,None,min_collision
                    
                        else:
                            return False,None,min_collision
            else:
                return False,None,min_collision 

    """
    This function return 2 values

        i. Do we need to stop (bool)
        ii. Are we at n intersection (bool)

    These will be used as necessary 
    """

    def need_to_stop(self,lead_vehicle,closest_index,ego_state):
        # print(lead_vehicle,closest_index)
        # stop = False
        is_intersection = self.is_approaching_intersection(self._waypoints,closest_index,ego_state)       
        # distance_lead = get_road_dist(collission_idx)
        
        if(is_intersection):
            green_light = self.is_green_light(self.ego_vehicle)

            # print(green_light)
            if(green_light):
                return False,is_intersection
            else:
                # stp = not stop
                return True,is_intersection
        else:

            if(lead_vehicle!=None):
                #lead_vehicle = lane_vehicles[0]
                lead_velocity = lead_vehicle.get_velocity()
                val_ = np.linalg.norm([lead_velocity.x,lead_velocity.y,lead_velocity.z])

                if(val_ <0.5):
                    return True,is_intersection 
                else:
                    return False, is_intersection
            else:
            
                return True,is_intersection

    """
    This function returns distance to the lead vehicle
    """

    def get_road_dist(self,collission_idx,path):


        mask = np.ones(collission_idx-1,path.shape[0],1)-np.ones(collission_idx-1,path.shape[0])
        diff_points = mask@path

        diff_points = np.square(diff_points)

        distance_lead = np.sum(np.sum(diff_points,axis =1))
        
        return distance_lead

    """
    This function returns distance to the lead vehicle
    """

    def is_green_light(self,ego_vehicle):

        traffic_light = ego_vehicle.get_traffic_light_state()
        # print(traffic_light)
        if(traffic_light =="Green"):
            return True
        else:
            return False

    def can_overtake(self):

        return False 







