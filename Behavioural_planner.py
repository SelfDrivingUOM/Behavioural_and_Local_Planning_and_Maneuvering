######################################################
######################################################
##############   BEHAVIOURAL PLANNER  ################
######################################################
##  This is the top layer of the code and handle    ## 
##  all maneuvers in below layers                   ##
##  2021.02.10                                      ##
##  Authors - Gershom                               ##
##          - Saumya                                ##
##          - Yasintha                              ##
######################################################
######################################################




######################################################
#####                 FILE IMPORTS              ######
######################################################

import numpy as np
import glob
import os
import sys
import time
import copy
import local_planner
from tools.misc import get_speed
from tools.misc import debug_print
from tools.misc import draw_bound_box,draw_bound_box_actor,compute_magnitude_angle,is_within_distance_ahead
import tools.misc as misc
import time 
import carla

from overtake_state import check_lane_closest, can_we_overtake

######################################################
#####              GLOBAL VARIABLES             ######
######################################################

# states
FOLLOW_LANE                     = 0
DECELERATE_TO_STOP              = 1
STAY_STOPPED                    = 2
INTERSECTION                    = 3
FOLLOW_LEAD_VEHICLE             = 4
OVERTAKE                        = 5
EMERGENCY_STOP                  = 6

dict_                           = ["FOLLOW_LANE","DECELERATE_TO_STOP","STAY_STOPPED","INTERSECTION","FOLLOW_LEAD_VEHICLE","OVERTAKE","EMERGENCY_STOP"]

# important variables
SPEED                           = 5       # Vehicle speed (m/s)
TRAFFIC_LIGHT_CHECK_DISTANCE    = 35      # Distance to detect traffic lights (m)
BP_LOOKAHEAD_BASE               = 8.0     # Base distance to create lattice paths (m)
OVERTAKE_LOOKAHEAD_BASE         = 1.5     # Base distance to create lattice paths (m) when in overtake state
FOLLOW_LEAD_RANGE               = 15 #9   # Range to follow lead vehicles (m)
DEBUG_STATE_MACHINE             = False   # Set this to true to see all function outputs in state machine. This is better for full debug
ONLY_STATE_DEBUG                = True    # Set this to true to see current state of state machine
UNSTRUCTURED                    = True    # Set this to True to behave according to the unstructured walkers
FOLLOW_LANE_OFFSET              = 0.1     # Path goal point offset in follow lane  (m)
DECELERATE_OFFSET               = 0.1     # Path goal point offset in decelerate state (m) 
Z                               = 1.843102

# normal variables
BP_LOOKAHEAD_TIME               = 1.0     # Lookahead creating time (s) 
INTERSECTION_APPROACH_DISTANCE  = 10      # This is used to chnage state machine to intersection state (m)
WALKER_THRESHOLD                = 0.1     # This is used to identify dynamic walkers (m/s)
HEADING_CHECK_LOOKAHEAD         = 10      # Lookahead to identify the turning direction in junctions (m)
JUNCTION_HEADING_CHECK_ANGLE    = 30      # To identify the turning direction (degrees)     
GET_ACTOR_RANGE                 = 40      # Radius to filter actors
DIST_WALKER_INTERSECTION        = 9       # Dont decrease this unless you make the velocity planner correct (m)
WALKER_DIST_RANGE_BASE          = 5       # Minimum walker detection distance in unsrtuctured  (m)
WALKER_DIST_RANGE_MAX           = 15      # Maximum walker detection distance in unsrtuctured  (m)
LANE_WIDTH_WALKERS              = 2       # Width of checking walkers for botth left and right (m)
LEAD_SPEED_THRESHOLD            = 0.5     # Threshold to stop when there is a lead vehicle


class BehaviouralPlanner:
    def __init__(self, world, world_map, ego_vehicle, environment, lp, waypoints, HOP_RESOLUTION):
        """
        param   : world             : CARLA world
                : world_map         : CARLA world map
                : ego_vehicle        
                : environment       : Class to get the information about the actors around the ego-vehicle 
                                      within a radius
                : lp                : Initialized local planner in the main module
                : waypoints         : Numpy array containing global waypoints of the path of ego-vehicle
                                      Length and speed in m and m/s.
                                      format - numpy.ndarray[[x coordinate,y coordinate, ego-vehicle speed] - waypoint 0
                                                             [x coordinate,y coordinate, ego-vehicle speed] - waypoint 1
                                                                        :                   :
                                                                        :                   :
                                                             [x coordinate,y coordinate, ego-vehicle speed] - waypoint n-2
                                                             [x coordinate,y coordinate, ego-vehicle speed]] - waypoint n-1  
                : HOP_RESOLUTION    : Distance between two waypoints (m)

        """
        self._world                         = world
        self._map                           = world_map
        self._ego_vehicle                   = ego_vehicle
        self._environment                   = environment
        self._lp                            = lp
        self._waypoints                     = waypoints
        self._hop_resolution                = HOP_RESOLUTION

        self._state                         = FOLLOW_LANE           # State of the ego-vehicle
        self._previous_state                = None                  # Previous state of the ego-vehicle

        self._lookahead                     = BP_LOOKAHEAD_BASE     # Paths making lookahead of ego-vehicle 
        self._prev_lookahead                = BP_LOOKAHEAD_BASE     # Paths making lookahead of ego-vehicle in previous state
        self._paths                          = None                  # Lattice Paths of ego vehicle. 
                                                                    # Numpy array containing the paths provide by the local planner.
                                                                    # numpy.ndarray[[ [path1_x1,path1_x2,.....,path1_xn],    (x)
                                                                    #                 [path1_y1,path1_y2,.....,path1_yn],    (y)    Path1
                                                                    #                 [path1_h1,path1_h2,.....,path1_hn] ]  (teta)    :
                                                                    #                                 :                                :
                                                                    #                                 :                                :
                                                                    #                 [ [pathN_x1,pathN_x2,.....,pathN_xn],    (x)      :
                                                                    #                 [pathN_y1,pathN_y2,.....,pathN_yn],    (y)    PathN
                                                                    #                 [pathN_h1,pathN_h2,.....,pathN_hn] ]] (teta) 

        self._goal_index                    = 0                     # Index of goal waypoint in global waypoints array   
        self._goal_state                    = [0.0, 0.0, 0.0]       # [x coordinate,y coordinate, ego-vehicle speed]  of the goal point
        self._goal_state_next               = None                  # [x coordinate,y coordinate, ego-vehicle speed]  of the next goal point
        self._collission_index              = 48                    # Least local collision index from all the paths in paths array.
        self._collission_actor              = None                  # Collision actor according to state
        self._not_passed                    = False                 # Flag to check whether interesection is fully passed or not
        self._started_decel                 = False                 # Flag to check whether first time of deceleration or not
        self.stop_threshold                 = 0.01                  # Ego vehicle goes to STAY_STOPPED state from DECELERATE_TO_STOP state when the speed is less than this threshold
        self.num_layers                     = None                  # Number of path layers in local planner
        self._first_time                    = False                 # Flag to make high on when entering the intersection and to make low otherwise
        self._triangle_points               = None                  # Get the relevant traingle points to check pedestrians in intersections in structured environment           
        self._junc_bx_pts                   = None                  # Get the relevant box points of an intersections                             
        self._traffic_lights                = self._environment.lights_list # All the traffic lights in CARLA world       
        self._proximity_threshold           = 10.0                  # Max distance from a reference object
        self._stopped                       = None                  # Flag to check whether stopped at a intersection or not
        self._color_light_state             = None                  # State of color light, red or not
        self._intersection_state            = None                  # this is to print intersection state. check only
        self._need_to_stop                  = None                  # check only
        self._best_index_from_decelerate    = None                  # Index of the best feasible path in DECELERATE_TO_STOP state                      
        self._open_loop_speed               = None
        self._overtakeLookahead             = 0
        self._overtakeVeh                   = None
        self._isOvertake                    = 0

    ######################################################
    #####              State Machine                ######
    ######################################################

    def state_machine(self, ego_state, current_timestamp, prev_timestamp,current_speed):
        """
        param   : ego_state         : List containing location and heading of ego- vehicle
                                      [x coordinate of ego vehicle (m), y coordinate of ego vehicle (m), yaw of ego vehicle (degrees)]
                : current timestamp : Time difference between current time and simulation starting time
                : prev_timestamp    : Time difference between time in previous loop and simulation starting time 
                : current_speed     : Current speed of ego-vehicle 

        return  : local_waypoints   : List which contains the local path as well as the speed to be tracked by the controller (global frame).
                                      Length and speed in m and m/s.
                                        Format: [[x0, y0, v0],
                                                [x1, y1, v1],
                                                ...,
                                                [xm, ym, vm]]
                                        example:
                                            local_waypoints[2][1]: 
                                            returns the 3rd point's y position in the local path

                                            local_waypoints[5]:
                                            returns [x5, y5, v5] (6th point in the local path)
        """

        # set map traffic lights green
        # self.traffic_light_green(self._traffic_lights)

        if(self._collission_actor!=None):
            draw_bound_box_actor(self._collission_actor,self._world,255,0,0)
        if(ONLY_STATE_DEBUG):
            print(dict_[self._state])

        open_loop_speed = self._lp._velocity_planner.get_open_loop_speed(current_timestamp - prev_timestamp)
        self._open_loop_speed = open_loop_speed
        self._lookahead = (BP_LOOKAHEAD_BASE*(1-self._isOvertake)) + (OVERTAKE_LOOKAHEAD_BASE*self._isOvertake) + (BP_LOOKAHEAD_TIME * open_loop_speed)

        ################## Get list of actors seperately #########################################
        vehicles_static, vehicles_dynamic, walkers, closest_vehicle, x_vec, y_vec, walkers_y, walkers_x = self._environment.get_actors(GET_ACTOR_RANGE,self._paths,self._lp._num_paths//2,  self._intersection_state)
        vehicles_static = list(vehicles_static)
        vehicles_dynamic = list(vehicles_dynamic)
        walkers = list(walkers)
        obstacle_actors = vehicles_static + vehicles_dynamic
        all_obstacle_actors = vehicles_static + vehicles_dynamic + walkers

        # if(closest_vehicle!=None):
        #     ######checking lane
        #     ego_waypoint = self._map.get_waypoint(self._ego_vehicle.get_transform().location,project_to_road=True)
        #     ego_lane = ego_waypoint.lane_id
        #     lead_waypoint = self._map.get_waypoint(closest_vehicle.get_transform().location,project_to_road=True)
        #     lead_lane = lead_waypoint.lane_id
        #     print("ego",ego_lane)    
        #     print("lead",lead_lane)    
        #     draw_bound_box_actor(closest_vehicle,self._world,0,0,0)
        #     self._world.debug.draw_box(bounding_box,transform.rotation,1, carla.Color(255,0,0,0),0.001)
        # else:
        #     print("No lead vehicle")

        ###########################################################################################

        ########################## Ego-vehicle information #######################################
        ego_location = carla.Location(x=ego_state[0], y=ego_state[1], z= Z )
        ego_waypoint = self._map.get_waypoint(ego_location,project_to_road = True, lane_type = carla.LaneType.Driving)
        ego_lane = ego_waypoint.lane_id
        ##########################################################################################

        ########################## Making the emergency stop array ################################
        """
        Have considered only one meter distance from the centre of ego-vehicle to the 
        heading direction and have collision checked with actors. Coordinates are duplicated 
        in emergency_array as the collision checker returns 1 when a collision
        not detected and 0 when a collision is detected
        """
        emergency_loc = 1* y_vec
        emergency_array= np.array([[[emergency_loc[0]+ego_state[0],emergency_loc[0]+ego_state[0]],
                                    [emergency_loc[1]+ego_state[1],emergency_loc[1]+ego_state[1]],
                                    [0+ego_state[2],0+ego_state[2]]]])
        ###########################################################################################


        if (self._state   == FOLLOW_LANE):
            self._previous_state = self._state
            
            # Find the closest index to the ego vehicle.
            closest_len, closest_index = self.get_closest_index(ego_state)
            # Find the goal index that lies within the lookahead distance along the waypoints.
            goal_index = self.get_goal_index(ego_state, closest_len, closest_index)
            self._goal_index = goal_index
            # Set the goal state by getting the location and giving derired speed
            self._goal_state = self._waypoints[goal_index]
            self._goal_state[2] = SPEED

            self.num_layers = (goal_index - closest_index)//5
        
            goal_location = carla.Location(x=self._goal_state[0], y=self._goal_state[1], z= Z )
            goal_waypoint = self._map.get_waypoint(goal_location,project_to_road=True)

            if goal_index < (self._waypoints.shape[0]-1):
                point_1 = self._waypoints[goal_index+1]
                point_2 = self._waypoints[goal_index-1]

            else: 
                point_1 = self._waypoints[goal_index]
                point_2 = self._waypoints[goal_index-2]

            goal_state_set = self._lp.get_goal_state_set(point_1,point_2,self._goal_state, ego_state,FOLLOW_LANE_OFFSET)

            # Calculate planned paths in the local frame.
            paths, path_validity,mid_path_len = self._lp.plan_paths(goal_state_set)

            # Transform those paths back to the global frame.
            paths = local_planner.transform_paths(paths, ego_state)

            # Check collisions for every path
            collision_check_array,min_collision, min_collision_actor = self._lp._collision_checker.collision_check_static(paths, all_obstacle_actors,self._world)
            emergency_collision_check_array,emergency_min_collision,emg_min_collision_actor = self._lp._collision_checker.collision_check_static(emergency_array, all_obstacle_actors,self._world)

            # Calculate the index of the best feasible path
            best_index = self._lp._collision_checker.select_best_path_index(paths, collision_check_array, self._goal_state,self._waypoints,ego_state)

            # Check whether the ego vehicle is approaching an intersection
            intersection,triangle_points,junc_bx_pts = self.is_approaching_intersection(self._waypoints, closest_index, ego_state, ego_waypoint)
            self._intersection_state = intersection

            # Check for walkers
            walker_collide,col_walker,min_collision = self.check_walkers(self._map, walkers, ego_state, ego_lane, paths, best_index,\
                                                        x_vec, min_collision, walkers_y, walkers_x, mid_path_len, intersection, triangle_points, junc_bx_pts)
            
             # Check whether the ego vehicle needs to stop
            need_to_stop= self.need_to_stop(closest_vehicle,closest_index,ego_location,ego_waypoint,goal_waypoint,ego_state,min_collision,min_collision_actor,intersection)
            self._need_to_stop = need_to_stop    
            
            # Check all paths of the ego vehicle is blocked or not
            lane_path_blcked = self.lane_paths_blocked(best_index)

            if (DEBUG_STATE_MACHINE):
                print("{:<20} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25}".format(   "Pedestrian={}".format(walker_collide), \
                                                                                                    "LanePathBlock={}".format(lane_path_blcked), \
                                                                                                    "CanOvertake={}".format(self.can_overtake(ego_state,vehicles_static, vehicles_dynamic, walkers, closest_vehicle, x_vec, y_vec, walkers_y, walkers_x,current_speed)), \
                                                                                                    "DoWeNeedToStop={}".format(self._need_to_stop), \
                                                                                                    "IsIntersection={}".format(self._intersection_state), \
                                                                                                    "{}".format('-'*20), \
                                                                                                    "{}".format('-'*20), \
                                                                                                    "{}".format(dict_[self._previous_state])  ))
           
            if (emergency_min_collision!=1):
                self._state   = EMERGENCY_STOP
                   
            elif(walker_collide):
                self._collission_actor = col_walker
                self._state   = DECELERATE_TO_STOP
                self._collission_index = min_collision
 
            elif(lane_path_blcked):  
                if(self.can_overtake(ego_state,vehicles_static, vehicles_dynamic, walkers, closest_vehicle, x_vec, y_vec, walkers_y, walkers_x,current_speed)):
                    ##### FIX GOAL STATE PROPERLY
                    self._state   = OVERTAKE
                    self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
                    self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
                    self._goal_state[2] = 0

                elif((not need_to_stop) and (closest_vehicle!=None)) :
                    self._collission_actor = closest_vehicle
                    self._state   = FOLLOW_LEAD_VEHICLE
                    self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
                    self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
                    self._goal_state[2] = 0
                
                else:
                    self._prev_lookahead = self._lookahead
                    self._collission_actor = closest_vehicle
                    self._state   = DECELERATE_TO_STOP
                    self._collission_index = min_collision
                    
            elif(self._intersection_state):
                self._state   = INTERSECTION
                self._collission_actor = closest_vehicle
                self._collission_index = min_collision
            
            else:
                self._state   = FOLLOW_LANE
                self._collission_actor = closest_vehicle
                self._collission_index = min_collision

            if(best_index == None):
                best_index = self._lp._num_paths//2
            
            best_path = paths[best_index]
            self._paths = paths
            debug_print(paths,self._world,best_index)

            local_waypoints = self._lp._velocity_planner.nominal_profile(best_path, open_loop_speed, self._goal_state[2])
            return local_waypoints
    
        elif (self._state == DECELERATE_TO_STOP):
            self._previous_state = self._state

            closest_len, closest_index = self.get_closest_index(ego_state)
            goal_index = self.get_goal_index(ego_state, closest_len, closest_index)
            self._goal_index = goal_index
            self._goal_state = self._waypoints[goal_index]
            self._goal_state[2] = 0
            
            goal_location = carla.Location(x=self._goal_state[0], y=self._goal_state[1], z= Z )
            goal_waypoint = self._map.get_waypoint(goal_location,project_to_road=True)

            if goal_index < (self._waypoints.shape[0]-1):
                point_1 = self._waypoints[goal_index+1]
                point_2 = self._waypoints[goal_index-1]

            else: 
                point_1 = self._waypoints[goal_index]
                point_2 = self._waypoints[goal_index-2]

            goal_state_set = self._lp.get_goal_state_set(point_1,point_2,self._goal_state, ego_state,FOLLOW_LANE_OFFSET)

            paths, path_validity,mid_path_len = self._lp.plan_paths(goal_state_set)
            paths = local_planner.transform_paths(paths, ego_state)

            collision_check_array,min_collision, min_collision_actor = self._lp._collision_checker.collision_check_static(paths, obstacle_actors,self._world)   
            emergency_collision_check_array,emergency_min_collision,emg_min_collision_actor = self._lp._collision_checker.collision_check_static(emergency_array, all_obstacle_actors,self._world)

            best_index = self._lp._collision_checker.select_best_path_index(paths, collision_check_array, self._goal_state,self._waypoints,ego_state)
            self._best_index_from_decelerate = best_index
            best_index = self._lp._num_paths//2
            debug_print(paths,self._world,best_index)

            intersection,triangle_points, junc_bx_pts = self.is_approaching_intersection(self._waypoints,max(self._collission_index,1),ego_state, ego_waypoint)
            self._intersection_state = intersection

            walker_collide,col_walker,min_collision = self.check_walkers(self._map,walkers,ego_state, ego_lane, paths,best_index,x_vec,min_collision,walkers_y,walkers_x,mid_path_len,intersection,triangle_points,junc_bx_pts)
            
            local_waypoints = self._lp._velocity_planner.decelerate_profile(paths[best_index],current_speed,min_collision)

            red_light = self.is_light_red_or_no_light(self._traffic_lights,ego_location,ego_waypoint,goal_waypoint,ego_state)  
            if(type(red_light) == type("str")):
                red_light = True       
            self._color_light_state = red_light

            need_to_stop = self.need_to_stop(closest_vehicle,self._collission_index,ego_location,ego_waypoint,goal_waypoint,ego_state,min_collision,min_collision_actor,intersection)
            self._need_to_stop = need_to_stop

            lane_path_blcked = self.lane_paths_blocked(self._best_index_from_decelerate)
            
            if (DEBUG_STATE_MACHINE):
                print("{:<20} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25}".format(   "Pedestrian={}".format(walker_collide), \
                                                                                                    "LanePathBlocked={}".format(lane_path_blcked), \
                                                                                                    "IfEgoSpeedLess={}".format(self.is_speed_less()), \
                                                                                                    "DoWeNeedToStop={}".format(self._need_to_stop), \
                                                                                                    "IsIntersecAndNoSig={}".format((self._intersection_state and red_light)), \
                                                                                                    "{}".format('-'*20), \
                                                                                                    "{}".format('-'*20), \
                                                                                                    "{}".format(dict_[self._previous_state])  ))

            if (emergency_min_collision!=1):
                self._state   = EMERGENCY_STOP
            
            elif(self.is_speed_less()):
                self._state = STAY_STOPPED

            elif(walker_collide):
                self._collission_actor = col_walker
                self._state = DECELERATE_TO_STOP
                self._collission_index = min_collision

            elif(lane_path_blcked): 
                if((not need_to_stop) and (closest_vehicle!=None)) :
                    self._collission_actor = closest_vehicle
                    self._state = FOLLOW_LEAD_VEHICLE
                    self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
                    self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
                    self._goal_state[2] = 0

                else:
                    self._prev_lookahead = self._lookahead
                    self._collission_actor = closest_vehicle
                    self._state = DECELERATE_TO_STOP
                    self._collission_index = min_collision

            elif(self._intersection_state and red_light):
                self._prev_lookahead = self._lookahead
                self._state = DECELERATE_TO_STOP
                self._collission_index = min_collision
                self._collission_actor = closest_vehicle

            else:
                self._state = FOLLOW_LANE
                self._collission_actor = closest_vehicle
                self._collission_index = min_collision

            if(not self._started_decel):
                self._paths = paths
                self._started_decel = True

            return local_waypoints

        elif (self._state   == STAY_STOPPED):
            self._previous_state = self._state

            closest_len, closest_index = self.get_closest_index(ego_state)
            goal_index = self.get_goal_index(ego_state, closest_len, closest_index)
            self._goal_index = goal_index
            self._goal_state = self._waypoints[goal_index]
            self.num_layers = (goal_index - closest_index)//5

            goal_location = carla.Location(x = self._goal_state[0], y = self._goal_state[1], z= Z )
            self._world.debug.draw_string(goal_location, 'X', draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=100,persistent_lines=True)
            goal_waypoint = self._map.get_waypoint(goal_location,project_to_road=True)

            if goal_index < (self._waypoints.shape[0]-1):
                point_1 = self._waypoints[goal_index+1]
                point_2 = self._waypoints[goal_index-1]

            else: 
                point_1 = self._waypoints[goal_index]
                point_2 = self._waypoints[goal_index-2]

            goal_state_set = self._lp.get_goal_state_set(point_1,point_2,self._goal_state, ego_state,FOLLOW_LANE_OFFSET)

            paths, path_validity,mid_path_len = self._lp.plan_paths(goal_state_set)
            paths = local_planner.transform_paths(paths, ego_state)
            
            collision_check_array,min_collision, min_collision_actor = self._lp._collision_checker.collision_check_static(paths, obstacle_actors,self._world)
            emergency_collision_check_array,emergency_min_collision,emg_min_collision_actor = self._lp._collision_checker.collision_check_static(emergency_array, all_obstacle_actors,self._world)

            best_index = self._lp._collision_checker.select_best_path_index(paths, collision_check_array, self._goal_state,self._waypoints,ego_state)

            intersection,triangle_points,junc_bx_pts = self.is_approaching_intersection(self._waypoints,closest_index,ego_state, ego_waypoint)
            self._intersection_state = intersection

            walker_collide,col_walker,min_collision = self.check_walkers(self._map,walkers,ego_state, ego_lane, paths,best_index,x_vec,min_collision,walkers_y,walkers_x,mid_path_len,intersection,triangle_points,junc_bx_pts)
            
            red_light = self.is_light_red_or_no_light(self._traffic_lights,ego_location,ego_waypoint,goal_waypoint,ego_state)
            self._color_light_state = red_light
            
            need_to_stop= self.need_to_stop(closest_vehicle,closest_index,ego_location,ego_waypoint,goal_waypoint,ego_state,min_collision,min_collision_actor,intersection)
            self._need_to_stop = need_to_stop
            
            lane_path_blcked = self.lane_paths_blocked(best_index)

            if (DEBUG_STATE_MACHINE):
                print("{:<20} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25}".format(   "Pedestrian={}".format(walker_collide), \
                                                                                                    "TraficLight={}".format((red_light == "NTL")), \
                                                                                                    "IntersecAndSignal={}".format((self._intersection_state and red_light)), \
                                                                                                    "DoWeNeedToStop={}".format(self._need_to_stop), \
                                                                                                    "IsIntersecAndNoSig={}".format((self._intersection_state and red_light)), \
                                                                                                    "LanePathBlocked={}".format(lane_path_blcked), \
                                                                                                    "{}".format('-'*20), \
                                                                                                    "{}".format(dict_[self._previous_state])  ))
            
            if (emergency_min_collision!=1):
                self._state   = STAY_STOPPED

            elif(walker_collide):
                self._collission_actor = col_walker
                self._state   = STAY_STOPPED
                self._collission_index = min_collision

            elif(red_light == "NTL"):
                #time.sleep(2.0)
                c=0
                for i in range(1000000):          # this is to wait in intersections
                    c=c+i
                print("Delay")
                self._stopped = True
                self._state   = INTERSECTION
                
            elif((self._intersection_state and red_light)):             
                self._collission_actor = col_walker
                self._state   = STAY_STOPPED
                self._collission_index = min_collision

            
            elif(lane_path_blcked):
                if((not need_to_stop) and (closest_vehicle!=None)) :
                    self._collission_actor = closest_vehicle
                    self._state   = FOLLOW_LEAD_VEHICLE
                    self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
                    self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
                    self._goal_state[2] = 0

                else:
                    self._collission_actor = closest_vehicle
                    self._state   = STAY_STOPPED
                    self._collission_index = min_collision

            elif(self._intersection_state):
                self._state   = INTERSECTION
                self._collission_actor = closest_vehicle
                self._collission_index = min_collision

            else:
                self._state   = FOLLOW_LANE
                self._collission_actor = closest_vehicle
                self._collission_index = min_collision

            if(best_index == None):
                best_index = self._lp._num_paths//2
            
            best_path = paths[best_index]
            self._paths = paths
            debug_print(paths,self._world,best_index)

            local_waypoints = self._lp._velocity_planner.stop_profile(best_path)
            return local_waypoints
        
        elif(self._state == INTERSECTION):
            self._previous_state = self._state

            closest_len, closest_index = self.get_closest_index(ego_state)
            goal_index = self.get_goal_index(ego_state, closest_len, closest_index)
            self._goal_index = goal_index
            self._goal_state = self._waypoints[goal_index]
            self.num_layers = (goal_index - closest_index)//5
            self._goal_state[2] = SPEED

            goal_location = carla.Location(x=self._goal_state[0], y=self._goal_state[1], z= Z )
            goal_waypoint = self._map.get_waypoint(goal_location,project_to_road=True)

            if goal_index < (self._waypoints.shape[0]-1):
                point_1 = self._waypoints[goal_index+1]
                point_2 = self._waypoints[goal_index-1]

            else: 
                point_1 = self._waypoints[goal_index]
                point_2 = self._waypoints[goal_index-2]

            goal_state_set = self._lp.get_goal_state_set(point_1,point_2,self._goal_state, ego_state,FOLLOW_LANE_OFFSET)

            paths, path_validity,mid_path_len = self._lp.plan_paths(goal_state_set)
            paths = local_planner.transform_paths(paths, ego_state)
            
            collision_check_array,min_collision, min_collision_actor = self._lp._collision_checker.collision_check_static(paths, obstacle_actors,self._world)
            emergency_collision_check_array,emergency_min_collision, emg_min_collision_actor = self._lp._collision_checker.collision_check_static(emergency_array, all_obstacle_actors,self._world)

            best_index = self._lp._collision_checker.select_best_path_index(paths, collision_check_array, self._goal_state,self._waypoints,ego_state)

            intersection,triangle_points,junc_bx_pts = self.is_approaching_intersection(self._waypoints,closest_index,ego_state, ego_waypoint)
            self._intersection_state = intersection

            walker_collide,col_walker,min_collision = self.check_walkers(self._map,walkers,ego_state, ego_lane, paths,best_index,x_vec,min_collision,walkers_y,walkers_x,mid_path_len,intersection,triangle_points, junc_bx_pts)
    
            red_light = self.is_light_red_or_no_light(self._traffic_lights,ego_location,ego_waypoint,goal_waypoint,ego_state)
            self._color_light_state = red_light

            need_to_stop= self.need_to_stop(closest_vehicle,closest_index,ego_location,ego_waypoint,goal_waypoint,ego_state,min_collision,min_collision_actor, intersection)
            self._need_to_stop = need_to_stop
            
            if(type(red_light) == type("str")):
                red_light = True
            
            lane_path_blcked = self.lane_paths_blocked(best_index)

            if (DEBUG_STATE_MACHINE):
                print("{:<20} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25}".format(   "Pedestrian={}".format(walker_collide), \
                                                                                                    "LanePathBlocked={}".format(lane_path_blcked), \
                                                                                                    "DoWeNeedToStop={}".format(self._need_to_stop), \
                                                                                                    "IsIntersection={}".format(self._intersection_state), \
                                                                                                    "Stopped={}".format(self._stopped), \
                                                                                                    "ColorLight={}".format(red_light), \
                                                                                                    "{}".format('-'*20), \
                                                                                                    "{}".format(dict_[self._previous_state])  ))
    
            if (emergency_min_collision!=1):
                self._state   = EMERGENCY_STOP

            elif(walker_collide):
                self._prev_lookahead = self._lookahead
                self._collission_actor = col_walker
                self._state   = DECELERATE_TO_STOP
                self._collission_index = min_collision

            elif(lane_path_blcked):   
                if((not need_to_stop) and (closest_vehicle!=None)) :
                    self._collission_actor = closest_vehicle
                    self._state   = FOLLOW_LEAD_VEHICLE
                    self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
                    self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
                    self._goal_state[2] = 0

                else:
                    self._prev_lookahead = self._lookahead
                    self._collission_actor = closest_vehicle
                    self._state   = DECELERATE_TO_STOP
                    self._collission_index = min_collision
            
            elif((not self._intersection_state) ):               
                self._state   = FOLLOW_LANE
                self._collission_actor = closest_vehicle
                self._collission_index = min_collision

            elif(self._stopped):
                self._state   = INTERSECTION
                self._collission_actor = closest_vehicle
                self._collission_index = min_collision

            elif(red_light): 
                self._prev_lookahead = self._lookahead
                ###change the goal point
                self._state   = DECELERATE_TO_STOP
                self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
                self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
                self._goal_state[2] = 0
                
            else:
                self._state = INTERSECTION
                self._collission_actor = closest_vehicle
                self._collission_index = min_collision

            if(best_index == None):
                best_index = self._lp._num_paths//2
            
            best_path = paths[best_index]
            self._paths = paths
            debug_print(paths,self._world,best_index)

            local_waypoints = self._lp._velocity_planner.nominal_profile(best_path, open_loop_speed, self._goal_state[2])
            return local_waypoints

        elif(self._state == OVERTAKE):
            self._isOvertake = 1
            self._overtakeLookahead = 0
            self._previous_state = self._state

            # Find the closest index to the ego vehicle.
            closest_len, closest_index = self.get_closest_index(ego_state)
            # Find the goal index that lies within the lookahead distance along the waypoints.
            goal_index = self.get_goal_index(ego_state, closest_len, closest_index)
            self._goal_index = goal_index
            # Set the goal state by getting the location and giving derired speed
            self._goal_state = self._waypoints[goal_index]
            self._goal_state[2] = SPEED + 3 #get_speed(self._overtakeVeh)

            self.num_layers = (goal_index - closest_index)//5
        
            goal_location = carla.Location(x=self._goal_state[0], y=self._goal_state[1], z= Z )
            goal_waypoint = self._map.get_waypoint(goal_location,project_to_road=True)

            if goal_index < (self._waypoints.shape[0]-1):
                point_1 = self._waypoints[goal_index+1]
                point_2 = self._waypoints[goal_index-1]

            else: 
                point_1 = self._waypoints[goal_index]
                point_2 = self._waypoints[goal_index-2]

            goal_state_set = self._lp.get_goal_state_set(point_1,point_2,self._goal_state, ego_state,0.8)

            # Calculate planned paths in the local frame.
            paths, path_validity,mid_path_len = self._lp.plan_paths(goal_state_set)

            # Transform those paths back to the global frame.
            paths = local_planner.transform_paths(paths, ego_state)
            
            ego_to_veh_vector = np.array([self._overtakeVeh.get_transform().location.x,self._overtakeVeh.get_transform().location.y]) - np.array(ego_state[:2])
            y_dist = np.dot(y_vec,ego_to_veh_vector)
            
            if y_dist<-(3.5+self._ego_vehicle.bounding_box.extent.y+self._overtakeVeh.bounding_box.extent.y):
                paths = paths[2:9]
                self._overtakeLookahead = 0
                closest_dist, _ = self.get_closest_index(ego_state)
                if closest_dist<0.2:
                    self._overtakeVeh = None
                    self._isOvertake = 0
                    self._state   = FOLLOW_LANE
            else:
                paths = paths[:3]
            
            

            # Check collisions for every path
            collision_check_array,min_collision, min_collision_actor = self._lp._collision_checker.collision_check_static(paths, all_obstacle_actors,self._world)
            emergency_collision_check_array,emergency_min_collision,emg_min_collision_actor = self._lp._collision_checker.collision_check_static(emergency_array, all_obstacle_actors,self._world)

            # Calculate the index of the best feasible path
            best_index = self._lp._collision_checker.select_best_path_index(paths, collision_check_array, self._goal_state,self._waypoints,ego_state)

            # # Check whether the ego vehicle is approaching an intersection
            # intersection,triangle_points,junc_bx_pts = self.is_approaching_intersection(self._waypoints, closest_index, ego_state, ego_waypoint)
            # self._intersection_state = intersection

            # # Check for walkers
            # walker_collide,col_walker,min_collision = self.check_walkers(self._map, walkers, ego_state, ego_lane, paths, best_index,\
            #                                             x_vec, min_collision, walkers_y, walkers_x, mid_path_len, intersection, triangle_points, junc_bx_pts)
            
            #  # Check whether the ego vehicle needs to stop
            # need_to_stop= self.need_to_stop(closest_vehicle,closest_index,ego_location,ego_waypoint,goal_waypoint,ego_state,min_collision,min_collision_actor,intersection)
            # self._need_to_stop = need_to_stop    
            
            # # Check all paths of the ego vehicle is blocked or not
            # lane_path_blcked = self.lane_paths_blocked(best_index)

            # if (DEBUG_STATE_MACHINE):
            #     print("{:<20} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25}".format(   "Pedestrian={}".format(walker_collide), \
            #                                                                                         "LanePathBlock={}".format(lane_path_blcked), \
            #                                                                                         "CanOvertake={}".format(self.can_overtake(ego_state,vehicles_static, vehicles_dynamic, walkers, closest_vehicle, x_vec, y_vec, walkers_y, walkers_x,current_speed)), \
            #                                                                                         "DoWeNeedToStop={}".format(self._need_to_stop), \
            #                                                                                         "IsIntersection={}".format(self._intersection_state), \
            #                                                                                         "{}".format('-'*20), \
            #                                                                                         "{}".format('-'*20), \
            #                                                                                         "{}".format(dict_[self._previous_state])  ))
           
            # if (emergency_min_collision!=1):
            #     self._state   = EMERGENCY_STOP
                   
            # elif(walker_collide):
            #     self._collission_actor = col_walker
            #     self._state   = DECELERATE_TO_STOP
            #     self._collission_index = min_collision
 
            # elif(lane_path_blcked):  
            #     if(self.can_overtake(ego_state,vehicles_static, vehicles_dynamic, walkers, closest_vehicle, x_vec, y_vec, walkers_y, walkers_x,current_speed)):
            #         ##### FIX GOAL STATE PROPERLY
            #         self._state   = OVERTAKE
            #         self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
            #         self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
            #         self._goal_state[2] = 0

            #     elif((not need_to_stop) and (closest_vehicle!=None)) :
            #         self._collission_actor = closest_vehicle
            #         self._state   = FOLLOW_LEAD_VEHICLE
            #         self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
            #         self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
            #         self._goal_state[2] = 0
                
            #     else:
            #         self._prev_lookahead = self._lookahead
            #         self._collission_actor = closest_vehicle
            #         self._state   = DECELERATE_TO_STOP
            #         self._collission_index = min_collision
                    
            # elif(self._intersection_state):
            #     self._state   = INTERSECTION
            #     self._collission_actor = closest_vehicle
            #     self._collission_index = min_collision
            
            # else:
            #     self._state   = FOLLOW_LANE
            #     self._collission_actor = closest_vehicle
            #     self._collission_index = min_collision

            if(best_index == None):
                best_index = 0 #self._lp._num_paths//2
            
            best_path = paths[best_index]
            self._paths = paths
            debug_print(paths,self._world,best_index)

            local_waypoints = self._lp._velocity_planner.nominal_profile(best_path, open_loop_speed, self._goal_state[2])
            return local_waypoints
        
        elif(self._state == FOLLOW_LEAD_VEHICLE):
            self._previous_state = self._state

            closest_len, closest_index = self.get_closest_index(ego_state)
            goal_index = self.get_goal_index(ego_state, closest_len, closest_index)
            self._goal_index = goal_index
            self._goal_state = self._waypoints[goal_index]
            self.num_layers = (goal_index - closest_index)//5

            goal_location = carla.Location(x=self._goal_state[0], y=self._goal_state[1], z= Z )
            goal_waypoint = self._map.get_waypoint(goal_location,project_to_road=True)

            if goal_index < (self._waypoints.shape[0]-1):
                point_1 = self._waypoints[goal_index+1]
                point_2 = self._waypoints[goal_index-1]

            else: 
                point_1 = self._waypoints[goal_index]
                point_2 = self._waypoints[goal_index-2]

            goal_state_set = self._lp.get_goal_state_set(point_1,point_2,self._goal_state, ego_state,FOLLOW_LANE_OFFSET)

            paths, path_validity,mid_path_len = self._lp.plan_paths(goal_state_set)
            paths = local_planner.transform_paths(paths, ego_state)
            
            collision_check_array,min_collision, min_collision_actor = self._lp._collision_checker.collision_check_static(paths, obstacle_actors,self._world)
            emergency_collision_check_array,emergency_min_collision, emg_min_collision_actor = self._lp._collision_checker.collision_check_static(emergency_array, all_obstacle_actors,self._world)
          
            best_index = self._lp._collision_checker.select_best_path_index(paths, collision_check_array, self._goal_state,self._waypoints,ego_state)

            intersection,triangle_points, junc_bx_pts = self.is_approaching_intersection(self._waypoints,closest_index,ego_state, ego_waypoint)
            self._intersection_state = intersection

            walker_collide,col_walker,min_collision = self.check_walkers(self._map,walkers, ego_state, ego_lane, paths,best_index,x_vec,min_collision,walkers_y,walkers_x,mid_path_len,intersection,triangle_points, junc_bx_pts)

            need_to_stop= self.need_to_stop(closest_vehicle,closest_index,ego_location,ego_waypoint,goal_waypoint,ego_state,min_collision,min_collision_actor, intersection)
            self._need_to_stop = need_to_stop
            
            lane_path_blcked = self.lane_paths_blocked(best_index)

            if (DEBUG_STATE_MACHINE):
                print("{:<20} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25} | {:<25}".format(   "Pedestrian={}".format(walker_collide), \
                                                                                                    "LanePathBlocked={}".format(lane_path_blcked), \
                                                                                                    "CanOvertake={}".format(self.can_overtake(ego_state,vehicles_static, vehicles_dynamic, walkers, closest_vehicle, x_vec, y_vec, walkers_y, walkers_x,current_speed)), \
                                                                                                    "DoWeNeedToStop={}".format(self._need_to_stop), \
                                                                                                    "IsIntersection={}".format(self._intersection_state), \
                                                                                                    "{}".format('-'*20), \
                                                                                                    "{}".format('-'*20), \
                                                                                                    "{}".format(dict_[self._previous_state])  ))

            if (emergency_min_collision!=1):
                self._state   = EMERGENCY_STOP

            elif(walker_collide):
                self._prev_lookahead = self._lookahead
                self._collission_actor = col_walker
                self._state   = DECELERATE_TO_STOP
                self._collission_index = min_collision
   
            elif(lane_path_blcked): 
                if(self.can_overtake(ego_state,vehicles_static, vehicles_dynamic, walkers, closest_vehicle, x_vec, y_vec, walkers_y, walkers_x,current_speed)):
                    ##### FIX GOAL STATE PROPERLY
                    self._state   = OVERTAKE
                    self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
                    self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
                    self._goal_state[2] = 0
                
                elif((not need_to_stop) and (closest_vehicle!=None)) :
                    self._collission_actor = closest_vehicle
                    self._state   = FOLLOW_LEAD_VEHICLE
                    self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
                    self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
                    self._goal_state[2] = 0

                else:
                    self._prev_lookahead = self._lookahead
                    self._collission_actor = closest_vehicle
                    self._state   = DECELERATE_TO_STOP
                    self._collission_index = min_collision

            elif(self._intersection_state):
                self._state   = INTERSECTION
                self._collission_actor = closest_vehicle
                self._collission_index = min_collision
            
            else:
                self._state   = FOLLOW_LANE
                self._collission_actor = closest_vehicle
                self._collission_index = min_collision

            if(self._collission_actor == None):
                    
                if(best_index == None):
                    best_index = self._lp._num_paths//2
                
                best_path = paths[best_index]
                self._paths = paths            
                debug_print(paths,self._world,best_index)

                local_waypoints = self._lp._velocity_planner.nominal_profile(best_path, open_loop_speed, self._goal_state[2])


            else:
                if(best_index == None):
                    best_index = self._lp._num_paths//2
                
                lead_loc = self._collission_actor.get_location()
                lead_vehicle_state = [lead_loc.x,lead_loc.y,misc.get_speed(self._collission_actor)]

                best_path = paths[best_index]
                self._paths = paths
                debug_print(paths,self._world,best_index)

                local_waypoints = self._lp._velocity_planner.follow_profile(best_path, open_loop_speed, self._goal_state[2],lead_vehicle_state)

            return local_waypoints
        
        elif(self._state == EMERGENCY_STOP):
            self._state = STAY_STOPPED


        







######################################################
######################################################
######################################################
######################################################
######################################################
######################################################
######################################################
######################################################
##############   LOCAL FUNCTIONS  ####################
######################################################
######################################################
######################################################
######################################################
######################################################
######################################################
######################################################
######################################################
######################################################





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

    def get_offset(self,points):
        is_True_1 = False
        is_True_2 = False
        theta = np.arctan2(points[1][1] - points[0][1],points[1][0] - points[0][0])   
        
        for radius in range(3,16):
            # print(radius)
            temp_1 = points[0]+0.5*np.array([np.cos(theta),np.sin(theta)])
            temp_2 = points[1]-0.5*np.array([np.cos(theta),np.sin(theta)])
            
            # print(temp_1,temp_2)
            wayp_1 = self._map.get_waypoint(carla.Location(x = temp_1[0],y = temp_1[1],z = 0),project_to_road = True,lane_type = (carla.LaneType.Driving|carla.LaneType.Sidewalk))
            wayp_2 = self._map.get_waypoint(carla.Location(x = temp_2[0],y = temp_2[1],z = 0),project_to_road = True,lane_type = (carla.LaneType.Driving|carla.LaneType.Sidewalk))

            
            if(wayp_1.lane_type == carla.LaneType.Driving):
                is_True_1 = True

            else:
                points[0] += 0.5*np.array([np.cos(theta),np.sin(theta)])

                # break

            if(wayp_2.lane_type == carla.LaneType.Driving):
                is_True_2 = True
                
            else:
                points[1] -= 0.5*np.array([np.cos(theta),np.sin(theta)])
                # break

            if(is_True_1 and is_True_2):
                break

        return points
                
    def within_polygon(self,points,location):
        within = False
        sign = None
        for i in range(-1,points.shape[0]-1,1):
            cross = np.cross(points[i+1]-points[i],location-points[i])
            # print(cross)
            if sign == None:
                if(cross>=0):
                    sign = 1
                else:
                    sign = -1

            else:
                if(sign*cross>=0):
                    continue
                else:
                    break
        else:
            within = True

        return(within)

    def get_shape(self,check_wayp_idx,inter_points,ego_state):

        idx_heading_check = int(HEADING_CHECK_LOOKAHEAD/self._hop_resolution)
        n_ = self._waypoints.shape[0]
        
        if(idx_heading_check+check_wayp_idx>n_-1):
            checking_waypoint = self._waypoints[n_-2]

            heading = np.arctan2(self._waypoints[n_-1][1]-self._waypoints[n_-3][1],self._waypoints[n_-1][0]-self._waypoints[n_-3][0])

        else:    
            checking_waypoint = self._waypoints[check_wayp_idx+idx_heading_check]
            heading = np.arctan2(self._waypoints[check_wayp_idx+idx_heading_check+1][1]-self._waypoints[check_wayp_idx+idx_heading_check-1][1],self._waypoints[check_wayp_idx+idx_heading_check+1][0]-self._waypoints[check_wayp_idx+idx_heading_check-1][0])

        # heading = np.degrees(heading)
        ego_heading = np.arctan2(self._waypoints[check_wayp_idx+1][1] -self._waypoints[check_wayp_idx-1][1],self._waypoints[check_wayp_idx+1][0] -self._waypoints[check_wayp_idx-1][0] )
        heading_check = ego_heading - heading

        if heading_check > np.pi:
            heading_check -= 2*np.pi
        elif heading_check < -np.pi:
            heading_check += 2*np.pi

        # print(check_wayp_idx,idx_heading_check,ego_heading,heading)
        dist = np.sum(np.square(inter_points-ego_state[:2]),axis =1)
        #argpartition gives the indices when sorted
        closest_points = np.sort(np.argpartition(dist, 2)[:2])

        if(np.abs(heading_check)<np.radians(JUNCTION_HEADING_CHECK_ANGLE)):

            # print("A")
            if(np.all(closest_points==np.array([0,3]))):
                set_1 = inter_points[np.array([3,0])]
                set_2 = inter_points[np.array([1,2])]
            else:
                set_1 = inter_points[closest_points]
                set_2 = inter_points[np.array([(closest_points[1]+1)%4,(closest_points[1]+2)%4])]

            # print(set_1,set_2)
            set_1 = self.get_offset(set_1)
            set_2 = self.get_offset(set_2)

            set_1 = np.append(set_1,set_2,axis = 0)

            for i in range(set_1.shape[0]):
                # print(inter_junc_points[i])
                self._world.debug.draw_string(carla.Location(x=set_1[i,0],y = set_1[i,1],z = 1),"A", draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)

            return set_1

            # print("STRAIGHT")
            # pass
        elif(heading_check>0):
            
            # print("B",closest_points)
            
            if(closest_points[0] ==0 and closest_points[1] ==3 ):
                closest_points = inter_points[np.append(closest_points[:2],[(closest_points[0]+1)%4])]

            else:
                closest_points = inter_points[np.append(closest_points[:2],[(closest_points[1]+1)%4])]

            for i in range(closest_points.shape[0]):
                # print(inter_junc_points[i])
                self._world.debug.draw_string(carla.Location(x=closest_points[i,0],y = closest_points[i,1],z = 1),"A", draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)
            # print(closest_points,inter_points,dist)
            return closest_points
            # print("LEFT")
            # pass
        else:
            
            # print("C")

            if(closest_points[0] ==0 and closest_points[1] ==3 ):
                closest_points = inter_points[np.append(closest_points[:2],[(closest_points[1]-1)])]
            else:
                closest_points = inter_points[np.append(closest_points[:2],[closest_points[0]-1])]

            for i in range(closest_points.shape[0]):
                # print(inter_junc_points[i])
                self._world.debug.draw_string(carla.Location(x=closest_points[i,0],y = closest_points[i,1],z = 1),"A", draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)

            return closest_points
            # print("RIGHT")
            # pass


    def is_approaching_intersection(self, waypoints, closest_index,ego_state,ego_waypoint):
        """
        This method is specialized to check whether ego-vehicle is approaching an intersection

        param   : waypoints         : Numpy array containing global waypoints of the path of ego-vehicle
                                      Length and speed in m and m/s.
                                      format - numpy.ndarray[[x coordinate,y coordinate, ego-vehicle speed] - waypoint 0
                                                             [x coordinate,y coordinate, ego-vehicle speed] - waypoint 1
                                                                        :                   :
                                                                        :                   :
                                                             [x coordinate,y coordinate, ego-vehicle speed] - waypoint n-2
                                                             [x coordinate,y coordinate, ego-vehicle speed]] - waypoint n-1  
                : closest_index     : Index of the closest waypoint to the ego-vehicle
                : ego_state         : Current state of the ego-vehicle
                                      format [x,y,yaw]
                : ego_waypoint      : Nearest waypoint to the ego-vehicle projected to the middle of the lane
                                      format - Carla.Waypoint

        return  : bool              : Output True if ego-vehicle is approaching an intersection
                : triangle_points   : List of points of triangles that is taken for checking walkers in 
                                      an intersection
                : junc_bx_pts       : List of points of junction that is taken for checking walkers in 
                                      an intersection 
        """   
        # Get number of waypoints in  INTERSECTION_APPROACH_DISTANCE

        index_length_for_approaching = int(self._lookahead / self._hop_resolution)
        closest_len, closest_index = self.get_closest_index(ego_state)

        # Make last index of waypoint as the idx if the calculated 
        # number of waypoints + closest index is higher than the number of waypoints 
        if (index_length_for_approaching + closest_index > self._waypoints.shape[0]-1):
            checking_waypoint = waypoints[self._waypoints.shape[0]-1]
            idx = self._waypoints.shape[0]-1

        else:    
            checking_waypoint = waypoints[closest_index+index_length_for_approaching]
            idx = closest_index+index_length_for_approaching

        exwp = self._map.get_waypoint(carla.Location( x= checking_waypoint[0], y=checking_waypoint[1], z= Z))
        out = exwp.is_junction
        intersection = False
        
        # Check the calculated approaching index is in the junction
        if (out):
            self._not_passed = True
            intersection = True

        elif (ego_waypoint.is_junction):    # Check the ego-vehicle is in the junction
            self._not_passed = False        # Make self._not_passed False when the vehicle is in the junction but the approaching waypoint is out
            intersection = True

        elif (self._not_passed):
            intersection = True

        else:
            self._first_time = False
            intersection = False
            self._stopped = False

        if(intersection and not self._first_time):
            if not UNSTRUCTURED:
                # Finding the triangle points to check walker in intersections
                junc_points = misc.print_junction(self._world,exwp) 
                lines = misc.get_line(junc_points)
                inter_junc_points = misc.solve_lines(lines)
                box_points = misc.get_box(self._map,inter_junc_points)
                self._junc_bx_pts = box_points
                self._triangle_points = self.get_shape(idx,box_points,ego_state)

                for i in range(box_points.shape[0]):
                    self._world.debug.draw_string(carla.Location(x=box_points[i,0],y = box_points[i,1],z = 1),"A", draw_shadow=False,color=carla.Color(r=255, g=255, b=0), life_time=10000,persistent_lines=True)
                    
            else:
                self._box_points = None
                self._junc_bx_pts = None

            self._first_time = True
            return intersection,self._triangle_points,self._junc_bx_pts

        else:
            if(not intersection):
                self._box_points = None
                self._junc_bx_pts = None
            return intersection,self._box_points,  self._junc_bx_pts

    def lane_paths_blocked(self,best_index):

        if(self._collission_actor != None):
            #print("from 1")
            dist_closest = np.sqrt(np.sum(np.square(np.array([self._collission_actor.get_location().x,self._collission_actor.get_location().y]) \
                             - np.array([self._ego_vehicle.get_location().x,self._ego_vehicle.get_location().y]))))

            if(dist_closest < FOLLOW_LEAD_RANGE):# and dist_closest< self._lookahead):
                return True
            # elif (best_index == None):
            #         print("Lane path debug is working congratulations!!!!!")
            #         return True
            else:
                return False         

        elif(best_index == None):
            return True
        else:
            return False

    def is_speed_less(self):
        """
        This method is specialized to check whether the ego vehicle speed is less than a threshold.
        return  : bool  : True is the ego vehicle speed is less than the threshold and False 
                          otherwise
        """

        velocity = self._ego_vehicle.get_velocity()
        if(np.linalg.norm(np.array([velocity.x,velocity.y,velocity.z]))<self.stop_threshold):
            return True
        else:
            return False

    def check_walkers(self,world_map,walkers,ego_state,ego_lane,paths,best_index,vec_rd,min_collision,walkers_y,walkers_x,mid_path_len,intersection,triangle_points,junc_bx_pts):
        """
        This method is specialized to check whether is there any walkers in front of 
        the ego vehicle to a distance of WALKER_DIST_RANGE and within a width of LANE WIDTH

        param   : world_map             : CARLA world map
                : walkers               : List of walkers in front of the vehicle in a radius
                                          (Walkers list is arranged with the walkers from the lowest distance to highest
                                          distance to the direction of heading)
                : ego_state             : Current state of the ego-vehicle
                                          format [x,y,yaw]
                : ego_lane              : Current lane of ego-vehicle 
                                          format - integer
                : paths                 : List of paths containing x,y and v values for each path(Check local planner)
                : best_index            : Index of best feasible path 
                : vec_rd
                : min_collision         : Least local collision index from all the paths in paths array
                : walkers_y             : Distance to the walkers in the direction of heading
                : walkers_x             : Distance to the walkers in the direction perpendicular to the heading
                : mid_path_len          : Length of the middle path in list of paths
                : intersection          : Output True if the ego-vehicle is at an intersection False otherwise
                : triangle_points       : Points of the traingle affecting to the ego vehicle in intersections
                : junc_bx_pts           : Points of the junction that the ego vehicle is in
                
        return  : bool                  : Output True if walker is available and False otherwise
                : actor                 : Walker actor relevant to the collision
                : min_collision         : Least local collision index from all the paths in paths array
        """ 
         
        WALKER_DIST_RANGE = min(WALKER_DIST_RANGE_MAX,WALKER_DIST_RANGE_BASE + 2*self._open_loop_speed)
        
        if UNSTRUCTURED:     
            danger_walkers=[]
            # Number of waypoints to check in the range of WALKER_DIST_RANGE
            INDEX_AMOUNT = int(WALKER_DIST_RANGE/self._hop_resolution)
            closest_len, closest_index = self.get_closest_index(ego_state)
            Checking_waypoints = self._waypoints[closest_index:closest_index+INDEX_AMOUNT]

            counter = -1
            if walkers:
                for w in walkers:
                    counter+=1
                    walker_loc = w.get_location()
                    for i in range (len(Checking_waypoints)):
                        wayp = Checking_waypoints[i]
                        # Get distance to walkers
                        dist = np.sqrt(np.sum(np.square(np.array([walker_loc.x,walker_loc.y]) \
                                    - np.array([wayp[0],wayp[1]]))))
                        if (dist<LANE_WIDTH_WALKERS):
                            danger_walkers.append([w,closest_index+i,counter])
                            break

            # Sorting according to the ascending order of indices
            danger_walkers.sort(key=lambda x:x[1])

            for d in danger_walkers:
                walk = d[0]
                draw_bound_box_actor(walk,self._world,255,255,255)

            if(len(danger_walkers)==0):
                return False,None,min_collision

            else:
                # if the collision index of walker is closer than min collision replace min collision with new collision index
                if(walkers_y[danger_walkers[0][2]]<(mid_path_len/49)*(min_collision+1)):
                    min_collision = int(walkers_y[danger_walkers[0][2]]//(mid_path_len/49))
                return True,danger_walkers[0][0],min_collision  

        else:
            if(best_index == None):
                best_index = self._lp._num_paths//2
            best_path = paths[best_index]

            dest_waypoint = world_map.get_waypoint(carla.Location(x=best_path[0][-1], y=best_path[1][-1], z= Z ),project_to_road = True, lane_type =carla.LaneType.Driving)
            dest_lane = dest_waypoint.lane_id
            counter = -1
            if walkers:
                for w in walkers:
                    walker_loc= w.get_location()
                    walker_waypoint=world_map.get_waypoint(carla.Location(x=walker_loc.x, y=walker_loc.y, z= walker_loc.z ),project_to_road=True,lane_type = ( carla.LaneType.Driving | carla.LaneType.Sidewalk ))
                    w_lane = walker_waypoint.lane_id
                    dist_walker = np.sqrt(np.sum(np.square(np.array([walker_loc.x,walker_loc.y]) - np.array([ego_state[0],ego_state[1]]))))
                    if (intersection and self.within_polygon(triangle_points,np.array([walker_loc.x,walker_loc.y])) and walker_waypoint.lane_type == carla.LaneType.Driving ):
                        draw_bound_box_actor(w,self._world,0,255,0)
                    
                    elif not intersection:
                        if(ego_lane==w_lane):
                            draw_bound_box_actor(w,self._world,0,255,0)

                for person in walkers:
                    counter+=1
                    walker_loc= person.get_location()

                    # Get distance to walkers
                    dist_walker = np.sqrt(np.sum(np.square(np.array([walker_loc.x,walker_loc.y]) - np.array([ego_state[0],ego_state[1]]))))
                    
                    # Get lane of the walker and lanes two points perpendicular to the heading direction in 1 m from walker
                    walker_waypoint=world_map.get_waypoint(carla.Location(x=walker_loc.x, y=walker_loc.y, z= walker_loc.z ),project_to_road=True,lane_type = ( carla.LaneType.Driving | carla.LaneType.Sidewalk ))
                    w_lane = walker_waypoint.lane_id
                    walker_pt1 = np.array([walker_loc.x,walker_loc.y]) + vec_rd
                    walker_pt2 = np.array([walker_loc.x,walker_loc.y]) - vec_rd
                    walker_way_pt1=world_map.get_waypoint(carla.Location(x=walker_pt1[0], y=walker_pt1[1], z= walker_loc.z ),project_to_road=True, lane_type = ( carla.LaneType.Driving | carla.LaneType.Sidewalk ))
                    walker_way_pt2=world_map.get_waypoint(carla.Location(x=walker_pt2[0], y=walker_pt2[1], z= walker_loc.z),project_to_road=True, lane_type = ( carla.LaneType.Driving | carla.LaneType.Sidewalk ))
                    w_pt1_lane = walker_way_pt1.lane_id
                    w_pt2_lane = walker_way_pt2.lane_id
                    w_speed = person.get_control().speed
                 
                    if (intersection and (dist_walker<DIST_WALKER_INTERSECTION)):
                        # Check walker is within the polygon or not
                        in_polygon = self.within_polygon(triangle_points,np.array([walker_loc.x,walker_loc.y]))
                        # Check walker is within the junction or not
                        in_junction = self.within_polygon(junc_bx_pts,np.array([walker_loc.x,walker_loc.y]))
                        # Check goal point is within the polygon or not
                        dest_in_polygon = self.within_polygon(triangle_points,np.array([best_path[0][-1],best_path[1][-1]]))

                        if (in_polygon and walker_waypoint.lane_type == carla.LaneType.Driving):
                            # print("AA")
                            if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                min_collision = int(walkers_y[counter]//(mid_path_len/49))
                            return True,person,min_collision
                        # When the goal point and walker both are not within junction check lanes 
                        elif ((not in_junction) and (not dest_in_polygon) ):    
                            # print("not in junction")
                            if (dest_lane==w_lane):
                                # print("EE")                        
                                if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                        min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                return True,person,min_collision   
                        
                            elif (dest_lane-1==0):
                                # print("in double line 1")
                                if ((w_lane==dest_lane-2) or (w_lane==dest_lane-1) or (w_lane==dest_lane+1)):
                                    if (((dest_lane==w_pt1_lane) or (dest_lane==w_pt2_lane)) and (w_speed>WALKER_THRESHOLD)):
                                        print("FF")                        
                                        if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                            min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                        return True,person,min_collision       
                                    else:
                                        # print("ABC")
                                        continue
                                else:
                                    # print("ABCD")
                                    continue                          

                            elif (dest_lane+1==0):
                                # print("in double line 2")
                                if ((w_lane==dest_lane+2) or (w_lane==dest_lane+1) or (w_lane==dest_lane-1)):
                                    if (((dest_lane==w_pt1_lane) or (dest_lane==w_pt2_lane)) and (w_speed>WALKER_THRESHOLD)):
                                        # print("GG")                        
                                        if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                            min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                        return True,person,min_collision       
                                    else:
                                        # print("ABCDE")
                                        continue
                                else:
                                    # print("ABCDEF")
                                    continue
                                                                   
                            elif ((w_lane==dest_lane-1) or (w_lane==dest_lane+1)):
                                # print("side lanes")
                                if (((dest_lane==w_pt1_lane) or (dest_lane==w_pt2_lane)) and (w_speed>WALKER_THRESHOLD)):
                                    # print("HH")                        
                                    if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                        min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                    return True,person,min_collision  
                                else:
                                    # print("ABBB")
                                    continue
                            else:
                                # print("ACCCC")
                                continue

                        else:
                            # print("intersection")
                            continue    

                        
                    else:
                        if (intersection):
                            # print("not in range intersection")
                            continue

                        else:
                            if (walkers_y[counter] < WALKER_DIST_RANGE):
                                if (ego_lane==w_lane):
                                    # print("A")
                                    if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                        min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                    return True,person,min_collision    

                                elif (ego_lane-1==0):
                                    if ((w_lane==ego_lane-2) or (w_lane==ego_lane-1)or (w_lane==ego_lane+1)):
                                        if (((ego_lane==w_pt1_lane) or (ego_lane==w_pt2_lane)) and (w_speed>WALKER_THRESHOLD)):
                                            if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                                min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                            return True,person,min_collision 

                                elif (ego_lane+1==0):
                                    if ((w_lane==ego_lane+2) or (w_lane==ego_lane+1) or (w_lane==ego_lane-1)):
                                        if (((ego_lane==w_pt1_lane) or (ego_lane==w_pt2_lane)) and (w_speed>WALKER_THRESHOLD)):
                                            # print("C")                        
                                            if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                                min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                            return True,person,min_collision   
                                        
                                elif ((w_lane==ego_lane-1) or (w_lane==ego_lane+1)):
                                    if (((ego_lane==w_pt1_lane) or (ego_lane==w_pt2_lane)) and (w_speed>WALKER_THRESHOLD)):
                                            # print("D")                        
                                            if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                                min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                            return True,person,min_collision       

                                if (dest_lane==w_lane):
                                    # print("E")                        
                                    if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                            min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                    return True,person,min_collision 

                                elif (dest_lane-1==0):
                                    if ((w_lane==dest_lane-2) or (w_lane==dest_lane-1) or (w_lane==dest_lane+1)):
                                        if (((dest_lane==w_pt1_lane) or (dest_lane==w_pt2_lane)) and (w_speed>WALKER_THRESHOLD)):
                                            print("F")                        
                                            if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                                min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                            return True,person,min_collision      

                                elif (dest_lane+1==0):
                                    if ((w_lane==dest_lane+2) or (w_lane==dest_lane+1) or (w_lane==dest_lane-1)):
                                        if (((dest_lane==w_pt1_lane) or (dest_lane==w_pt2_lane)) and (w_speed>WALKER_THRESHOLD)):
                                            # print("G")                        
                                            if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                                min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                            return True,person,min_collision     
                                        
                                elif ((w_lane==dest_lane-1) or (w_lane==dest_lane+1)):
                                    if (((dest_lane==w_pt1_lane) or (dest_lane==w_pt2_lane)) and (w_speed>WALKER_THRESHOLD)):
                                        # print("H")                        
                                        if(walkers_y[counter]<(mid_path_len/49)*(min_collision+1)):
                                            min_collision = int(walkers_y[counter]//(mid_path_len/49))

                                        return True,person,min_collision     
                            
                else:
                    # print("Y")   
                    return False,None,min_collision
            else:
                # print("Z")   
                return False,None,min_collision 

    def need_to_stop(self,lead_vehicle,closest_index,ego_location, ego_vehicle_waypoint,goal_waypoint,ego_state,min_collision,min_collision_actor,is_intersection):
        """
        This method is specialized to check whether we need to stop or not  

        param   : lead_vehicle           : list containing TrafficLight objects
                : closest_index
                : ego_location          : Current location of ego-vehicle 
                                          format - Carla.Location
                : ego_vehicle_waypoint  : Nearest waypoint to the ego-vehicle projected to the middle of the lane
                                          format - Carla.Waypoint
                : goal_waypoint         : Nearest waypoint to the goal state
                                          format - Carla.Waypoint
                : ego_state             : Current state of the ego-vehicle
                                          format [x,y,yaw]
                : min_collision         : Least local collision index from all the paths in paths array
                : min_collision_actor   : Actor corresponding to the min_collision
                : is_intersection       : Output True if the ego-vehicle is at an intersection False otherwise
                
        return  : bool                  : Output True if need to stop and False otherwise
        """    

        if(lead_vehicle!=None):
            # set to True if the closest colliding actor is not the lead vehicle
            if (lead_vehicle==min_collision_actor):
                lead_velocity = lead_vehicle.get_velocity()
                lead_velocity = np.array([lead_velocity.x,lead_velocity.y,lead_velocity.z])
                lead_speed = np.linalg.norm(lead_velocity)

                ego_velocity  = self._ego_vehicle.get_velocity()
                ego_velocity = np.array([ego_velocity.x,ego_velocity.y,ego_velocity.z])
                # angle between lead velocity and ego velocity(radians)             
                angle = np.arccos(np.dot(ego_velocity,lead_velocity)/(np.linalg.norm(ego_velocity)*lead_speed))
                
                if(lead_speed <LEAD_SPEED_THRESHOLD):
                    return True
                elif(angle>np.pi/2): 
                    # When the lead vehicle going across the path of ego vehicle
                    return True
                else:
                    return False
            else:
                return True

        elif(is_intersection):
            green_light = not self.is_light_red_or_no_light(self._traffic_lights,ego_location, ego_vehicle_waypoint,goal_waypoint ,ego_state)
            # When traffic lights present
            if(type(green_light)!= type("str")):
                if(green_light):
                    return False
                else:
                    return True
            else:
                # When already stopped at the stop line in intersection
                if(self._stopped):
                    return False
                else:
                    return True
        else:       
            return True

    def can_overtake(self, ego_state,vehicles_static, vehicles_dynamic, walkers, closest_vehicle, x_vec, y_vec, walkers_y, walkers_x,current_speed):
        """
        Should we overtake?
        """
        can_overtake_ = False
        # print("can overtake is called")
        # id there is no leading vehicle, no need to overtake
        # if the closest vehicle is not in the same lane of the ego vehicle, no need to overtaking
        # if current ego speed is greater than that of the leading vehicle, we need to overtake 
        # print(closest_vehicle)
        if (not(closest_vehicle is None)) and check_lane_closest(closest_vehicle, self._ego_vehicle, self._map) and (current_speed < SPEED):
            # if the ego vehicle on an intersection, zebra crossing, or the lane markings does not allow overtaking...
            # ... we should not overtake
            if can_we_overtake(self._ego_vehicle, vehicles_static, vehicles_dynamic, walkers, closest_vehicle, x_vec, y_vec, walkers_y, walkers_x, current_speed, self._map, self._world,self._waypoints):
                can_overtake_ = True
            ########
            # TODO #
            ########
            # The vehicles both static and dynamic are considered inside a radius, this should be changed 
            # such that only the relavant vehicles should be taken not considering a radius
            ########
        if can_overtake_:
            self._overtakeVeh = closest_vehicle
        return can_overtake_ 
        # return False 

    def is_light_red_or_no_light(self, lights_list, ego_location, ego_vehicle_waypoint,goal_waypoint ,ego_state,debug=False):
        """
        This method is specialized to check if there are traffic lights or not and if
        there are traffic lights, output it red or not.  

        :param  : lights_list           : list containing TrafficLight objects
                : ego_location          : Current location of ego-vehicle 
                                          format - Carla.Location
                : ego_vehicle_waypoint  : Nearest waypoint to the ego-vehicle projected to the middle of the lane
                                          format - Carla.Waypoint
                : goal_waypoint         : Nearest waypoint to the goal state
                                          format - Carla.Waypoint
                : ego_state             : Current state of the ego-vehicle
                                          format [x,y,yaw]
                : debug                 : Enable and disable printing for debugging
                                          format - bool
                
        :return : bool or "NTL"         : a bool flag or a string "NTL" 
                                        - Output True if there is a traffic light in RED
                                        affecting us and False otherwise"
                                        - "NTL" if there is no traffic light in the junction
        """
        
        if ego_vehicle_waypoint.is_junction:
            # It is too late. Do not block the intersection! Keep going!
            return False
        
        if goal_waypoint is not None:
            if (self._intersection_state):
                min_angle = 180.0
                sel_magnitude = 0.0
                sel_traffic_light = None
                angles = []
                mags = []

                for traffic_light in lights_list:                
                    loc = traffic_light.get_location()
                    magnitude, angle = compute_magnitude_angle(loc, ego_location, ego_state[2])  # angle in degrees

                    if((magnitude < TRAFFIC_LIGHT_CHECK_DISTANCE + self._lookahead ) and (angle < min_angle) and (angle>0) and (angle<= 60)):
                        angles.append(angle)
                        mags.append(magnitude)
                        sel_magnitude = magnitude
                        sel_traffic_light = traffic_light
                        min_angle = angle

                if(sel_traffic_light!= None):
                    self._world.debug.draw_line(ego_location, sel_traffic_light.get_location(), thickness=0.5, color=carla.Color(r=255, g=0, b=0), life_time=0.05)

                if sel_traffic_light is not None:
                    if debug:
                        print('=== Magnitude = {} | Angle = {} | ID = {}'.format(
                            sel_magnitude, min_angle, sel_traffic_light.id))

                    if (sel_traffic_light.state == carla.TrafficLightState.Red or sel_traffic_light.state == carla.TrafficLightState.Yellow):
                        return True
                else:
                    # No traffic lights in the intersection
                    return "NTL"   
        return False

    def traffic_light_green(self,lights_list):
        for traffic_light in lights_list:
            traffic_light.set_state(carla.TrafficLightState.Green)
            # traffic_light.set_green_time(self, 5000.0)

