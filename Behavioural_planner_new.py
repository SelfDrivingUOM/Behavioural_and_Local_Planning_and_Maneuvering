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

BP_LOOKAHEAD_BASE      = 10.0             # m
BP_LOOKAHEAD_TIME      = 1.0              # s

#states
FOLLOW_LANE            = 0
DECELERATE_TO_STOP     = 1
STAY_STOPPED           = 2

##initial state


class BehaviouralPlanner:
    def __init__(self, lp, waypoints,environment, world):
        self._lp            = lp
        self._waypoints     = waypoints
        self._goal_state    = [0.0, 0.0, 0.0]
        self._goal_index    = 0
        self._environment   = environment
        self._world         = world
        self._state         = FOLLOW_LANE
        self._lookahead     = BP_LOOKAHEAD_BASE
        self.paths = None
        self._goal_state_next =None
        self.min_collision_idx = None


    def state_machine(self, ego_state, current_timestamp, prev_timestamp,current_speed):

        open_loop_speed = self._lp._velocity_planner.get_open_loop_speed(current_timestamp - prev_timestamp)
        self._lookahead= BP_LOOKAHEAD_BASE + BP_LOOKAHEAD_TIME * open_loop_speed 
        vehicles_static, vehicles_dynamic, walkers = self._environment.get_actors(30)
        obstacle_actors = vehicles_static + vehicles_dynamic + walkers
        
        # draw_bound_box(obstacle_actors,self._world)
        
        if (self._state   == FOLLOW_LANE):
            # First, find the closest index to the ego vehicle.
            closest_len, closest_index = self.get_closest_index(ego_state)
            # Next, find the goal index that lies within the lookahead distance
            # along the waypoints.
            goal_index = self.get_goal_index(ego_state, closest_len, closest_index)
            self._goal_index = goal_index
            self._goal_state = self._waypoints[goal_index]


            if goal_index < (self._waypoints.shape[0]-1):
                point_1 = self._waypoints[goal_index+1]
                point_2 = self._waypoints[goal_index]

            else: 
                point_1 = self._waypoints[goal_index]
                point_2 = self._waypoints[goal_index-1]

            # Compute the goal state set from the behavioural planner's computed goal state.
            goal_set, goal_index_set =  self._lp.lattice_layer_stations(self._goal_state , self._waypoints, ego_state)
            goal_state = goal_set[0]
            gi = goal_index_set[0]

            goal_state_set = self._lp.get_goal_state_set(point_1,point_2,goal_state, ego_state)
            
            # Calculate planned paths in the local frame.
            paths, path_validity = self._lp.plan_paths(goal_state_set)

            # Transform those paths back to the global frame.
            paths = local_planner.transform_paths(paths, ego_state)

            collision_check_array,min_collision = self._lp._collision_checker.collision_check_static(paths, obstacle_actors,self._world)
            

            # print(min_collisions)
            # Compute the best local path.
            best_index = self._lp._collision_checker.select_best_path_index(paths, collision_check_array, self._goal_state,self._waypoints,ego_state)

            if((abs(best_index - self._lp._num_paths//2))>= 3):
        
                self._state   = DECELERATE_TO_STOP
                self._goal_state = paths[self._lp._num_paths//2,:,max(min_collision-1,1)]
                self._goal_state_next = paths[self._lp._num_paths//2,:,max(min_collision,0)]
                self._goal_state[2] = 0

            else:

                best_index = 7
                best_path = paths[best_index]
            
                debug_print(paths,self._world,best_index)
                local_waypoints = self._lp._velocity_planner.nominal_profile(best_path, open_loop_speed, self._goal_state[2])

                return local_waypoints

        if (self._state == DECELERATE_TO_STOP):
            
            # print(self.paths.shape)
            
            goal_set, goal_index_set =  self._lp.lattice_layer_stations(self._goal_state , self._waypoints, ego_state)
            
            goal_state = goal_set[0]
            gi = goal_index_set[0]
            goal_state_set = self._lp.get_goal_state_set(self._goal_state_next,self._goal_state, self._goal_state, ego_state)

            paths, path_validity = self._lp.plan_paths(goal_state_set)

            # Transform those paths back to the global frame.
            paths = local_planner.transform_paths(paths, ego_state)
            
            collision_check_array,min_collision = self._lp._collision_checker.collision_check_static(paths, obstacle_actors,self._world)   
            # best_index = self._lp._collision_checker.select_best_path_index(paths, collision_check_array, self._goal_state,self._waypoints,ego_state)
            best_index = 7
            debug_print(paths,self._world,best_index)
            # print("a")

            local_waypoints = self._lp._velocity_planner.decelerate_profile(paths[best_index],current_speed)
            # print(local_waypoints)

            # raise Exception
            return local_waypoints
        if (self._state   == STAY_STOPPED):
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







