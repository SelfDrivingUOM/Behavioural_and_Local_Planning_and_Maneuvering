
# Author: Yasintha supun
# Date: July 27, 2020

import numpy as np
import copy
import path_optimizer
import collision_check
import velocity_planner
from math import sin, cos, pi, sqrt
from enum import Enum
from collections import deque
import random
import time


from collections import defaultdict
import multiprocessing
import concurrent.futures



class RoadOption(Enum):
    """
    RoadOption represents the possible topological configurations when moving from a segment of lane to other.
    """
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6


class LocalPlanner:
    def __init__(self, num_paths, path_offset, length, width, 
                 path_select_weight, time_gap, a_max, slow_speed, 
                 stop_line_buffer,NUMBER_OF_LAYERS):
        self._num_paths = num_paths
        self._path_offset = path_offset
        self._path_optimizer = path_optimizer.PathOptimizer()
        self._collision_checker = collision_check.CollisionChecker(length,
                                                                     width,
                                                                     path_select_weight)
        self._velocity_planner = velocity_planner.VelocityPlanner(time_gap, a_max, slow_speed,stop_line_buffer)
        self.number_of_layers = NUMBER_OF_LAYERS
        self.prev_goal_t=1000
        self.vals  = [0.0,0.0,0.000001]
        self.LUT = np.load("/home/selfdriving/BP_with_git/LUT.npz")["arr_0"]
        
    ######################################################
    # GOAL STATE COMPUTATION
    # Computes the goal state set from a given goal position. This is done by
    # laterally sampling offsets from the goal location along the direction
    # perpendicular to the goal yaw of the ego vehicle.
    ######################################################

    def get_goal_state_set(self, point_1,point_2, goal_state, ego_state,offset_):

        # wp1_x = (waypoints[goal_index+1][0] - ego_state[0]) * cos(-ego_state[2]) - (waypoints[goal_index+1][1] - ego_state[1]) * sin(-ego_state[2])
        # wp1_y = (waypoints[goal_index+1][0] - ego_state[0]) * sin(-ego_state[2]) + (waypoints[goal_index+1][1] - ego_state[1]) * cos(-ego_state[2])
        # wp0_x = (waypoints[goal_index][0] - ego_state[0]) * cos(-ego_state[2]) - (waypoints[goal_index][1] - ego_state[1]) * sin(-ego_state[2])
        # wp0_y = (waypoints[goal_index][0] - ego_state[0]) * sin(-ego_state[2]) + (waypoints[goal_index][1] - ego_state[1]) * cos(-ego_state[2])
        # wp_1_x = (waypoints[goal_index-1][0] - ego_state[0]) * cos(-ego_state[2]) - (waypoints[goal_index-1][1] - ego_state[1]) * sin(-ego_state[2])
        # wp_1_y = (waypoints[goal_index-1][0] - ego_state[0]) * sin(-ego_state[2]) + (waypoints[goal_index-1][1] - ego_state[1]) * cos(-ego_state[2])

        delta_x = point_1[0] - point_2[0]
        delta_y = point_1[1] - point_2[1]

        # if goal_index < (waypoints.shape[0]-1):
        #     delta_x = waypoints[goal_index+1][0] - waypoints[goal_index][0]
        #     delta_y = waypoints[goal_index+1][1] - waypoints[goal_index][1]
        #     # delta_x = wp1_x - wp0_x
        #     # delta_y = wp1_y - wp0_y
        # else: 
        #     delta_x = waypoints[goal_index][0] - waypoints[goal_index-1][0]
        #     delta_y = waypoints[goal_index][1] - waypoints[goal_index-1][1]
            # delta_x = wp0_x - wp_1_x
            # delta_y = wp0_y - wp_1_y

        # print(delta_x,delta_y,delta_x1,delta_y1)

        #closest_len, closest_index=get_closest_index(waypoints, goal_state)


        # delta_x = goal_state[0]-waypoints[goal_index-2][0]
        # delta_y = goal_state[1]-waypoints[goal_index-2][1]


        heading = np.arctan2(delta_y,delta_x)

        goal_state_local = copy.copy(goal_state)


        goal_state_local[0] -= ego_state[0] 
        goal_state_local[1] -= ego_state[1] 

        theta = -ego_state[2]
        goal_x = goal_state_local[0] * cos(theta) - goal_state_local[1] * sin(theta)
        goal_y = goal_state_local[0] * sin(theta) + goal_state_local[1] * cos(theta)

        goal_t = heading - ego_state[2]
        goal_v = goal_state[2]

        if goal_t > pi:
            goal_t -= 2*pi
        elif goal_t < -pi:
            goal_t += 2*pi
        '''if(self.prev_goal_t!=1000):
            check_difference_t = abs(goal_t- self.prev_goal_t)
            if check_difference_t>0.5:
            	goal_t=self.prev_goal_t
            	print("corrected")'''
        
        #print(goal_t)
        # Compute and apply the offset for each path such that
        # all of the paths have the same heading of the goal state, 
        # but are laterally offset with respect to the goal heading.
        goal_state_set = []
        for i in range(self._num_paths):

            offset = (i - self._num_paths // 2) * offset_

            x_offset = offset * cos(goal_t + pi/2)
            y_offset = offset * sin(goal_t + pi/2)

            goal_state_set.append([goal_x + x_offset, 
                                   goal_y + y_offset, 
                                   goal_t, 
                                   goal_v])


        self.prev_goal_t=goal_t
           
        return goal_state_set
   
    # def get_goal_state_set(self, goal_index, goal_state, waypoints, ego_state):

    #     # wp1_x = (waypoints[goal_index+1][0] - ego_state[0]) * cos(-ego_state[2]) - (waypoints[goal_index+1][1] - ego_state[1]) * sin(-ego_state[2])
    #     # wp1_y = (waypoints[goal_index+1][0] - ego_state[0]) * sin(-ego_state[2]) + (waypoints[goal_index+1][1] - ego_state[1]) * cos(-ego_state[2])
    #     # wp0_x = (waypoints[goal_index][0] - ego_state[0]) * cos(-ego_state[2]) - (waypoints[goal_index][1] - ego_state[1]) * sin(-ego_state[2])
    #     # wp0_y = (waypoints[goal_index][0] - ego_state[0]) * sin(-ego_state[2]) + (waypoints[goal_index][1] - ego_state[1]) * cos(-ego_state[2])
    #     # wp_1_x = (waypoints[goal_index-1][0] - ego_state[0]) * cos(-ego_state[2]) - (waypoints[goal_index-1][1] - ego_state[1]) * sin(-ego_state[2])
    #     # wp_1_y = (waypoints[goal_index-1][0] - ego_state[0]) * sin(-ego_state[2]) + (waypoints[goal_index-1][1] - ego_state[1]) * cos(-ego_state[2])

    #     if goal_index < (waypoints.shape[0]-1):
    #         delta_x = waypoints[goal_index+1][0] - waypoints[goal_index][0]
    #         delta_y = waypoints[goal_index+1][1] - waypoints[goal_index][1]
    #         # delta_x = wp1_x - wp0_x
    #         # delta_y = wp1_y - wp0_y
    #     else: 
    #         delta_x = waypoints[goal_index][0] - waypoints[goal_index-1][0]
    #         delta_y = waypoints[goal_index][1] - waypoints[goal_index-1][1]
    #         # delta_x = wp0_x - wp_1_x
    #         # delta_y = wp0_y - wp_1_y

    #     # print(delta_x,delta_y,delta_x1,delta_y1)

    #     #closest_len, closest_index=get_closest_index(waypoints, goal_state)


    #     # delta_x = goal_state[0]-waypoints[goal_index-2][0]
    #     # delta_y = goal_state[1]-waypoints[goal_index-2][1]


    #     heading = np.arctan2(delta_y,delta_x)

    #     goal_state_local = copy.copy(goal_state)


    #     goal_state_local[0] -= ego_state[0] 
    #     goal_state_local[1] -= ego_state[1] 

    #     theta = -ego_state[2]
    #     goal_x = goal_state_local[0] * cos(theta) - goal_state_local[1] * sin(theta)
    #     goal_y = goal_state_local[0] * sin(theta) + goal_state_local[1] * cos(theta)

    #     goal_t = heading - ego_state[2]
    #     goal_v = goal_state[2]

    #     if goal_t > pi:
    #         goal_t -= 2*pi
    #     elif goal_t < -pi:
    #         goal_t += 2*pi
    #     '''if(self.prev_goal_t!=1000):
    #         check_difference_t = abs(goal_t- self.prev_goal_t)
    #         if check_difference_t>0.5:
    #         	goal_t=self.prev_goal_t
    #         	print("corrected")'''
        
    #     #print(goal_t)
    #     # Compute and apply the offset for each path such that
    #     # all of the paths have the same heading of the goal state, 
    #     # but are laterally offset with respect to the goal heading.
    #     goal_state_set = []
    #     for i in range(self._num_paths):

    #         offset = (i - self._num_paths // 2) * self._path_offset

    #         x_offset = offset * cos(goal_t + pi/2)
    #         y_offset = offset * sin(goal_t + pi/2)

    #         goal_state_set.append([goal_x + x_offset, 
    #                                goal_y + y_offset, 
    #                                goal_t, 
    #                                goal_v])


    #     self.prev_goal_t=goal_t
           
    #     return goal_state_set
         
    ######################################################     
    # Plans the path set using polynomial spiral optimization to
    # each of the goal states.
    ######################################################
    def plan_paths(self, goal_state_set):

        paths         = []
        path_validity = []
        #print(goal_state_set[7][2])

        toc = time.time()
        
        # processes = []
        # manager = multiprocessing.Manager()
        # return_dict = manager.dict()

        # for i in range(7):
            
        #     self.vals = self.LUT[min(int(goal_state_set[i][0]/0.1),99),min(100+ int(goal_state_set[i][1]/0.1),200),min(180+int(np.degrees(goal_state_set[i][2])),360)]

        #     p = multiprocessing.Process(target=self._path_optimizer.optimize_spiral,args= [goal_state_set[i][0],goal_state_set[i][1],goal_state_set[i][2],self.vals,return_dict,i])
        #     p.start()
        #     processes.append(p)
        
        # for i in processes:

        #     i.join()

        # # print(return_dict.values())
        # for res in return_dict.values():

        #     path,vals,goal_state = res
        #     # print(res)
        #     if np.linalg.norm([path[0][-1] - goal_state[0], 
        #                     path[1][-1] - goal_state[1], 
        #                     path[2][-1] - goal_state[2]]) > 0.1:
        #         path_validity.append(False)
        #         #paths.append(path)
        #     else:
        #         paths.append(path)
        #         path_validity.append(True)
        #     # print(vals)
        #     self.vals = vals

        # with concurrent.futures.ProcessPoolExecutor() as executor:
        # self.vals = self.LUT[min(int(goal_state[0]/0.1),99),min(100+ int(goal_state[1]/0.1),200),min(180+int(np.degrees(goal_state[2])),360)]
        #     exec_ = [executor.submit(self._path_optimizer.optimize_spiral,goal_state[0],goal_state[1],goal_state[2],self.vals) for goal_state in goal_state_set]

        #     for res in concurrent.futures.as_completed(exec_):
                
        #         path,vals,goal_state = res.result()

        #         if np.linalg.norm([path[0][-1] - goal_state[0], 
        #                         path[1][-1] - goal_state[1], 
        #                         path[2][-1] - goal_state[2]]) > 0.1:
        #             path_validity.append(False)
        #             #paths.append(path)
        #         else:
        #             paths.append(path)
        #             path_validity.append(True)
        #         # print(vals)
        #         self.vals = vals

        for goal_state in goal_state_set:
            
            # print(goal_state[0] - min(round(goal_state[0]/0.05),399)*0.05,min(200+ round(goal_state[1]/0.05),400),min(180+round(np.degrees(goal_state[2])),360))
            self.vals = self.LUT[int(min(round(goal_state[0]/0.05),399)),int(min(200+ round(goal_state[1]/0.05),400)),int(min(180+round(np.degrees(goal_state[2])),360))]
            path,vals,goal_state = self._path_optimizer.optimize_spiral(goal_state[0], 
                                                      goal_state[1], 
                                                      goal_state[2],self.vals)

            # print(np.linalg.norm([path[0][-1] - goal_state[0], path[1][-1] - goal_state[1], path[2][-1] - goal_state[2]]))
            if np.linalg.norm([path[0][-1] - goal_state[0], 
                               path[1][-1] - goal_state[1], 
                               path[2][-1] - goal_state[2]]) > 100:
                path_validity.append(False)
                #paths.append(path)
            else:
                paths.append(path)
                path_validity.append(True)
            # print(vals)
            self.vals = vals
        
        # tic = time.time()
        # print(path_validity)
        # print(tic-toc)
        return paths, path_validity

    def plan_lane_change(self, goal_state):
            
        # print(goal_state[0] - min(round(goal_state[0]/0.05),399)*0.05,min(200+ round(goal_state[1]/0.05),400),min(180+round(np.degrees(goal_state[2])),360))
        self.vals = self.LUT[int(min(round(goal_state[0]/0.05),399)),int(min(200+ round(goal_state[1]/0.05),400)),int(min(180+round(np.degrees(goal_state[2])),360))]
        path,vals,goal_state = self._path_optimizer.optimize_spiral(goal_state[0], 
                                                    goal_state[1], 
                                                    goal_state[2],self.vals)

        return path

    
    #number of layers mean how many longitudinal stations are there along the waypoints

    def lattice_layer_stations(self, final_goal, waypoints, ego_state):
        goal_set=[]
        goal_index_set=[]

        # final_goal=waypoints[goal_index]
        
        for i in range(self.number_of_layers):
            goal=[]
            goal_x = ego_state[0]+((i+1)/self.number_of_layers)*(final_goal[0]-ego_state[0])
            goal_y = ego_state[1]+((i+1)/self.number_of_layers)*(final_goal[1]-ego_state[1])
            goal_speed = final_goal[2]

            goal.append(goal_x)
            goal.append(goal_y)
            goal.append(goal_speed)

            closest_len, closest_index=get_closest_index(waypoints, goal)
            goal_index_set.append(closest_index)
            goal_set.append(waypoints[closest_index])

        return goal_set, goal_index_set
    # def lattice_layer_stations(self, goal_index, waypoints, ego_state):
    #     goal_set=[]
    #     goal_index_set=[]

    #     final_goal=waypoints[goal_index]
        
    #     for i in range(self.number_of_layers):
    #         goal=[]
    #         goal_x = ego_state[0]+((i+1)/self.number_of_layers)*(final_goal[0]-ego_state[0])
    #         goal_y = ego_state[1]+((i+1)/self.number_of_layers)*(final_goal[1]-ego_state[1])
    #         goal_speed = final_goal[2]

    #         goal.append(goal_x)
    #         goal.append(goal_y)
    #         goal.append(goal_speed)

    #         closest_len, closest_index=get_closest_index(waypoints, goal)
    #         goal_index_set.append(closest_index)
    #         goal_set.append(waypoints[closest_index])

    #     return goal_set, goal_index_set

######################################################
# Converts the to the global coordinate frame.
######################################################

def transform_paths(paths, ego_state):

    #change to num paths
    paths_np = np.array(paths)

    transformed_paths_np = np.empty((11,3,49))

    transformed_paths_np[:,0,:] = ego_state[0] + (paths_np[:,0,:]*np.cos(ego_state[2])) - (paths_np[:,1,:]*np.sin(ego_state[2]))
    transformed_paths_np[:,1,:] = ego_state[1] + (paths_np[:,0,:]*np.sin(ego_state[2])) + (paths_np[:,1,:]*np.cos(ego_state[2]))
    transformed_paths_np[:,2,:] = ego_state[2] + paths_np[:,2,:]

    # print(paths_np.shape)
    # transformed_paths = []
    # for path in paths:
    #     x_transformed = []
    #     y_transformed = []
    #     t_transformed = []

    #     for i in range(len(path[0])):
    #         x_transformed.append(ego_state[0] + path[0][i]*cos(ego_state[2]) - path[1][i]*sin(ego_state[2]))

    #         y_transformed.append(ego_state[1] + path[0][i]*sin(ego_state[2]) + path[1][i]*cos(ego_state[2]))
    #         t_transformed.append(path[2][i] + ego_state[2])

    #     transformed_paths.append([x_transformed, y_transformed, t_transformed])

    return np.array(transformed_paths_np)

def transform_goal_set(goal_set,ego_state):
    transformed_goals=[]
    for goal in goal_set:
        g_goal=[]
        g_goal.append(ego_state[0] + goal[0]*cos(ego_state[2]) - goal[1]*sin(ego_state[2]))
        g_goal.append(ego_state[1] + goal[0]*sin(ego_state[2]) + goal[1]*cos(ego_state[2]))
        g_goal.append(goal[2] + ego_state[2])
        g_goal.append(goal[3])

        transformed_goals.append(g_goal)

    return transformed_goals


def get_closest_index(waypoints, ego_state):
    """
    Gets closest index a given list of waypoints to the vehicle position.
    """
    closest_len = float('Inf')
    closest_index = 0

    waypoint_dists = np.sqrt(np.square(waypoints[:,0] - ego_state[0]) + np.square(waypoints[:,1] - ego_state[1]))
    closest_len = np.amin(waypoint_dists)
    closest_index = np.where((waypoint_dists==closest_len))[0][0]
    return closest_len, closest_index