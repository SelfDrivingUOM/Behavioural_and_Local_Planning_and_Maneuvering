import glob
import os
import sys
import time
from os_carla import WINDOWS

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
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

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

MAX_WEIGHT = 100000

import carla

import numpy as np
from collections import defaultdict
class CollisionChecker:
    def __init__(self, length, width, weight):
        self.length = length
        self.width =  width
        self.weight = weight

        self.A = self.length/(2**0.5)
        self.B = self.width/(2**0.5)

        ###!!!###
        #----to store the previous path ----
        self.prev_path_index=0  
        ###!!!###

    def get_points(self,obstacle_actors, dict_obs,world):
        """
            Inputs
                obstacle_actors -   Numpy array containing the filtered obstacle actors colliding with considereed local path
                                        numpy.ndarray[obs0, obs1, .., obsN]

                dict_obs        -   Dictionary containing indices of obstacle actor in obstacle_actor input(as keys) and the
                                    corrosponding collision risk local planner path point indices for the considered local path(as values)
                                        { 0 : List of indexes of points with risk of colliding with obs0, 
                                          1 : List of indexes of points with risk of colliding with obs1,
                                          :       :
                                          :       :
                                          N : List of indexes of points with risk of colliding with obsN }

                world           -   world object of the CARLA world in the simulator

            Outputs
                obstacle_pts    -   Numpy array containing 8 boundary points of each obstances repeated by the number of local path points 
                                    with collision risk with the corresponding obstacle.
                                        numpy.ndarray[ [obs0_X1,obs0_Y1],  =
                                                        [obs0_X2,obs0_Y2],  |
                                                            :      :        |
                                                        [obs0_X8,obs0_Y8],  |
                                                                :           |  Repeated by number of point
                                                                :           |  in local path considered with 
                                                                :           |  risk of collision with obs0
                                                        [obs0_X1,obs0_Y1],  |
                                                        [obs0_X2,obs0_Y2],  |
                                                            .      .        |
                                                        [obs0_X8,obs0_Y8],  =
                                                                :           
                                                                :
                                                                :
                                                                :
                                                        [obsN_X1,obsN_Y1],  =
                                                        [obsN_X2,obsN_Y2],  |
                                                            :      :        |
                                                        [obsN_X8,obsN_Y8],  |
                                                                :           |  Repeated by number of point
                                                                :           |  in local path considered with 
                                                                :           |  risk of collision with obsN
                                                        [obsN_X1,obsN_Y1],  |
                                                        [obsN_X2,obsN_Y2],  |
                                                            :      :        |
                                                        [obsN_X8,obsN_Y8] ] =
        """                         
        obstacle_pts = np.empty((0,2))
        for i in range(obstacle_actors.shape[0]):
            # Find the 8 obstacle boundary points (4 vertices + midpoints of 4 sides) wrt to obstacle frame
            temp =np.array([[ obstacle_actors[i].bounding_box.extent.x, obstacle_actors[i].bounding_box.extent.y, obstacle_actors[i].bounding_box.location.z],
                            [-obstacle_actors[i].bounding_box.extent.x, obstacle_actors[i].bounding_box.extent.y, obstacle_actors[i].bounding_box.location.z],
                            [-obstacle_actors[i].bounding_box.extent.x,-obstacle_actors[i].bounding_box.extent.y, obstacle_actors[i].bounding_box.location.z],
                            [ obstacle_actors[i].bounding_box.extent.x,-obstacle_actors[i].bounding_box.extent.y, obstacle_actors[i].bounding_box.location.z],
                            [                                        0, obstacle_actors[i].bounding_box.extent.y, obstacle_actors[i].bounding_box.location.z],
                            [-obstacle_actors[i].bounding_box.extent.x,                                        0, obstacle_actors[i].bounding_box.location.z],
                            [                                        0,-obstacle_actors[i].bounding_box.extent.y, obstacle_actors[i].bounding_box.location.z],
                            [ obstacle_actors[i].bounding_box.extent.x,                                        0, obstacle_actors[i].bounding_box.location.z]])
            
            # Get roll, pitch, yaw of obstacle and calculate rotational transformation of obstacle bounding box wrt to World 
            # (Obstacle bounding box has same orientaion wrt to obstacle frame)
            yaw   = -np.radians(obstacle_actors[i].get_transform().rotation.yaw)      # about -z (CARLA uses Left Handed system (axes))
            pitch =  np.radians(obstacle_actors[i].get_transform().rotation.pitch)    # about +y
            roll  =  np.radians(obstacle_actors[i].get_transform().rotation.roll)     # about +x

            Rot_mat = np.array([[np.cos(pitch)*np.cos(yaw), np.sin(roll)*np.sin(pitch)*np.cos(yaw)-np.cos(roll)*np.sin(yaw), np.cos(roll)*np.sin(pitch)*np.cos(yaw)+np.sin(roll)*np.sin(yaw)],
                                [np.cos(pitch)*np.sin(yaw), np.sin(roll)*np.sin(pitch)*np.sin(yaw)+np.cos(roll)*np.cos(yaw), np.cos(roll)*np.sin(pitch)*np.sin(yaw)-np.sin(roll)*np.cos(yaw)],
                                [-np.sin(pitch), np.sin(roll)*np.cos(pitch), np.cos(roll)*np.cos(pitch)]])

            # Consider both location of obstacle and also the offset of bounding box center from location of obstacle for 
            # translational transormation of obastacle bounding box wrt to world
            trans = np.array([[obstacle_actors[i].bounding_box.location.x + obstacle_actors[i].get_location().x],
                              [obstacle_actors[i].bounding_box.location.y + obstacle_actors[i].get_location().y],
                              [obstacle_actors[i].bounding_box.location.z + obstacle_actors[i].get_location().z]])

            # Tranform Obstacle boundary points to world frame
            temp = np.linalg.inv(Rot_mat)@temp.T + trans
            temp = temp[:2,:].T.reshape(8,2)
            for ll in range(8):
                loc = carla.Location(x=temp[ll,0],y=temp[ll,1],z=0)
                world.debug.draw_string(loc, 'Z', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=0.1,persistent_lines=True)
            
            # vertically stack set of obstacle boundary point by number of points in local path considered that has risk of colliding with the obstacle
            temp = np.vstack((temp,)*len(dict_obs[list(dict_obs.keys())[i]]))
            obstacle_pts = np.append(obstacle_pts, temp, axis = 0)

        return obstacle_pts


    def collision_check_static(self, paths, obstacle_actors, world):
        """
            Inputs
                paths           -   Numpy array containing the paths provide by the local planner.
                                        numpy.ndarray[[ [path1_x1,path1_x2,.....,path1_xn],    (x)
                                                        [path1_y1,path1_y2,.....,path1_yn],    (y)    Path1
                                                        [path1_h1,path1_h2,.....,path1_hn] ]  (teta)    :
                                                                        :                                :
                                                                        :                                :
                                                        [ [pathN_x1,pathN_x2,.....,pathN_xn],    (x)      :
                                                        [pathN_y1,pathN_y2,.....,pathN_yn],    (y)    PathN
                                                        [pathN_h1,pathN_h2,.....,pathN_hn] ]] (teta)
                                    
                obstacle_actors -   Numpy array containing the filtered obstacle actors colliding with considereed local path
                                        numpy.ndarray[obs0, obs1, .., obsN]

                world           -   world object of the CARLA world in the simulator

            Outputs
                collision_check_array - Numpy array of boolean type with each element coorsponding to the obstacle collision free status of each local path.
                                            'True'  =>  local path collison free
                                            'False' =>  local path collison detected

                closest_colln_index   - minimum index of path point considering all local paths at which collision witha an obstacle takes place.
                                        If no collions the index of the last point(furthest point) of a local path is returned.
        """
        ##############################################################################
            #   TODO:
            #       min_ set for 48 should be extarcted from paths shape
            #       14 used for number of paths should be extracted from paths shape
        ##############################################################################

        obstacle_num = len(obstacle_actors)
        # Initially set all paths are considered as collision free ('True' implies coreosponding local path collison free)
        collision_check_array = np.array([True]*paths.shape[0])
        mins = []
        min_objs = []
        obstacle_actors = np.array(obstacle_actors)

        # If no obstacles present there is no collision of paths
        if obstacle_num > 0:
            j = 0
            # Check Collision of each local path provided by loal planner one by one
            for path in paths:
                theta = path[2,:].reshape((1,path.shape[1])).T
                path = path[:2,:].T
                path_len = path.shape[0]

                # Consider the distance (DisPtObs) between each path point and each obstacle center to evaluate collision risk
                # Both obstacle and Ego are considerd as circle boundaries
                # Radius of Ego Boundary (R_ego)      = (length of Ego * (2**0.5))
                # Radius of Obstacle Boundary (R_obs) = ((obs_len/2)**2 + obs_wdt/2)**2)**0.5 {length of extent vector projected to XY plane of obstacle bounding box}
                # (DisPtObs - R_ego - R_obs) >= 0  implies no collision risk of Ego when at considered local path point with considered obstacle
                dist = np.zeros((obstacle_num*path_len,1))
                for i in range(obstacle_num):
                    dist[path_len*i:path_len*(i+1),:] = np.sum(np.square(path -np.array([obstacle_actors[i].get_location().x,obstacle_actors[i].get_location().y])),axis=1).reshape(path_len,1) - \
                                                        np.square(np.sqrt(np.square(obstacle_actors[i].bounding_box.extent.x)+np.square(obstacle_actors[i].bounding_box.extent.y))+self.A)  
                    # print(np.sum(np.square(path -np.array([obstacle_actors[i].get_location().x,obstacle_actors[i].get_location().y])),axis=1).shape)
                    # for z in range(49):
                    # world.debug.draw_line(carla.Location(x=path[0,0] , y=path[0,1],z=0),obstacle_actors[i].get_location(), thickness=0.1, color=carla.Color(r=0, g=255, b=0), life_time=-1.)


                if(np.any(dist<0)):

                    # Get details of path points with collision risk
                    # dict_obs -  Dictionary sorted by keys containing indices of obstacle actor in obstacle_actor variable(as keys) and the
                    #             corrosponding collision risk local planner path point indices for the considered local path(as values)
                    #                   { 0 : List of indexes of points with risk of colliding with obs0, 
                    #                     1 : List of indexes of points with risk of colliding with obs1,
                    #                     :       :
                    #                     :       :
                    #                     N : List of indexes of points with risk of colliding with obsN }
                    # count_obs - Dictionary sorted by keys containing indices of obstacle actor in obstacle_actor variable(as keys) and the
                    #             number of collision risk local planner path points with the corrosponding obstacle(as values)
                    #                   { 0 : len(List of indexes of points with risk of colliding with obs0), 
                    #                     1 : len(List of indexes of points with risk of colliding with obs1),
                    #                     :       :
                    #                     :       :
                    #                     N : len(List of indexes of points with risk of colliding with obsN) }

                    temp = np.where(dist<0)[0]

                    check_wayp = temp%path_len
                    obst_what = temp//path_len
                    dict_obs = defaultdict(list)
                    count_obs = defaultdict(int)
                    for i in range(check_wayp.shape[0]):
                        dict_obs[obst_what[i]].append(check_wayp[i])
                        count_obs[obst_what[i]]+=1
                    dict_obs = {k: v for k, v in sorted(dict_obs.items(), key=lambda item: item[0])}
                    count_obs = {k: v for k, v in sorted(count_obs.items(), key=lambda item: item[0])}

                    obst_ind  = np.sort(np.array(list(set(obst_what))))

                    # Get obstacle boundary points to check collision with 
                    obs_points  = self.get_points(obstacle_actors[obst_ind], dict_obs,world)
                    
                    path_points = path[np.vstack((np.hstack(np.array(list(dict_obs.values()))),)*8)]
                    path_points = path_points.reshape(int(path_points.size/2),2)

                    theta_points = theta[np.vstack((np.hstack(np.array(list(dict_obs.values()))),)*8)]
                    theta_points = theta_points.reshape(int(theta_points.size),1)
                    
                    ellipse_check = (np.square((obs_points[:,0]-path_points[:,0]).reshape(int(theta_points.size),1)*np.cos(theta_points)-(obs_points[:,1]-path_points[:,1]).reshape(int(theta_points.size),1)*np.sin(theta_points))/self.A**2 + \
                                    np.square((obs_points[:,0]-path_points[:,0]).reshape(int(theta_points.size),1)*np.sin(theta_points)+(obs_points[:,1]-path_points[:,1]).reshape(int(theta_points.size),1)*np.cos(theta_points))/self.B**2 ) - 1


                    if(np.any(ellipse_check<0)):
                        collision_check_array[j] = False

                        collide_points = np.where(ellipse_check<0)[0]//8
                        collision_vals = list(count_obs.values())
                        obst_indices = list(count_obs.keys())
                        
                        min_ = paths.shape[2]
                        min_obj_ = None
                        sum_obs = [0]
                        for i in collision_vals:
                            sum_obs.append(sum_obs[-1]+i)

                        for i in np.sort(collide_points):
                            for thresh in range(1,len(sum_obs)):
                                if i>=sum_obs[thresh]:
                                    continue
                                else:
                                    break
                            pt_ind = i - sum_obs[thresh-1]
                            pt = dict_obs[obst_indices[thresh-1]][pt_ind]
                            if pt<min_:
                                min_ = pt
                                min_obj_ = obstacle_actors[obst_indices[thresh-1]]

                        mins.append(min_)
                        min_objs.append(min_obj_)
                    else:
                        mins.append(paths.shape[2]-1)
                        min_objs.append(None)
                else:
                    mins.append(paths.shape[2]-1)
                    min_objs.append(None)
                j += 1
        else:
            mins = [paths.shape[2]-1]*paths.shape[0]
            min_objs = [None]*paths.shape[0]

        
        closest_colln_index = np.amin(np.array(mins))
        closest_min_obj    = min_objs[np.argmin(np.array(mins))]


        return collision_check_array, closest_colln_index, closest_min_obj


    def select_best_path_index(self, paths, collision_check_array, goal_state,waypoints,ego_state):
            """Returns the path index which is best suited for the vehicle to
            traverse.

            Selects a path index which is closest to the center line as well as far
            away from collision paths.

            args:
                paths: A list of paths in the global frame.  
                    A path is a list of points of the following format:
                        [x_points, y_points, t_points]:
                            x_points: List of x values (m)
                            y_points: List of y values (m)
                            t_points: List of yaw values (rad)
                        Example of accessing the ith path, jth point's t value:
                            paths[i][2][j]
                collision_check_array: A list of boolean values which classifies
                    whether the path is collision-free (true), or not (false). The
                    ith index in the collision_check_array list corresponds to the
                    ith path in the paths list.
                goal_state: Goal state for the vehicle to reach (centerline goal).
                    format: [x_goal, y_goal, v_goal], unit: [m, m, m/s]
            useful variables:
                self._weight: Weight that is multiplied to the best index score.
            returns:
                best_index: The path index which is best suited for the vehicle to
                    navigate with.
            """
            best_index = None
            best_score = float('Inf')

            _,closest_ego = get_closest_index(waypoints,ego_state)
            _,closest_goal = get_closest_index(waypoints,goal_state)
            

            # print(closest_ego,closest_goal,ego_state,goal_state)
            waypoints = waypoints[closest_ego:closest_goal]
            valid_paths = paths[collision_check_array]
            
            goal_score = np.sqrt((paths[:,0,-1]-goal_state[0])**2+(paths[:,1,-1]-goal_state[1])**2) #.reshape(7,1)

            
            centerline_score = []
            for path in paths:
                # print(path.shape,waypoints.shape)
                path_x = path[0,:].reshape(1,path.shape[1])
                path_y = path[1,:].reshape(1,path.shape[1])

                waypoints_x = waypoints[:,0].reshape(waypoints.shape[0],1)
                waypoints_y = waypoints[:,1].reshape(waypoints.shape[0],1)

                # print((path_x-waypoints_x).shape)
                sum_ = np.square(path_x - waypoints_x) + np.square(path_y - waypoints_y)
                min_dist = np.sqrt(np.min(sum_,axis=1))
                sum_=np.sum(min_dist)
                centerline_score.append(sum_)
                
            centerline_score = np.reshape(np.array(centerline_score),goal_score.shape)   

            total_score = goal_score + 20*centerline_score 
            total_score[~np.array(collision_check_array)] = 100000
            # print(goal_score.shape)


            #####Try to optimize this part better

            '''for i in range(paths.shape[0]):
                if collision_check_array[i]==True:
                    dist_arr = waypoints[:][:2].reshape(waypoints.shape[0],2) - paths[i][:2][:].reshape(2,49)'''

            # for i in range(paths.shape[0]):
            #     # Handle the case of collision-free paths.
            #     if collision_check_array[i]:
            #         # Compute the "distance from centerline" score.
            #         # The centerline goal is given by goal_state.
            #         # The exact choice of objective function is up to you.
            #         # A lower score implies a more suitable path.
            #         # TODO: INSERT YOUR CODE BETWEEN THE DASHED LINES
            #         # --------------------------------------------------------------
                    
            #         centerline_score = 0

        
            #         for j in range(paths.shape[2]):
            #             current_state = paths[i,0:2,j].reshape((2,))
                        
            #             closest_len, closest_index = get_closest_index(waypoints, current_state)
            #             centerline_score += closest_len
                    
            #         #print("new")
            #         #print(goal_score)
            #         #print(centerline_score)

            #         score = goal_score[i]#+100*centerline_score
            #         # --------------------------------------------------------------

                    

            #         # Compute the "proximity to other colliding paths" score and
            #         # add it to the "distance from centerline" score.
            #         # The exact choice of objective function is up to you.
            #         '''for j in range(len(paths)):
            #             if j == i:
            #                 continue
            #             else:
            #                 if not collision_check_array[j]:
            #                     # TODO: INSERT YOUR CODE BETWEEN THE DASHED LINES
            #                     # --------------------------------------------------
            #                     # score += self._weight * ...
            #                     #score += self._weight * np.sqrt((paths[i][0][-1]-paths[j][0][-1])**2+(paths[i][1][-1]-paths[j][1][-1])**2) * -1
            #                     # --------------------------------------------------
                                
            #                     pass'''

            #     # Handle the case of colliding paths.
            #     else:
            #         score = float('Inf')
                    
                # Set the best index to be the path index with the lowest score

            temp_ = np.amin(total_score)


            if(temp_ == MAX_WEIGHT):
                return None
            else:
                best_index = np.where(total_score == temp_)[0][0]

                ##!!!###
                #--- ~~~~~~~~~~~~~~~~~~~ ---#
                '''if abs(best_index - self.prev_path_index)<=3:
                    best_index = self.prev_path_index

                self.prev_path_index = best_index'''
                #--- ~~~~~~~~~~~~~~~~~~~ ---#
                ##!!!###
                # print(best_index)

       
                return best_index


def get_speed(vehicle):
    """
    Compute speed of a vehicle in Kmh
    :param vehicle: the vehicle for which speed is calculated
    :return: speed as a float in Kmh
    """
    vel = vehicle.get_velocity()
    return np.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

def get_closest_index(waypoints, ego_state):
    """
    Gets closest index a given list of waypoints to the vehicle position.
    """

    waypoint_dists = np.sqrt(np.square(waypoints[:,0] - ego_state[0]) + np.square(waypoints[:,1] - ego_state[1]))
    closest_len = np.amin(waypoint_dists)
    closest_index = np.where((waypoint_dists==closest_len))[0][0]
    return closest_len, closest_index




