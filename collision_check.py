import glob
import os
import sys
import time

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('/home/selfdriving/carla-precompiled/CARLA_0.9.9/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg' )[0])
except IndexError:
    pass

try:
    sys.path.append('/home/selfdriving/yasintha/Path_planner_6/')

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
        # points are in global frame 

        obstacle_pts = np.empty((0,2))
        # print(dict_obs)

        # print(obstacle_actors)
        for i in range(obstacle_actors.shape[0]):
            temp =np.array([[obstacle_actors[i].bounding_box.extent.x,obstacle_actors[i].bounding_box.extent.y,obstacle_actors[i].bounding_box.location.z],
                            [-obstacle_actors[i].bounding_box.extent.x,obstacle_actors[i].bounding_box.extent.y,obstacle_actors[i].bounding_box.location.z],
                            [-obstacle_actors[i].bounding_box.extent.x,-obstacle_actors[i].bounding_box.extent.y,obstacle_actors[i].bounding_box.location.z],
                            [obstacle_actors[i].bounding_box.extent.x,-obstacle_actors[i].bounding_box.extent.y,obstacle_actors[i].bounding_box.location.z],
                            [0,obstacle_actors[i].bounding_box.extent.y,obstacle_actors[i].bounding_box.location.z],
                            [-obstacle_actors[i].bounding_box.extent.x,0,obstacle_actors[i].bounding_box.location.z],
                            [0,-obstacle_actors[i].bounding_box.extent.y,obstacle_actors[i].bounding_box.location.z],
                            [obstacle_actors[i].bounding_box.extent.x,0,obstacle_actors[i].bounding_box.location.z]] )
            
            yaw = -np.radians(obstacle_actors[i].get_transform().rotation.yaw)     #z
            pitch = np.radians(obstacle_actors[i].get_transform().rotation.pitch) #y
            roll = np.radians(obstacle_actors[i].get_transform().rotation.roll)   #x

            Rot_mat = np.array([[np.cos(pitch)*np.cos(yaw),np.sin(roll)*np.sin(pitch)*np.cos(yaw)-np.cos(roll)*np.sin(yaw),np.cos(roll)*np.sin(pitch)*np.cos(yaw)+np.sin(roll)*np.sin(yaw)],
                                [np.cos(pitch)*np.sin(yaw),np.sin(roll)*np.sin(pitch)*np.sin(yaw)+np.cos(roll)*np.cos(yaw),np.cos(roll)*np.sin(pitch)*np.sin(yaw)-np.sin(roll)*np.cos(yaw)],
                                [-np.sin(pitch),np.sin(roll)*np.cos(pitch),np.cos(roll)*np.cos(pitch)]])

            temp = np.linalg.inv(Rot_mat)@temp.T
            trans = np.array([[obstacle_actors[i].bounding_box.location.x+obstacle_actors[i].get_location().x],
                              [obstacle_actors[i].bounding_box.location.y+obstacle_actors[i].get_location().y],
                              [obstacle_actors[i].bounding_box.location.z+obstacle_actors[i].get_location().z]])

            temp += trans
            temp = temp[:2,:].T.reshape(8,2)
            # print(list(dict_obs.keys())[i])
            for ll in range(8):
                loc = carla.Location(x=temp[ll,0],y=temp[ll,1],z=0)
                world.debug.draw_string(loc, 'Z', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=0.1,persistent_lines=True)
            temp = np.vstack((temp,)*len(dict_obs[list(dict_obs.keys())[i]]))
            obstacle_pts = np.append(obstacle_pts, temp, axis = 0)

        return obstacle_pts


    def collision_check_static(self, paths, obstacle_actors,world):
        
        # collision_check_array = [False]*len(paths)
        # i = 0
        # for path in paths:
        obstacle_num = len(obstacle_actors)
        collission_check_array = np.array([True]*paths.shape[0])
        # print(obstacle_num)
        mins = []
        obstacle_actors = np.array(obstacle_actors)
        if obstacle_num > 0:
            j=0
            
            for path in paths:
                theta = path[2,:].reshape((1,path.shape[1])).T
                path = path[:2,:]
                path_len = path.shape[1]
                path = path.T

                dist = np.zeros((obstacle_num*path_len,1))
                for i in range(obstacle_num):
                    dist[path_len*i:path_len*(i+1),:] = np.sum(np.square(path -np.array([obstacle_actors[i].get_location().x,obstacle_actors[i].get_location().y])),axis=1).reshape(path_len,1) - \
                                                        np.square(np.sqrt(np.square(obstacle_actors[i].bounding_box.extent.x)+np.square(obstacle_actors[i].bounding_box.extent.y))+self.A)  
                    # print(np.sum(np.square(path -np.array([obstacle_actors[i].get_location().x,obstacle_actors[i].get_location().y])),axis=1).shape)
                    # for z in range(49):
                    # world.debug.draw_line(carla.Location(x=path[0,0] , y=path[0,1],z=0),obstacle_actors[i].get_location(), thickness=0.1, color=carla.Color(r=0, g=255, b=0), life_time=-1.)

            
                if(np.any(dist<0)):
                    
                    temp = np.where(dist<0)[0]

                    check_wayp = temp%path_len
                    obst_what = temp//path_len
                    dict_obs = defaultdict(list)
                    count_obs = defaultdict(int)
                    for i in range(check_wayp.shape[0]):
                        dict_obs[obst_what[i]].append(check_wayp[i])
                        count_obs[obst_what[i]]+=1


                    obst_ind  = np.sort(np.array(list(set(obst_what))))

                    obs_points  = self.get_points(obstacle_actors[obst_ind], dict_obs,world)
                    
                    path_points = path[np.vstack((np.hstack(np.array(list(dict_obs.values()))),)*8)]
                    path_points = path_points.reshape(int(path_points.size/2),2)

                  #  print(np.array(list(dict_obs.values())).shape,"shape")

                    theta_points = theta[np.vstack((np.hstack(np.array(list(dict_obs.values()))),)*8)]
                    theta_points = theta_points.reshape(int(theta_points.size),1)
                    
                    ellipse_check = (np.square((obs_points[:,0]-path_points[:,0]).reshape(int(theta_points.size),1)*np.cos(theta_points)-(obs_points[:,1]-path_points[:,1]).reshape(int(theta_points.size),1)*np.sin(theta_points))/self.A**2 + \
                                    np.square((obs_points[:,0]-path_points[:,0]).reshape(int(theta_points.size),1)*np.sin(theta_points)+(obs_points[:,1]-path_points[:,1]).reshape(int(theta_points.size),1)*np.cos(theta_points))/self.B**2 ) - 1


                    if(np.any(ellipse_check<0)):
                        collission_check_array[j] = False

                        collide_points = np.where(ellipse_check<0)[0]//8
                        temp_dict = {k: v for k, v in sorted(count_obs.items(), key=lambda item: item[1])}
                        collision_vals = list(temp_dict.values())
                        obst_indices = list(temp_dict.keys())
                        
                        # print(collision_vals)
                        # print(collide_points)
                        temp_ = collision_vals.pop(0)
                        obst_ind = obst_indices.pop(0)
                        min_ = 49
                        start = 0
                        
                        for i in collide_points:
                            if(i-start<temp_):
                                if(dict_obs[obst_ind][i-start]<min_):
                                    min_ = dict_obs[obst_ind][i-start]

                            else:
                                
                                while(True):
                                    start += temp_
                                    temp_ = collision_vals.pop()
                                    obst_ind = obst_indices.pop(0)

                                    if(i-start<temp_):
                                        if(dict_obs[obst_ind][i-start]<min_):
                                            min_ = dict_obs[obst_ind][i-start]
                                        break

                        mins.append(min_)

                        # print(count_obs)
                        # print(lol)


                      #  print(np.where(ellipse_check<0)[0],lol)
                    else:
                        mins.append(48)
                        pass
                else:
                    mins.append(48)
                    pass
                j += 1
        else:
            mins = [48]*14
        # print(mins)
        return collission_check_array,min(mins)
        #no collision
        """get the obstacle radii and obtain the points which of he path which may intersect with the vehicle """

    
    # def collision_check_dynamic(self,state):


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
    closest_len = float('Inf')
    closest_index = 0
    # TODO: INSERT YOUR CODE BETWEEN THE DASHED LINES
    # ------------------------------------------------------------------
    # for i in range(len(waypoints)):
    #   ...
    # for i in range(len(waypoints)):
    #     temp = (waypoints[i][0] - ego_state[0])**2 + (waypoints[i][1] - ego_state[1])**2
    #     if temp < closest_len:
    #         closest_len = temp
    #         closest_index = i
    # closest_len = np.sqrt(closest_len)
    # # ------------------------------------------------------------------

    waypoint_dists = np.sqrt(np.square(waypoints[:,0] - ego_state[0]) + np.square(waypoints[:,1] - ego_state[1]))
    closest_len = np.amin(waypoint_dists)
    closest_index = np.where((waypoint_dists==closest_len))[0][0]
    return closest_len, closest_index




