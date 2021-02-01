import numpy as np
import carla

class Environment():
    def __init__(self, world, ego_vehicle,map_):
        self.world = world
        self.ego_vehicle = ego_vehicle
        self.ego_vehicle_loc = self.ego_vehicle.get_location()
        # self.distance = 10**4
        self._map = map_
        self.yaw = np.radians(ego_vehicle.get_transform().rotation.yaw)
        self.actors = self.world.get_actors()
        self.lights_list = self.actors.filter("*traffic_light*")
        self.vehicles = self.actors.filter('vehicle.*')
        self.walkers = self.actors.filter('walker.*')

        # self.actors = self.world.get_actors()
        # self.vehicles = self.actors.filter('vehicle.*')
        # self.walkers = self.actors.filter('walker.*')
        # self.first_time = True
        # print(len(self.lights_list),"A")
    def in_front(self,rot,ego_loc,actors,actors_loc):

        # print(rot.shape,actors.shape,ego_loc.shape,actors_loc.shape)

        car_frame = rot@((actors_loc - ego_loc).T)
        # return_walkers = walkers[crit]

        crit = car_frame[1] - self.ego_vehicle.bounding_box.extent.y >= 0
        car_frame = car_frame[:,crit]

        car_frame = np.append(car_frame,[actors[crit]],axis = 0)
        car_frame = car_frame[:, car_frame[1].argsort(kind = "mergesort")]

        # print(car_frame.shape,"HUUUU")
        return_actors = car_frame[2]
        actors_y = car_frame[1]

        # print(return_actors,actors_y)
        if(car_frame.shape[1] == 0):
            return None,None
        else:
            return actors_y[0],return_actors[0]



    def get_actors(self,in_radius,paths,middle_path_idx, intersection_state):
        
        # if(self.first_time):
        # 
        # #### This can be removed by taking the spawn NPC code within the new_main
        

        # self.first_time = False
        #print(np.shape(paths))
        vehicles = self.vehicles
        walkers  = self.walkers
        
        in_radius_sqr = np.square(in_radius)
        self.ego_vehicle_loc = self.ego_vehicle.get_location()
        self.yaw = np.radians(self.ego_vehicle.get_transform().rotation.yaw)

        return_dynamic_vehicles = []
        return_static_vehicles = []
        return_walkers = []
        vehicle_lanes = []
        vehicle_sections = []

        ego_waypoint=self._map.get_waypoint(self.ego_vehicle_loc,project_to_road=True,lane_type=carla.LaneType.Driving)
        ego_lane = ego_waypoint.lane_id
        ego_section = ego_waypoint.section_id

        if (type(paths) == type(None)):
            pass
        else:
            goal_location = carla.Location(x=paths[middle_path_idx,0,-1], y=paths[middle_path_idx,1,-1], z= 1.843102 )
            goal_waypoint=self._map.get_waypoint(goal_location,project_to_road=True,lane_type=carla.LaneType.Driving)
            goal_lane = goal_waypoint.lane_id
            goal_section = goal_waypoint.section_id
        
            
        distance = 10**4
        closest_vehicle = None

        # vehicle_loc = []
        # walker loc = []
        
        for vehicle in vehicles:
            if self.ego_vehicle.id == vehicle.id:
                return_dynamic_vehicles.append([10000,10000])
                return_static_vehicles.append([10000,10000])
                vehicle_lanes.append(ego_lane)
                vehicle_sections.append(ego_section)
                # continue
            else:
                
                # print("a")
                vehicle_loc = vehicle.get_location()
                vehicle_vel = vehicle.get_velocity()
                vehicle_waypoint = self._map.get_waypoint(vehicle_loc,project_to_road=True,lane_type=carla.LaneType.Driving)

                vehicle_lane = vehicle_waypoint.lane_id
                vehicle_section = vehicle_waypoint.section_id

                vehicle_lanes.append(vehicle_lane)
                vehicle_sections.append(vehicle_section)

                locs = [vehicle_loc.x,vehicle_loc.y]
                vels = np.array([vehicle_vel.x,vehicle_vel.y,vehicle_vel.z])

                if(vels[vels>0.0].size == 0 ):
                    return_dynamic_vehicles.append([10000,10000])
                    return_static_vehicles.append([vehicle_loc.x,vehicle_loc.y])
                else:
                    # print()
                    return_dynamic_vehicles.append([vehicle_loc.x,vehicle_loc.y])
                    return_static_vehicles.append([10000,10000])

        

                # vehicle_dist_sqr = np.sqrt((np.square(vehicle_loc.x - self.ego_vehicle_loc.x) + \
                #                     np.square(vehicle_loc.y - self.ego_vehicle_loc.y) + \
                #                     np.square(vehicle_loc.z - self.ego_vehicle_loc.z)))
                # if vehicle_dist_sqr < in_radius:
                #     vehicle_velocity = vehicle.get_velocity()
                #     vehicle_speed = np.linalg.norm(np.array([vehicle_velocity.x,vehicle_velocity.y,vehicle_velocity.z]))
                #     vehicle_lane = self._map.get_waypoint(vehicle_loc,project_to_road=True,lane_type=carla.LaneType.Driving).lane_id

                #     if(vehicle_dist_sqr<distance  and ego_lane == vehicle_lane):
                #         closest_vehicle = vehicle
                #         distance = vehicle_dist_sqr

                #     if vehicle_speed == 0.0:
                #         return_static_vehicles.append(vehicle)
                #     else:
                #         return_dynamic_vehicles.append(vehicle)
        # print(return_dynamic_vehicles,return_static_vehicles)
        for walker in walkers:
            walker_loc = walker.get_location()

            return_walkers.append([walker_loc.x,walker_loc.y])
            # walker_dist_sqr =  (np.square(walker_loc.x - self.ego_vehicle_loc.x) + \
            #                     np.square(walker_loc.y - self.ego_vehicle_loc.y) + \
            #                     np.square(walker_loc.z - self.ego_vehicle_loc.z))
            # if walker_dist_sqr < in_radius_sqr:
            #     return_walkers.append(walker)
        loc = np.array([self.ego_vehicle_loc.x,self.ego_vehicle_loc.y])
        dist_dynamic = None
        dist_static = None

        dynamic_closest = None
        static_closest = None

        vehicles = np.array(vehicles)
        walkers = np.array(walkers)
        walkers_y = None
        rot = np.array([[np.cos(self.yaw),-np.sin(self.yaw)],[np.cos(self.yaw),np.sin(self.yaw)]])

        x_vec = rot.T@np.array([[1],[0]])
        x_vec = x_vec.reshape((2,))

        y_vec = rot.T@np.array([[0],[1]])
        y_vec = y_vec.reshape((2,))

        if(vehicle_lanes!=[]):
            vehicle_lanes = np.array(vehicle_lanes)
            vehicle_sections = np.array(vehicle_sections)

        

        if(return_dynamic_vehicles !=[]):

            return_dynamic_vehicles = np.array(return_dynamic_vehicles)


            dist_dynamic = np.sum(np.square(return_dynamic_vehicles-loc),axis =1 )
            dist_temp = dist_dynamic
            # print(dist_dynamic,"b")
            # print(dist_dynamic<(in_radius**2))

            # dyns = vehicles[np.logical_and(dist_dynamic<in_radius**2, ego_lane == vehicle_lanes)]
            # # print(dyns.shape,(ego_lane==vehicle_lane).shape)
            # # dyns = dyns[ego_lane == vehicle_lanes]
            # # print(dyns.shape)
            # # print()
            # # if (type(paths) == type(None)):
            # return_dynamic_vehicles = return_dynamic_vehicles[np.logical_and(dist_dynamic<in_radius**2, ego_lane == vehicle_lanes)]

            # return_dynamic_vehicles = np.array(return_dynamic_vehicles)


            # dist_dynamic = np.sum(np.square(return_dynamic_vehicles-loc),axis =1 )
            # dist_temp = dist_dynamic
            # # print(dist_dynamic,"b")
            # # print(dist_dynamic<(in_radius**2))

            # if (intersection_state):
            #     dyns = vehicles[dist_dynamic<in_radius**2]
    
            # #     dyns = vehicles[np.logical_and(dist_dynamic<in_radius**2, ego_section == vehicle_sections)]                
            # #     return_dynamic_vehicles = return_dynamic_vehicles[np.logical_and(dist_dynamic<in_radius**2, ego_section == vehicle_sections)]


            #     # if (type(paths) == type(None)):
            #     #     dyns = vehicles[np.logical_and(dist_dynamic<in_radius**2, ego_section == vehicle_sections)]                
            #     #     return_dynamic_vehicles = return_dynamic_vehicles[np.logical_and(dist_dynamic<in_radius**2, ego_section == vehicle_sections)]

            #     #     # print("paths = None ",return_dynamic_vehicles)
            #     # else:
            #     #     dyns = vehicles[np.logical_and(dist_dynamic<in_radius**2,np.logical_or(ego_lane == vehicle_lanes, goal_lane == vehicle_lanes))]
            #     #     return_dynamic_vehicles = return_dynamic_vehicles[np.logical_and(dist_dynamic<in_radius**2,np.logical_or(ego_lane == vehicle_lanes, goal_lane == vehicle_lanes))]
            # else:

            if (type(paths) == type(None)):
                dyns = vehicles[np.logical_and(dist_dynamic<in_radius**2, ego_lane == vehicle_lanes)]                
                return_dynamic_vehicles = return_dynamic_vehicles[np.logical_and(dist_dynamic<in_radius**2, ego_lane == vehicle_lanes)]

                # dyns = vehicles[np.logical_and(dist_dynamic<in_radius**2, np.logical_and(ego_lane == vehicle_lanes,ego_section == vehicle_sections))]                
                # return_dynamic_vehicles = return_dynamic_vehicles[np.logical_and(dist_dynamic<in_radius**2, np.logical_and(ego_lane == vehicle_lanes,ego_section == vehicle_sections))]

                # print("paths = None ",return_dynamic_vehicles)
            else:
                
                dyns = vehicles[np.logical_and(dist_dynamic<in_radius**2,np.logical_or(ego_lane == vehicle_lanes, goal_lane == vehicle_lanes))]
                return_dynamic_vehicles = return_dynamic_vehicles[np.logical_and(dist_dynamic<in_radius**2,np.logical_or(ego_lane == vehicle_lanes, goal_lane == vehicle_lanes))]
                
                # vehicles_in_ego = np.logical_and(ego_lane == vehicle_lanes,ego_section==vehicle_sections)
                # vehicles_in_goal = np.logical_and(goal_lane == vehicle_lanes,goal_section == vehicle_sections)

                # dyns = vehicles[np.logical_and(dist_dynamic<in_radius**2,np.logical_or(vehicles_in_ego,vehicles_in_goal))]
                # return_dynamic_vehicles = return_dynamic_vehicles[np.logical_and(dist_dynamic<in_radius**2,np.logical_or(vehicles_in_ego,vehicles_in_goal))]


                # dyns = vehicles[np.logical_and(dist_dynamic<in_radius**2, ego_lane == vehicle_lanes)]
                # return_dynamic_vehicles = return_dynamic_vehicles[np.logical_and(dist_dynamic<in_radius**2, ego_lane == vehicle_lanes)]
            #     #print(np.logical_and(dist_dynamic<in_radius**2, ego_lane == vehicle_lanes),np.logical_and(dist_dynamic<in_radius**2, goal_lane == vehicle_lanes))
            #     temp_1 = return_dynamic_vehicles[np.logical_and(dist_dynamic<in_radius**2, ego_lane == vehicle_lanes)]
            #     temp_2 = return_dynamic_vehicles[np.logical_and(dist_dynamic<in_radius**2, goal_lane == vehicle_lanes)]
            #     return_dynamic_vehicles = temp_1 + temp_2
    
            # print(return_dynamic_vehicles.shape)
            #print("dyns", dyns)

            dist_dynamic,dynamic_closest = self.in_front(rot,loc,dyns,return_dynamic_vehicles)
            return_dynamic_vehicles = dyns
            # temp_ = np.amin(dist_dynamic[vehicle_lanes == ego_lane])
            # dyn_idx = np.where(dist_dynamic == temp_)
            # dist_dynamic = temp_ #closest dynamic distance squared
            return_dynamic_vehicles = vehicles[dist_temp<in_radius**2]

            
        
        if(return_static_vehicles!=[]):

            return_static_vehicles = np.array(return_static_vehicles)
            # print(return_static_vehicles.shape,loc.shape,return_static_vehicles[0])
            dist_static = np.sum(np.square(return_static_vehicles-loc),axis = 1)
            stat_temp = dist_static

            # stats = vehicles[np.logical_and(dist_static<in_radius**2, ego_lane == vehicle_lanes)]
            
            # #print(dist_static,"a")
            # return_static_vehicles = return_static_vehicles[np.logical_and(dist_static<in_radius**2, ego_lane == vehicle_lanes)]
            # return_static_vehicles = return_static_vehicles[]

            # temp_ = np.amin(dist_static[vehicle_lanes == ego_lane])
            # stat_idx = np.where(dist_static ==temp_)
            # dist_static = temp_  #closest static distance squared


            # if (intersection_state):
            #     stats = vehicles[dist_static<in_radius**2]
            # #     stats = vehicles[np.logical_and(dist_static<in_radius**2, ego_section == vehicle_sections)]
            # #     return_static_vehicles = return_static_vehicles[np.logical_and(dist_static<in_radius**2, ego_section == vehicle_sections)]

            # else:

            if (type(paths) == type(None)):
                stats = vehicles[np.logical_and(dist_static<in_radius**2, ego_lane == vehicle_lanes)]
                return_static_vehicles = return_static_vehicles[np.logical_and(dist_static<in_radius**2, ego_lane == vehicle_lanes)]

                # stats = vehicles[np.logical_and(dist_static<in_radius**2, np.logical_and(ego_lane == vehicle_lanes,ego_section == vehicle_sections))]
                # return_static_vehicles = return_static_vehicles[np.logical_and(dist_static<in_radius**2, np.logical_and(ego_lane == vehicle_lanes,ego_section == vehicle_sections))]

                # print("paths = None ",return_static_vehicles)
            else:
                # print(np.logical_and(dist_static<in_radius**2, ego_lane == vehicle_lanes),np.logical_and(dist_static<in_radius**2, goal_lane == vehicle_lanes))
                # stats = vehicles[np.logical_and(dist_static<in_radius**2, np.logical_or(ego_lane == vehicle_lanes, goal_lane == vehicle_lanes))]
                # return_static_vehicles = return_static_vehicles[np.logical_and(dist_static<in_radius**2, np.logical_or(ego_lane == vehicle_lanes, goal_lane == vehicle_lanes))]
                # stats = vehicles[np.logical_and(dist_static<in_radius**2, ego_lane == vehicle_lanes)]
                # return_static_vehicles = return_static_vehicles[np.logical_and(dist_static<in_radius**2, ego_lane == vehicle_lanes)]

                # stats = vehicles[np.logical_and(dist_static<in_radius**2, ego_lane == vehicle_lanes)]
                # return_static_vehicles = return_static_vehicles[np.logical_and(dist_static<in_radius**2, ego_lane == vehicle_lanes)]
                # vehicles_in_ego = np.logical_and(ego_lane == vehicle_lanes,ego_section==vehicle_sections)
                # vehicles_in_goal = np.logical_and(goal_lane == vehicle_lanes,goal_section == vehicle_sections)

                # stats = vehicles[np.logical_and(dist_static<in_radius**2, np.logical_or(vehicles_in_ego,vehicles_in_goal))]
                # return_static_vehicles = return_static_vehicles[np.logical_and(dist_static<in_radius**2, np.logical_or(vehicles_in_ego,vehicles_in_goal))]

                stats = vehicles[np.logical_and(dist_static<in_radius**2, np.logical_or(ego_lane == vehicle_lanes, goal_lane == vehicle_lanes))]
                return_static_vehicles = return_static_vehicles[np.logical_and(dist_static<in_radius**2, np.logical_or(ego_lane == vehicle_lanes, goal_lane == vehicle_lanes))]

            #print("stats", stats)
            dist_static,static_closest = self.in_front(rot,loc,stats,return_static_vehicles)
            return_static_vehicles = vehicles[stat_temp<in_radius**2]
            # temp_ = np.amin(dist_static[vehicle_lanes == ego_lane])
            # stat_idx = np.where(dist_static ==temp_)
            # dist_static = temp_  #closest static distance squared

            
        # print(dist_dynamic,dist_static)
        if(return_walkers!=[]):
            """
            Here we are sorting according to the y magnitude on the car frame, this could be implemented on the collission checker as wel to decrease the complexity
            """
            return_walkers = np.array(return_walkers)
            dist_walkers = np.sum(np.square(return_walkers-loc),axis = 1)
            
            crit = dist_walkers<in_radius**2

            # return_walkers = walkers[crit]
            walks = walkers[crit]



            car_frame = rot@((return_walkers[crit] - loc).T)
            # return_walkers = walkers[crit]

            crit = car_frame[1] - self.ego_vehicle.bounding_box.extent.y >= 0
            car_frame = car_frame[:,crit]
            

            car_frame = np.append(car_frame,[walks[crit]],axis = 0)
            car_frame = car_frame[:, car_frame[1].argsort(kind = "mergesort")]
            return_walkers = car_frame[2]
            walkers_y = car_frame[1]
        
        # print(dist_dynamic,dist_static)
        if (intersection_state):
            closest_vehicle = None
        else:
            if(dist_dynamic!=None and dist_static!=None):

                if(dist_dynamic<dist_static):
                    
                    # temp_ = vehicles[dyn_idx][0]
                    temp_ = dynamic_closest

                    if(temp_.id ==self.ego_vehicle.id):
                        closest_vehicle = None
                    else:
                        closest_vehicle = temp_

                else:

                    # temp_ = vehicles[dyn_idx][0]
                    temp_ = static_closest
                    if(temp_.id ==self.ego_vehicle.id):
                        closest_vehicle = None
                    else:
                        # closest_vehicle = vehicles[dyn_idx][0]
                        closest_vehicle = temp_

            elif(dist_dynamic!=None):
                # temp_ = vehicles[dyn_idx][0]
                temp_ = dynamic_closest
                if(temp_.id ==self.ego_vehicle.id):
                    closest_vehicle = None
                else:
                    # closest_vehicle = vehicles[dyn_idx][0]
                    closest_vehicle = temp_
            elif(dist_static!=None):
                # temp_ = vehicles[dyn_idx][0]
                temp_ = static_closest
                if(temp_.id ==self.ego_vehicle.id):
                    closest_vehicle = None
                else:
                    # closest_vehicle = vehicles[dyn_idx][0]
                    # closest_vehicle = vehicles[stat_idx][0]
                    closest_vehicle = temp_


        # closest_vehicle = min(dist_dynamic,dist_static)

        # print(closest_vehicle)
        # print(dist_dynamic,)
        return return_static_vehicles, return_dynamic_vehicles, return_walkers,closest_vehicle,x_vec,y_vec,walkers_y



'''import numpy as np
class Environment():
    def __init__(self, world,world_map, ego_vehicle):
        self.world = world
        self._world_map = world_map
        self.ego_vehicle = ego_vehicle
        self.ego_vehicle_loc = self.ego_vehicle.get_location()
        self._dynamic_vehicles = []
        self._static_vehicles = []
        self._walkers = []
        
    def get_actors(self,in_radius):
        actors = self.world.get_actors()
        vehicles = actors.filter('vehicle.*')
        walkers = actors.filter('walker.*')
        in_radius_sqr = np.square(in_radius)
        self.ego_vehicle_loc = self.ego_vehicle.get_location()
        return_dynamic_vehicles = []
        return_static_vehicles = []
        return_walkers = []
        static_vehicle_waypoints  = []
        dynamic_vehicle_waypoints = []
        walkers_waypoints         = []  
        for vehicle in vehicles:
            if self.ego_vehicle.id == vehicle.id:
                continue
            else:
                vehicle_loc = vehicle.get_location()
                vehicle_dist_sqr = (np.square(vehicle_loc.x - self.ego_vehicle_loc.x) + \
                                    np.square(vehicle_loc.y - self.ego_vehicle_loc.y) + \
                                    np.square(vehicle_loc.z - self.ego_vehicle_loc.z))
                if vehicle_dist_sqr < in_radius_sqr:
                    vehicle_velocity = vehicle.get_velocity()
                    vehicle_speed = np.linalg.norm(np.array([vehicle_velocity.x,vehicle_velocity.y,vehicle_velocity.z]))
                    if vehicle_speed == 0.0:
                        static_vehicle_waypoints.append(self._world_map.get_waypoint(self.vehicle_loc))
                        return_static_vehicles.append(vehicle)
                    else:
                        dynamic_vehicle_waypoints.append(self._world_map.get_waypoint(self.vehicle_loc))
                        return_dynamic_vehicles.append(vehicle)
        for walker in walkers:
            walker_loc = walker.get_location()
            walker_dist_sqr =  (np.square(walker_loc.x - self.ego_vehicle_loc.x) + \
                                np.square(walker_loc.y - self.ego_vehicle_loc.y) + \
                                np.square(walker_loc.z - self.ego_vehicle_loc.z))
            if walker_dist_sqr < in_radius_sqr:
                walkers_waypoints.append(self._world_map.get_waypoint(walker_loc))
                return_walkers.append(walker)
        self._static_vehicles = return_static_vehicles
        self._dynamic_vehicles = return_dynamic_vehicles
        self._walkers = return_walkers
        return return_static_vehicles, return_dynamic_vehicles, return_walkers
    def lane_front_obstacles():
        ego_waypoint=self._world_map.get_waypoint(self.ego_vehicle_loc)
        for static_vehicle in self._static_vehicles:
            if (static_waypoint.lane_id== ego_waypoint.lane_id):
'''          