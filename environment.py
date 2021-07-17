import numpy as np
from tools.misc import draw_bound_box_actor, draw_bound_box
import time
from collections import defaultdict
from os_carla import *
import sys
import glob


LANE_CHANGE_DIST = 15

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
elif SAUMYA_UBUNTU:
    try:
        sys.path.append(glob.glob('/home/pq-saumya/Documents/CARLA_0.9.9/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg' )[0])
    except IndexError:
        pass

else:
    try:
        sys.path.append(glob.glob('/home/selfdriving/carla-precompiled/CARLA_0.9.9/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg' )[0])
    except IndexError:
        pass

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
        self.traf_light_dict = {1260:[carla.Location(x=  17.250000, y= 177.399994, z= 0.000000),carla.Location(x=  15.650000, y= 197.949997, z=0.000000),carla.Location(x=  40.849998, y= 216.349991, z=0.000000)],
                                1070:[carla.Location(x=  19.150000, y= 101.699997, z= 0.000000),carla.Location(x=  18.600000, y=  79.750000, z=0.000000),carla.Location(x=  42.549999, y= 100.199997, z=0.000000),carla.Location(x=  40.250000, y=  78.599998, z=0.000000)],
                                829: [carla.Location(x=  40.899998, y=  10.500000, z= 0.000000),carla.Location(x=  40.250000, y= -11.099999, z=0.000000),carla.Location(x=  18.600000, y=  -9.950000, z=0.000000),carla.Location(x=  19.150000, y=  12.000000, z=0.000000)],
                                751 :[carla.Location(x=  41.399998, y= -78.099998, z= 0.000000),carla.Location(x=  41.849998, y=-100.199997, z=0.000000),carla.Location(x=  20.250000, y= -99.900002, z=0.000000),carla.Location(x=  21.250000, y= -78.849998, z=0.000000)],
                                1148:[carla.Location(x=  43.899998, y=-158.199997, z= 0.200000)],
                                943 :[carla.Location(x=  46.349998, y=-178.500000, z= 0.000000),carla.Location(x=  47.950001, y=-197.349991, z=0.000000),carla.Location(x=  22.750000, y=-215.149994, z=0.000000)],
                                139 :[carla.Location(x= -38.649998, y= 100.049995, z= 0.000000),carla.Location(x= -37.450001, y=  78.000000, z=0.000000),carla.Location(x= -61.250000, y=  76.699997, z=0.000000),carla.Location(x= -61.349998, y= 101.250000, z=0.000000)],
                                965 :[carla.Location(x= -38.549999, y=  11.599999, z= 0.000000),carla.Location(x= -38.599998, y=  -9.650000, z=0.000000),carla.Location(x= -60.699997, y=  -9.500000, z=0.000000),carla.Location(x= -59.599998, y=  12.450000, z=0.000000)],
                                599 :[carla.Location(x= -39.599998, y= -78.099998, z= 0.000000),carla.Location(x= -38.849998, y=-101.299995, z=0.000000),carla.Location(x= -62.799999, y=-101.799995, z=0.000000),carla.Location(x= -61.149998, y= -78.099998, z=0.000000)],
                                1050:[carla.Location(x=-112.299995, y= 159.399994, z= 0.000000),carla.Location(x=-138.500000, y= 137.550003, z=0.000000),carla.Location(x=-138.800003, y= 159.550003, z=0.000000)],
                                1175:[carla.Location(x=-112.299995, y= 101.099998, z= 0.000000),carla.Location(x=-113.399994, y=  78.000000, z=0.000000),carla.Location(x=-139.800003, y=  77.849998, z=0.000000),carla.Location(x=-138.800003, y= 100.049995, z=0.000000)],
                                509 :[carla.Location(x=-113.149994, y=  11.599999, z= 0.000000),carla.Location(x=-113.899994, y=  -9.650000, z=0.000000),carla.Location(x=-139.349991, y= -10.300000, z=0.000000),carla.Location(x=-140.399994, y=  12.450000, z=0.000000)],
                                53  :[carla.Location(x=-113.149994, y= -78.949997, z= 0.000000),carla.Location(x=-112.799995, y=-101.299995, z=0.000000),carla.Location(x=-139.349991, y=-100.849998, z=0.000000),carla.Location(x=-140.399994, y= -78.099998, z=0.000000)],
                                905 :[carla.Location(x=-112.966469, y=-128.849045, z=-0.000005),carla.Location(x=-112.765907, y=-150.949997, z=0.000000),carla.Location(x=-143.038879, y=-150.019562, z=0.000000)],
                                421 :[carla.Location(x=-179.349991, y=  12.450000, z= 0.000000),carla.Location(x=-179.250000, y=  -9.700000, z=0.000000),carla.Location(x=-201.399994, y= -10.300000, z=0.000000),carla.Location(x=-201.550003, y=  12.450000, z=0.000000)]}

                                # 1148 remaining color light post:carla.Location(x=  23.327223, y=-154.399994, z=0.100000)
        self.traf_actor_dict = defaultdict(list)
        for light_actor in self.lights_list:
            light_loc = light_actor.get_location()
            for junc_id in self.traf_light_dict.keys():
                for locs in self.traf_light_dict[junc_id]:
                    if ((((locs.x-light_loc.x)**2)+((locs.y-light_loc.y)**2)+((locs.z-light_loc.z)**2))<1):
                        self.traf_actor_dict[junc_id].append(light_actor)
        # for a in self.traf_actor_dict:
        #     print(a,self.traf_actor_dict[a])
        # print(self.traf_actor_dict)
            


        # for light in self.lights_list:
        #     world.debug.draw_string(light.get_location(), str(light.id), draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=10000,persistent_lines=True)
        #     print(str(light.id),light.get_location())
        # traf_loc  =[[carla.Location(x=  17.250000, y= 177.399994, z= 0.000000),carla.Location(x=  15.650000, y= 197.949997, z=0.000000),carla.Location(x=  40.849998, y= 216.349991, z=0.000000)],
        #             [carla.Location(x=  19.150000, y= 101.699997, z= 0.000000),carla.Location(x=  18.600000, y=  79.750000, z=0.000000),carla.Location(x=  42.549999, y= 100.199997, z=0.000000),carla.Location(x=  40.250000, y=  78.599998, z=0.000000)],
        #             [carla.Location(x=  40.899998, y=  10.500000, z= 0.000000),carla.Location(x=  40.250000, y= -11.099999, z=0.000000),carla.Location(x=  18.600000, y=  -9.950000, z=0.000000),carla.Location(x=  19.150000, y=  12.000000, z=0.000000)],
        #             [carla.Location(x=  41.399998, y= -78.099998, z= 0.000000),carla.Location(x=  41.849998, y=-100.199997, z=0.000000),carla.Location(x=  20.250000, y= -99.900002, z=0.000000),carla.Location(x=  21.250000, y= -78.849998, z=0.000000)],
        #             [carla.Location(x=  43.899998, y=-158.199997, z= 0.200000),carla.Location(x=  23.327223, y=-154.399994, z=0.100000)],
        #             [carla.Location(x=  46.349998, y=-178.500000, z= 0.000000),carla.Location(x=  47.950001, y=-197.349991, z=0.000000),carla.Location(x=  22.750000, y=-215.149994, z=0.000000)],
        #             [carla.Location(x= -38.649998, y= 100.049995, z= 0.000000),carla.Location(x= -37.450001, y=  78.000000, z=0.000000),carla.Location(x= -61.250000, y=  76.699997, z=0.000000),carla.Location(x= -61.349998, y= 101.250000, z=0.000000)],
        #             [carla.Location(x= -38.549999, y=  11.599999, z= 0.000000),carla.Location(x= -38.599998, y=  -9.650000, z=0.000000),carla.Location(x= -60.699997, y=  -9.500000, z=0.000000),carla.Location(x= -59.599998, y=  12.450000, z=0.000000)],
        #             [carla.Location(x= -39.599998, y= -78.099998, z= 0.000000),carla.Location(x= -38.849998, y=-101.299995, z=0.000000),carla.Location(x= -62.799999, y=-101.799995, z=0.000000),carla.Location(x= -61.149998, y= -78.099998, z=0.000000)],
        #             [carla.Location(x=-112.299995, y= 159.399994, z= 0.000000),carla.Location(x=-138.500000, y= 137.550003, z=0.000000),carla.Location(x=-138.800003, y= 159.550003, z=0.000000)],
        #             [carla.Location(x=-112.299995, y= 101.099998, z= 0.000000),carla.Location(x=-113.399994, y=  78.000000, z=0.000000),carla.Location(x=-139.800003, y=  77.849998, z=0.000000),carla.Location(x=-138.800003, y= 100.049995, z=0.000000)],
        #             [carla.Location(x=-113.149994, y=  11.599999, z= 0.000000),carla.Location(x=-113.899994, y=  -9.650000, z=0.000000),carla.Location(x=-139.349991, y= -10.300000, z=0.000000),carla.Location(x=-140.399994, y=  12.450000, z=0.000000)],
        #             [carla.Location(x=-113.149994, y= -78.949997, z= 0.000000),carla.Location(x=-112.799995, y=-101.299995, z=0.000000),carla.Location(x=-139.349991, y=-100.849998, z=0.000000),carla.Location(x=-140.399994, y= -78.099998, z=0.000000)],
        #             [carla.Location(x=-112.966469, y=-128.849045, z=-0.000005),carla.Location(x=-112.765907, y=-150.949997, z=0.000000),carla.Location(x=-143.038879, y=-150.019562, z=0.000000)],
        #             [carla.Location(x=-179.349991, y=  12.450000, z= 0.000000),carla.Location(x=-179.250000, y=  -9.700000, z=0.000000),carla.Location(x=-201.399994, y= -10.300000, z=0.000000),carla.Location(x=-201.550003, y=  12.450000, z=0.000000)]]
        # for j in traf_loc:
        #     _x = 0
        #     _y = 0
        #     _z = 0
        #     for i in j:
        #         _x += i.x
        #         _y += i.y
        #         _z += i.z
        #     mean_loc = carla.Location(x=_x/len(j),y=_y/len(j),z=_z/len(j))
        #     wpt = map_.get_waypoint(mean_loc,project_to_road=True)
        #     if wpt.is_junction:
        #         print(wpt.get_junction(),wpt.get_junction().id)
        #     else:
        #         print("no_juntion")
            # world.debug.draw_string(mean_loc, 'M', draw_shadow=False,color=carla.Color(r=255, g=255, b=255), life_time=100000,persistent_lines=True)
            # time.sleep(5)

        self.vehicles = self.actors.filter('vehicle.*')
        self.walkers = self.actors.filter('walker.*.*')
        # for w in (self.walkers ):
        #         draw_bound_box_actor(w,self.world,0,255,0)

        # self.actors = self.world.get_actors()
        # self.vehicles = self.actors.filter('vehicle.*')
        # self.walkers = self.actors.filter('walker.*')
        # self.first_time = True
        # print(len(self.lights_list),"A")
    def in_front(self,rot,ego_loc,actors,actors_loc):

        # print(actors)
        # print(rot.shape,actors.shape,ego_loc.shape,actors_loc.shape)

        car_frame = rot.T@((actors_loc - ego_loc).T)

        # return_walkers = walkers[crit]
        crit = car_frame[0] - self.ego_vehicle.bounding_box.extent.x >= 0
        car_frame = car_frame[:,crit]

        #print(car_frame)
        car_frame = np.append(car_frame,[actors[crit]],axis = 0)
        car_frame = car_frame[:,car_frame[0].argsort(kind = "mergesort")]

        # print(car_frame.shape,"HUUUU")
        return_actors = car_frame[2]
        actors_x = car_frame[0]

        # print(return_actors,actors_y)
        if(car_frame.shape[1] == 0):
            return None,None
        else:
            return actors_x[0],return_actors[0]

    # def in_front(self,rot,ego_loc,actors,actors_loc):

    #     # print(rot.shape,actors.shape,ego_loc.shape,actors_loc.shape)

    #     car_frame = rot@((actors_loc - ego_loc).T)
    #     # return_walkers = walkers[crit]
    #     print(car_frame)
    #     crit = car_frame[1] - self.ego_vehicle.bounding_box.extent.y >= 0
    #     car_frame = car_frame[:,crit]

    #     car_frame = np.append(car_frame,[actors[crit]],axis = 0)
    #     car_frame = car_frame[:, car_frame[1].argsort(kind = "mergesort")]

    #     # print(car_frame.shape,"HUUUU")
    #     return_actors = car_frame[2]
    #     actors_y = car_frame[1]

    #     # print(return_actors,actors_y)
    #     if(car_frame.shape[1] == 0):
    #         return None,None
    #     else:
    #         return actors_y[0],return_actors[0]

    def get_actors(self,in_radius,paths,middle_path_idx, intersection_state, intersection_waypoint,overtake_vehicle,lane_change_vehicle,danger_vehicle,jaywalking_ped,school_ped):
        
        # if(self.first_time):
        # 
        # #### This can be removed by taking the spawn NPC code within the new_main
        

        # self.first_time = False
        #print(np.shape(paths))
        vehicles = np.array(self.vehicles)
        walkers = np.array(self.walkers)
        if not (overtake_vehicle is None):
            vehicles=np.append(vehicles,overtake_vehicle) 
        if not (lane_change_vehicle is None):
            vehicles=np.append(vehicles,lane_change_vehicle) 

        if not (danger_vehicle is None):
            vehicles=np.append(vehicles,danger_vehicle) 
        if not (jaywalking_ped is None):
            walkers=np.append(walkers,jaywalking_ped) 
        if not (school_ped is None):
            walkers=np.append(walkers,school_ped)
        
        # for w in (walkers ):
        #         draw_bound_box_actor(w,self.world,0,255,0)
        
        in_radius_sqr = np.square(in_radius)
        self.ego_vehicle_loc = self.ego_vehicle.get_location()
        self.yaw = np.radians(self.ego_vehicle.get_transform().rotation.yaw)

        return_dynamic_vehicles = []
        return_static_vehicles = []
        return_walkers = []
        vehicle_lanes = []
        vehicle_sections = []
        veh_road_data = []

        vehicle_locs = []
        vehicle_front_locs = np.empty((0,2),dtype=np.float32)
        vehicle_backk_locs = np.empty((0,2),dtype=np.float32)

        ego_waypoint=self._map.get_waypoint(self.ego_vehicle_loc,project_to_road=True,lane_type=carla.LaneType.Driving)
        ego_lane = ego_waypoint.lane_id
        ego_section = ego_waypoint.section_id
        goal_lane =None

        if (type(paths) == type(None)):
            pass
        else:
            middle_path_idx = paths.shape[0]//2
            goal_location = carla.Location(x=paths[middle_path_idx,0,-1], y=paths[middle_path_idx,1,-1], z= 1.843102 )
            goal_waypoint=self._map.get_waypoint(goal_location,project_to_road=True,lane_type=carla.LaneType.Driving)
            goal_lane = goal_waypoint.lane_id
            goal_section = goal_waypoint.section_id
        
            
        distance = 10**4
        closest_vehicle = None

        # vehicle_loc = []
        # walker loc = []
        # print(vehicles)
        for vehicle in vehicles:
            if self.ego_vehicle.id == vehicle.id:
                return_dynamic_vehicles.append([10000,10000])
                return_static_vehicles.append([10000,10000])
                vehicle_lanes.append(ego_lane)
                vehicle_sections.append(ego_section)

                veh_road_data.append([ego_lane,ego_section,ego_waypoint.road_id])

                vehicle_locs.append([10000,10000])
                vehicle_front_locs = np.append(vehicle_front_locs,np.array([[100000,100000]]),axis=0)
                vehicle_backk_locs = np.append(vehicle_backk_locs,np.array([[100000,100000]]),axis=0)

                # continue
            else:
                
                # print("a")
                vehicle_loc = vehicle.get_location()
                vehicle_vel = vehicle.get_velocity()
                vehicle_waypoint = self._map.get_waypoint(vehicle_loc,project_to_road=True,lane_type=carla.LaneType.Driving)

                vehicle_lane = vehicle_waypoint.lane_id
                vehicle_section = vehicle_waypoint.section_id
                vehicle_road    = vehicle_waypoint.road_id

                veh_road_data.append([vehicle_lane,vehicle_section,vehicle_road])

                vehicle_lanes.append(vehicle_lane)
                vehicle_sections.append(vehicle_section)

                locs = [vehicle_loc.x,vehicle_loc.y]
                vels = np.array([vehicle_vel.x,vehicle_vel.y,vehicle_vel.z])
                vehicle_locs.append([vehicle_loc.x,vehicle_loc.y])
                

                ################### intersection closest vehicle handling #############################################
                
                veh_loc = np.array([[vehicle.get_transform().location.x,vehicle.get_transform().location.y]])

                yaw = np.radians(vehicle.get_transform().rotation.yaw)

                if(yaw>np.pi):
                    yaw-=2*np.pi
                elif(yaw<-np.pi):
                    yaw+=2*np.pi
                    
                rot = np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])

                x_vec = rot@np.array([[1],[0]])
                x_vec = x_vec.reshape((2,))

                y_vec = rot@np.array([[0],[1]])
                y_vec = y_vec.reshape((2,))
                # # print(y_vec)
                # print(np.array(locs))

                # SS = np.array(locs) + vehicle.bounding_box.extent.x*x_vec
                # SS_1 = np.array(locs) - vehicle.bounding_box.extent.x*x_vec
                # # print(yaw)
                # self.world.debug.draw_string(carla.Location(x=SS[0],y = SS[1],z = 0),"0", draw_shadow=False,color=carla.Color(r=255, g=255, b=255), life_time=100,persistent_lines=True)    
                # self.world.debug.draw_string(carla.Location(x=SS_1[0],y = SS_1[1],z = 0),"0", draw_shadow=False,color=carla.Color(r=255, g=255, b=255), life_time=100,persistent_lines=True)    
     
                offset = vehicle.bounding_box.extent.x*x_vec
                # SS = np.array(locs) + offset
                # SA = np.array(locs) - offset
                # self.world.debug.draw_string(carla.Location(x=SS[0],y = SS[1],z = 1.843),"0", draw_shadow=False,color=carla.Color(r=255, g=255, b=255), life_time=30,persistent_lines=True)    
                # self.world.debug.draw_string(carla.Location(x=SA[0],y = SA[1],z = 1.843),"0", draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=30,persistent_lines=True)    
                
                # # print(offset.shape)
                vehicle_front_locs = np.append(vehicle_front_locs,veh_loc+offset,axis=0)
                vehicle_backk_locs = np.append(vehicle_backk_locs,veh_loc-offset,axis=0)
                ###############################################################################################################

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
 
        walkers = np.array(walkers)
        walkers_y = None
        walkers_x = None
        rot = np.array([[np.cos(self.yaw),-np.sin(self.yaw)],[np.sin(self.yaw),np.cos(self.yaw)]])

        x_vec = rot@np.array([[1],[0]])
        x_vec = x_vec.reshape((2,))

        y_vec = rot@np.array([[0],[1]])
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
                crit_1 = np.logical_and(dist_dynamic<in_radius**2, ego_lane == vehicle_lanes)
                crit_2 = dist_dynamic<LANE_CHANGE_DIST**2

                dyns = vehicles[crit_1]                
                return_dynamic_vehicles = return_dynamic_vehicles[crit_1]
                
                dyn_lane_chng_dist = dist_dynamic[crit_2] 

                lane_change_dyn_veh = vehicles[crit_2]

                # dyns = vehicles[np.logical_and(dist_dynamic<in_radius**2, np.logical_and(ego_lane == vehicle_lanes,ego_section == vehicle_sections))]                
                # return_dynamic_vehicles = return_dynamic_vehicles[np.logical_and(dist_dynamic<in_radius**2, np.logical_and(ego_lane == vehicle_lanes,ego_section == vehicle_sections))]

                # print("paths = None ",return_dynamic_vehicles)
            else:
                crit_1 = np.logical_and(dist_dynamic<in_radius**2,np.logical_or(ego_lane == vehicle_lanes, goal_lane == vehicle_lanes))
                crit_2 = dist_dynamic<LANE_CHANGE_DIST**2

                dyns = vehicles[crit_1]
                return_dynamic_vehicles = return_dynamic_vehicles[crit_1]
                
                dyn_lane_chng_dist = dist_dynamic[crit_2]

                lane_change_dyn_veh = vehicles[crit_2]

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
            # return_dynamic_vehicles = dyns
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

                crit_3 = dist_static<LANE_CHANGE_DIST**2
                stat_lane_chng_dist = dist_static[crit_3]
                lane_change_stat_veh = vehicles[crit_3]

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

                crit_3 = dist_static<LANE_CHANGE_DIST**2
                stat_lane_chng_dist = dist_static[crit_3]
                lane_change_stat_veh = vehicles[crit_3]
                
            #print("stats", stats)
            dist_static,static_closest = self.in_front(rot,loc,stats,return_static_vehicles)
            return_static_vehicles = vehicles[stat_temp<in_radius**2]


            
            # return_static_vehicles = stats
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
            car_frame = rot.T@((return_walkers[crit] - loc).T)
            # return_walkers = walkers[crit]

            crit = car_frame[0] - self.ego_vehicle.bounding_box.extent.x >= 0
            car_frame = car_frame[:,crit]
            

            car_frame = np.append(car_frame,[walks[crit]],axis = 0)
            car_frame = car_frame[:, car_frame[0].argsort(kind = "mergesort")]
            return_walkers = car_frame[2]
            walkers_y = car_frame[0]
            walkers_x = car_frame[1]
        
        if (intersection_state):
            # for i in range(intersection_waypoint.shape[0]):
            #     self.world.debug.draw_string(carla.Location(x=intersection_waypoint[i,0],y = intersection_waypoint[i,1],z = 1),"0", draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=30,persistent_lines=True)    
            vehicle_locs = np.array(vehicle_locs)
            dist_vehicles = np.sum(np.square(vehicle_locs-loc),axis = 1)
            intersection_vehicles = vehicles[dist_vehicles<in_radius**2]
            # intersection_vehicle_locs = vehicle_locs[dist_vehicles<in_radius**2]      
            # intersection_vehicles_dist = np.sqrt(np.square(intersection_waypoint[:,0].reshape((-1,1)) - intersection_vehicle_locs[:,0].reshape((1,-1))) + np.square(intersection_waypoint[:,1].reshape((-1,1)) - intersection_vehicle_locs[:,1].reshape((1,-1))))
            # indx_list = np.argwhere(intersection_vehicles_dist < 3.7/2)

            # if (indx_list.size!=0):
            #     closest_waypoint = indx_list[0][0]
            #     vehicle_index  = np.argwhere(intersection_vehicles_dist[closest_waypoint,:]==np.amin(intersection_vehicles_dist[closest_waypoint,:]))
            #     vehicle_index = vehicle_index.flatten()
            #     intersection_vehicles = list(intersection_vehicles)
            #     draw_bound_box_actor(intersection_vehicles[vehicle_index[0]], self.world,255,255,255)
            #     closest_vehicle = intersection_vehicles[vehicle_index[0]]
            #     # print("Check this function when debugging. There can be issues")
            #     # closest_vehicle = None
            # else:
            #     closest_vehicle = None

            # # for k in range(vehicle_front_locs.shape[0]):
                
            # #     self.world.debug.draw_string(carla.Location(x=vehicle_front_locs[k,0],y = vehicle_front_locs[k,1],z = 1.843),"0", draw_shadow=False,color=carla.Color(r=255, g=255, b=255), life_time=30,persistent_lines=True)    
            # #     self.world.debug.draw_string(carla.Location(x=vehicle_backk_locs[k,0],y = vehicle_backk_locs[k,1],z = 1.843),"0", draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=30,persistent_lines=True)    


            intersection_vehicle_locs_Fnt = vehicle_front_locs[dist_vehicles<in_radius**2]  
            intersection_vehicle_locs_Bck = vehicle_backk_locs[dist_vehicles<in_radius**2] 
            intersection_vehicles_dist_Fnt = np.sqrt(np.square(intersection_waypoint[:,0].reshape((-1,1)) - intersection_vehicle_locs_Fnt[:,0].reshape((1,-1))) + np.square(intersection_waypoint[:,1].reshape((-1,1)) - intersection_vehicle_locs_Fnt[:,1].reshape((1,-1))))
            intersection_vehicles_dist_Bck = np.sqrt(np.square(intersection_waypoint[:,0].reshape((-1,1)) - intersection_vehicle_locs_Bck[:,0].reshape((1,-1))) + np.square(intersection_waypoint[:,1].reshape((-1,1)) - intersection_vehicle_locs_Bck[:,1].reshape((1,-1))))
            indx_list_Fnt = np.argwhere(intersection_vehicles_dist_Fnt < 3.7/2)
            indx_list_Bck = np.argwhere(intersection_vehicles_dist_Bck < 3.7/2)
            closest_waypoint_Fnt = None
            closest_waypoint_Bck = None

            if (indx_list_Fnt.size!=0):
                closest_waypoint_Fnt = indx_list_Fnt[0][0]

            if (indx_list_Bck.size!=0):
                closest_waypoint_Bck = indx_list_Bck[0][0]

            if ((closest_waypoint_Fnt is None) and (closest_waypoint_Bck is None)):
                closest_vehicle = None

            elif (closest_waypoint_Fnt is None):
                vehicle_index  = np.argwhere(intersection_vehicles_dist_Bck[closest_waypoint_Bck,:]==np.amin(intersection_vehicles_dist_Bck[closest_waypoint_Bck,:]))
                vehicle_index = vehicle_index.flatten()
                intersection_vehicles = list(intersection_vehicles)
                draw_bound_box_actor(intersection_vehicles[vehicle_index[0]], self.world,255,255,255)
                closest_vehicle = intersection_vehicles[vehicle_index[0]]

            elif (closest_waypoint_Bck is None):
                vehicle_index  = np.argwhere(intersection_vehicles_dist_Fnt[closest_waypoint_Fnt,:]==np.amin(intersection_vehicles_dist_Fnt[closest_waypoint_Fnt,:]))
                vehicle_index = vehicle_index.flatten()
                intersection_vehicles = list(intersection_vehicles)
                draw_bound_box_actor(intersection_vehicles[vehicle_index[0]], self.world,255,255,255)
                closest_vehicle = intersection_vehicles[vehicle_index[0]]
            
            else:
                if (closest_waypoint_Fnt <= closest_waypoint_Bck):
                    vehicle_index  = np.argwhere(intersection_vehicles_dist_Fnt[closest_waypoint_Fnt,:]==np.amin(intersection_vehicles_dist_Fnt[closest_waypoint_Fnt,:]))
                    vehicle_index = vehicle_index.flatten()
                    intersection_vehicles = list(intersection_vehicles)
                    draw_bound_box_actor(intersection_vehicles[vehicle_index[0]], self.world,255,255,255)
                    closest_vehicle = intersection_vehicles[vehicle_index[0]]

                else:
                    vehicle_index  = np.argwhere(intersection_vehicles_dist_Bck[closest_waypoint_Bck,:]==np.amin(intersection_vehicles_dist_Bck[closest_waypoint_Bck,:]))
                    vehicle_index = vehicle_index.flatten()
                    intersection_vehicles = list(intersection_vehicles)
                    draw_bound_box_actor(intersection_vehicles[vehicle_index[0]], self.world,255,255,255)
                    closest_vehicle = intersection_vehicles[vehicle_index[0]]

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
        veh_road_data = np.array(veh_road_data)
        return_stat_lanes = veh_road_data[stat_temp<LANE_CHANGE_DIST**2] 
        return_dyn_lanes  = veh_road_data[dist_temp<LANE_CHANGE_DIST**2]

        # print(return_dynamic_vehicles)
        return return_static_vehicles, return_dynamic_vehicles, return_walkers,closest_vehicle,x_vec,y_vec,walkers_y,walkers_x,return_stat_lanes,return_dyn_lanes,ego_lane,goal_lane,dyn_lane_chng_dist,stat_lane_chng_dist,lane_change_dyn_veh,lane_change_stat_veh


    def get_overtake_actors(self, _waypoint, thresholdSqr, ego_actor):
        vehicles = np.array(self.world.get_actors().filter('vehicle.*' ))
        walkers  = np.array(self.world.get_actors().filter('walker.*.*'))

        vehicle_locs = np.empty((0,2),dtype=np.float32)
        vehicle_front_locs = np.empty((0,2),dtype=np.float32)
        vehicle_backk_locs = np.empty((0,2),dtype=np.float32)
        
        for veh in vehicles:
            if (veh.id != ego_actor.id):
                veh_loc = np.array([[veh.get_transform().location.x,veh.get_transform().location.y]])

                yaw = np.radians(veh.get_transform().rotation.yaw)
                rot = np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
                # print(np.array[[veh.bounding_box.extent.x],[veh.bounding_box.extent.y]])

                ####Gershom was here



                offset = (rot@ np.array([[veh.bounding_box.extent.x],[veh.bounding_box.extent.y]])).T
                # print(offset.shape)

                # vehicle_locs = np.append(vehicle_locs,veh_loc,axis=0)
                vehicle_front_locs = np.append(vehicle_front_locs,veh_loc+offset,axis=0)
                vehicle_backk_locs = np.append(vehicle_backk_locs,veh_loc-offset,axis=0)
            else:
                # vehicle_locs = np.append(vehicle_locs,np.array([[100000,100000]]),axis=0)
                vehicle_front_locs = np.append(vehicle_front_locs,np.array([[100000,100000]]),axis=0)
                vehicle_backk_locs = np.append(vehicle_backk_locs,np.array([[100000,100000]]),axis=0)

        # closestVehDist = np.amin(np.square(_waypoint[:,0].reshape((-1,1)) - vehicle_locs[:,0].reshape((1,-1))) + np.square(_waypoint[:,1].reshape((-1,1)) - vehicle_locs[:,1].reshape((1,-1))),axis=0)
        # vehActorsInTresh = vehicles[closestVehDist<thresholdSqr].tolist()
        closestVehDist_Fnt = np.amin(np.square(_waypoint[:,0].reshape((-1,1)) - vehicle_front_locs[:,0].reshape((1,-1))) + np.square(_waypoint[:,1].reshape((-1,1)) - vehicle_front_locs[:,1].reshape((1,-1))),axis=0)
        closestVehDist_Bck = np.amin(np.square(_waypoint[:,0].reshape((-1,1)) - vehicle_backk_locs[:,0].reshape((1,-1))) + np.square(_waypoint[:,1].reshape((-1,1)) - vehicle_backk_locs[:,1].reshape((1,-1))),axis=0)
        vehActorsInTresh_Fnt = vehicles[closestVehDist_Fnt<thresholdSqr].tolist()
        vehActorsInTresh_Bck = vehicles[closestVehDist_Bck<thresholdSqr].tolist()
        vehActorsInTresh = list(set(vehActorsInTresh_Fnt+vehActorsInTresh_Bck))
        
        walker_locs  = np.empty((0,2),dtype=np.float32)
        for wkr in walkers:
            walker_locs  = np.append(walker_locs ,np.array([[wkr.get_transform().location.x,wkr.get_transform().location.y]]),axis=0)
        closestWkrDist = np.amin(np.square(_waypoint[:,0].reshape((-1,1)) - walker_locs [:,0].reshape((1,-1))) + np.square(_waypoint[:,1].reshape((-1,1)) - walker_locs [:,1].reshape((1,-1))),axis=0)
        wkrActorsInTresh = walkers[closestWkrDist<thresholdSqr].tolist()

        return vehActorsInTresh, wkrActorsInTresh

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