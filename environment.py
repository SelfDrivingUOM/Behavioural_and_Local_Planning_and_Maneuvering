import numpy as np
import carla

class Environment():
    def __init__(self, world, ego_vehicle,map_):
        self.world = world
        self.ego_vehicle = ego_vehicle
        self.ego_vehicle_loc = self.ego_vehicle.get_location()
        # self.distance = 10**4
        self._map = map_

    def get_actors(self,in_radius):
        actors = self.world.get_actors()
        vehicles = actors.filter('vehicle.*')
        walkers = actors.filter('walker.*')

        # in_radius_sqr = np.square(in_radius)
        self.ego_vehicle_loc = self.ego_vehicle.get_location()

        return_dynamic_vehicles = []
        return_static_vehicles = []
        return_walkers = []


        ego_waypoint=self._map.get_waypoint(self.ego_vehicle_loc,project_to_road=True,lane_type=carla.LaneType.Driving)
        ego_lane = ego_waypoint.lane_id
        
        distance = 10**4
        closest_vehicle = None
        for vehicle in vehicles:
            if self.ego_vehicle.id == vehicle.id:
                continue
            else:
                

                vehicle_loc = vehicle.get_location()
                vehicle_dist_sqr = np.sqrt((np.square(vehicle_loc.x - self.ego_vehicle_loc.x) + \
                                    np.square(vehicle_loc.y - self.ego_vehicle_loc.y) + \
                                    np.square(vehicle_loc.z - self.ego_vehicle_loc.z)))
                if vehicle_dist_sqr < in_radius:
                    vehicle_velocity = vehicle.get_velocity()
                    vehicle_speed = np.linalg.norm(np.array([vehicle_velocity.x,vehicle_velocity.y,vehicle_velocity.z]))
                    vehicle_lane = self._map.get_waypoint(vehicle_loc,project_to_road=True,lane_type=carla.LaneType.Driving).lane_id

                    if(vehicle_dist_sqr<distance  and ego_lane == vehicle_lane):
                        closest_vehicle = vehicle
                        distance = vehicle_dist_sqr

                    if vehicle_speed == 0.0:
                        return_static_vehicles.append(vehicle)
                    else:
                        return_dynamic_vehicles.append(vehicle)

        for walker in walkers:
            walker_loc = walker.get_location()
            walker_dist_sqr =  (np.square(walker_loc.x - self.ego_vehicle_loc.x) + \
                                np.square(walker_loc.y - self.ego_vehicle_loc.y) + \
                                np.square(walker_loc.z - self.ego_vehicle_loc.z))
            if walker_dist_sqr < in_radius_sqr:
                return_walkers.append(walker)

        return return_static_vehicles, return_dynamic_vehicles, return_walkers,closest_vehicle

    





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

    

