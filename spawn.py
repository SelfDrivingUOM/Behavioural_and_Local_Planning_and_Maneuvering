#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Spawn NPCs into the simulation"""

import glob
import os
import sys
import time
import numpy as np
from os_carla import WINDOWS

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

import carla
import argparse
import logging
import random

class Environment():
    def __init__(self, world, ego_vehicle):
        self.world = world
        self.ego_vehicle = ego_vehicle

    def get_actors(self,in_radius):

        actors = self.world.get_actors()
        vehicles = actors.filter('vehicle.*')
        walkers = actors.filter('walker.*')

        in_radius_sqr = np.square(in_radius)
        ego_vehicle_loc = self.ego_vehicle.get_location()
        
        return_vehicles = []
        return_walkers = []

        for vehicle in vehicles:
            if self.ego_vehicle.id == vehicle.id:
                continue
            else:
                vehicle_loc = vehicle.get_location()
                vehicle_dist_sqr = (np.square(vehicle_loc.x - ego_vehicle_loc.x) + \
                                    np.square(vehicle_loc.y - ego_vehicle_loc.y) + \
                                    np.square(vehicle_loc.z - ego_vehicle_loc.z))
                if vehicle_dist_sqr < in_radius_sqr:
                    return_vehicles.append(vehicle)

        for walker in walkers:
            walker_loc = walker.get_location()
            walker_dist_sqr =  (np.square(walker_loc.x - ego_vehicle_loc.x) + \
                                np.square(walker_loc.y - ego_vehicle_loc.y) + \
                                np.square(walker_loc.z - ego_vehicle_loc.z))
            if walker_dist_sqr < in_radius_sqr:
                return_walkers.append(walker)

        return return_vehicles, return_walkers

def spawn(num_vehicles=0,num_walkers=0,client=None,SPAWN_POINT=0,host='127.0.0.1',port=2000,tm_port= 8000,hybrid =True,sync = False,safe=True,filterv='vehicle.*',filterw="walker.pedestrian.*"):
    # argparser = argparse.ArgumentParser(
    #     description=__doc__)
    # argparser.add_argument(
    #     '--host',
    #     metavar='H',
    #     default='127.0.0.1',
    #     help='IP of the host server (default: 127.0.0.1)')
    # argparser.add_argument(
    #     '-p', '--port',
    #     metavar='P',
    #     default=2000,
    #     type=int,
    #     help='TCP port to listen to (default: 2000)')
    # argparser.add_argument(
    #     '-n', '--number-of-vehicles',
    #     metavar='N',
    #     default=30,
    #     type=int,
    #     help='number of vehicles (default: 10)')
    # argparser.add_argument(
    #     '-w', '--number-of-walkers',
    #     metavar='W',
    #     default=200,
    #     type=int,
    #     help='number of walkers (default: 200)')
    # argparser.add_argument(
    #     '--safe',
    #     action='store_true',
    #     help='avoid spawning vehicles prone to accidents')
    # argparser.add_argument(
    #     '--filterv',
    #     metavar='PATTERN',
    #     default='vehicle.*',
    #     help='vehicles filter (default: "vehicle.*")')
    # argparser.add_argument(
    #     '--filterw',
    #     metavar='PATTERN',
    #     default='walker.pedestrian.*',
    #     help='pedestrians filter (default: "walker.pedestrian.*")')
    # argparser.add_argument(
    #     '--tm-port',
    #     metavar='P',
    #     default=8000,
    #     type=int,
    #     help='port to communicate with TM (default: 8000)')
    # argparser.add_argument(
    #     '--sync',
    #     action='store_true',
    #     help='Synchronous mode execution')
    # argparser.add_argument(
    #     '--hybrid',
    #     action='store_true',
    #     help='Enanble')
    # args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    # client = carla.Client(host, port)
    client.set_timeout(10.0)
    synchronous_master = False

    # try:
    world = client.get_world()
    # blueprint_player = world.get_blueprint_library().filter("model3")[0]
    traffic_manager = client.get_trafficmanager(tm_port)
    
    if hybrid:
        traffic_manager.set_hybrid_physics_mode(True)

    if sync:
        settings = world.get_settings()
        traffic_manager.set_synchronous_mode(True)
        if not settings.synchronous_mode:
            synchronous_master = True
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)
        else:
            synchronous_master = False

    blueprints = world.get_blueprint_library().filter(filterv)
    blueprintsWalkers = world.get_blueprint_library().filter(filterw)

    if safe:
        blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
        blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
        blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
        blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
        blueprints = [x for x in blueprints if not x.id.endswith('t2')]

    spawn_points = world.get_map().get_spawn_points()
    # player_spawn = spawn_points[SPAWN_POINT]
    spawn_points.pop(SPAWN_POINT)
    number_of_spawn_points = len(spawn_points)

    if num_vehicles < number_of_spawn_points:
        random.shuffle(spawn_points)
    elif num_vehicles > number_of_spawn_points:
        msg = 'requested %d vehicles, but could only find %d spawn points'
        logging.warning(msg, num_vehicles, number_of_spawn_points)
        num_vehicles = number_of_spawn_points

    # @todo cannot import these directly.
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    # --------------
    # Spawn vehicles
    # --------------
    batch = []
    for n, transform in enumerate(spawn_points):


        if n >= num_vehicles:
            break
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)

        #if blueprint.has_attribute('speed'):
            #print("lol")
            #driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            #blueprint.set_attribute('driver_id', driver_id)

        blueprint.set_attribute('role_name', 'autopilot')
        
        batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))
    
    # batch.append(SpawnActor(blueprint_player, player_spawn))
    
    # traffic_manager.set_global_distance_to_leading_vehicle(10.0)
    # traffic_manager.global_percentage_speed_difference(5.0)
    
    
    for response in client.apply_batch_sync(batch, synchronous_master):
        if response.error:
            logging.error(response.error)
        else:
            vehicles_list.append(response.actor_id)
            x= response
    # player_actor = world.get_actor(vehicles_list[-1])

    for vehi in vehicles_list:
        # traffic_manager.collision_detection(player_actor,world.get_actor(vehi),True)
        traffic_manager.distance_to_leading_vehicle(world.get_actor(vehi),2)
    # print(vehicles_list)
    # -------------
    # Spawn Walkers
    # -------------
    # some settings
    percentagePedestriansRunning = 0.0      # how many pedestrians will run
    percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
    # 1. take all the random locations to spawn
    spawn_points = []
    for i in range(num_walkers):
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        if (loc != None):
            spawn_point.location = loc
            spawn_points.append(spawn_point)
    # 2. we spawn the walker object
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        # set as not invincible
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        # set the max speed
        if walker_bp.has_attribute('speed'):
            if (random.random() > percentagePedestriansRunning):
                # walking
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                # running
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        else:
            print("Walker has no speed")
            walker_speed.append(0.0)
        batch.append(SpawnActor(walker_bp, spawn_point))
    results = client.apply_batch_sync(batch, True)
    walker_speed2 = []
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})
            walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2
    # 3. we spawn the walker controller
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list[i]["con"] = results[i].actor_id
    # 4. we put altogether the walkers and controllers id to get the objects from their id
    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
    all_actors = world.get_actors(all_id)

    # wait for a tick to ensure client receives the last transform of the walkers we have just created
    if not sync or not synchronous_master:
        world.wait_for_tick()
    else:
        world.tick()

    # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
    # set how many pedestrians can cross the road
    world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
    for i in range(0, len(all_id), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        # max speed
        all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

    print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

    # example of how to use parameters
    traffic_manager.global_percentage_speed_difference(10.0)

    # return player_actor
        
        # while True:
        #     if sync and synchronous_master:
        #         world.tick()
        #     else:
        #         world.wait_for_tick()

    # finally:

    #     if sync and synchronous_master:
    #         settings = world.get_settings()
    #         settings.synchronous_mode = False
    #         settings.fixed_delta_seconds = None
    #         world.apply_settings(settings)

    #     print('\ndestroying %d vehicles' % len(vehicles_list))
    #     client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

    #     # stop walker controllers (list is [controller, actor, controller, actor ...])
    #     for i in range(0, len(all_id), 2):
    #         all_actors[i].stop()

    #     print('\ndestroying %d walkers' % len(walkers_list))
    #     client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

    #     time.sleep(0.5)
