from __future__ import print_function
from tools import misc
from os_carla import WINDOWS
from os_carla import YASINTHA_WINDOWS
from scenarios.school import school
from scenarios.Jaywalking import jaywalking

ITER_FOR_SIM_TIMESTEP  = 100     # no. iterations to compute approx sim timestep
WAIT_TIME_BEFORE_START = 0       # game seconds (time before controller start)
TOTAL_RUN_TIME         = 100.00  # game seconds (total runtime before sim end)
TOTAL_FRAME_BUFFER     = 300     # number of frames to buffer after total runtime
SIMULATION_TIME_STEP   = 0.034
# ==============================================================================
# --  Planning Constants -------------------------------------------------------
# ==============================================================================

HOP_RESOLUTION = 1
DIST_THRESHOLD_TO_LAST_WAYPOINT = 2.0  # some distance from last position before
                                       # simulation ends
MAX_STEER_ANGLE        = 70               #DEGREES
NUM_PATHS              = 11               # 
BP_LOOKAHEAD_BASE      = 10.0             # m
BP_LOOKAHEAD_TIME      = 1.0              # s
PATH_OFFSET            = 0.1              # m
NUMBER_OF_LAYERS       = 1
CIRCLE_OFFSETS         = [-1.0, 1.0, 3.0] # m
CIRCLE_RADII           = [1.8, 1.8, 1.8]  # m
TIME_GAP               = 1.0              # s
PATH_SELECT_WEIGHT     = 10               #
A_MAX                  = 5                # m/s^2
SLOW_SPEED             = 0                # m/s
STOP_LINE_BUFFER       = 1.5              # m
LEAD_VEHICLE_SPEED     = 15                # m/s
LEAD_VEHICLE_LOOKAHEAD = 20.0             # m
LP_FREQUENCY_DIVISOR   = 1                # Frequency divisor tdo make the 
                                          # local planner operate at a lower
                                          # frequency than the controller
                                          # (which operates at the simulation
                                          # frequency). Must be a natural
                                          # number.

C4_STOP_SIGN_FILE        = 'stop_sign_params.txt'
C4_STOP_SIGN_FENCELENGTH =  5             # m
C4_PARKED_CAR_FILE       = 'parked_vehicle_params.txt'

# Path interpolation parameters
INTERP_MAX_POINTS_PLOT    = 10   # number of points used for displaying
                                 # selected path
INTERP_DISTANCE_RES       = 0.1  # distance between interpolated points

NO_AGENT_VEHICLES = 0
NO_VEHICLES =  0
NO_WALKERS  =  0
ONLY_HIGWAY =  0

NUMBER_OF_STUDENT_IN_ROWS    = 10
NUMBER_OF_STUDENT_IN_COLUMNS = 5

SPAWN_POINT = 35#189#26  #36 ##20/40-best
END_POINT   = 92 #0     #119
# global_path_points_set = [60,130,62,301,32,[32,28],[31,17],17]
# global_path_points_set = [126,[7,72],[5,71],150,[150,87],[158,88],88, 92 ]
global_path_points_set = [35,39,98,146,113,284,142,[278,114],[279,115],56,126,[7,72],[5,71],150,[150,87],[158,88],88, 92 ]
# global_path_points_set = [179, 258]
LEAD_SPAWN  = False
spawn_wpt_parked = 50

OVERTAKE_WALKERS = False
spawn_wpt_overtake_wlker = -20

NAVIGATION_SPAWN = False
WALKER_SPAWN =  False
DANGER_CAR   = False

Z           = 1.843102


import glob
import os
import sys
import time
# ==============================================================================
# -- Find CARLA module ----------------------------------------------------------
# ==============================================================================
if WINDOWS:

    try:
        sys.path.append(glob.glob('C:/Carla0.99/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])

    except IndexError:
        pass

    if (NAVIGATION_SPAWN):

        try:
            sys.path.append('C:/Carla0.99/PythonAPI/carla')
        except IndexError:
            pass

        try:
            sys.path.append('C:/Carla0.99/PythonAPI/carla/agents/navigation')

        except IndexError:
            pass
elif YASINTHA_WINDOWS:

    try:
        sys.path.append(glob.glob('C:/Users/4Axis/Desktop/Project/Carla/WindowsNoEditor/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])
    except IndexError:
        pass

    if (NAVIGATION_SPAWN):

        try:
            sys.path.append('C:/Users/4Axis/Desktop/Project/Carla/WindowsNoEditor/PythonAPI/carla/')

        except IndexError:
            pass

        try:
            sys.path.append('C:/Users/4Axis/Desktop/Project/Carla/WindowsNoEditor/PythonAPI/carla/agents/navigation')

        except IndexError:
            pass
else:
    try:
        sys.path.append(glob.glob('/home/selfdriving/carla-precompiled/CARLA_0.9.9/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg' )[0])
    except IndexError:
        pass

    if (NAVIGATION_SPAWN):

        try:
            sys.path.append('/home/selfdriving/carla-precompiled/CARLA_0.9.9/PythonAPI/carla/')

        except IndexError:
            pass

        try:
            sys.path.append('/home/selfdriving/carla-precompiled/CARLA_0.9.9/PythonAPI/carla/agents/navigation')

        except IndexError:
            pass



# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla



from global_route_planner import GlobalRoutePlanner
from global_route_planner_dao import GlobalRoutePlannerDAO
from carla import ColorConverter as cc
import controller2d
import local_planner
if (LEAD_SPAWN or DANGER_CAR):
    from basic_agent.basic_agent import BasicAgent
# import ogm_generator
from local_planner import get_closest_index
from environment import Environment
from Behavioural_planner import BehaviouralPlanner
# from spawn import spawn

# from spawn_new import spawn_new
from global_route_planner import GlobalRoutePlanner
from global_route_planner_dao import GlobalRoutePlannerDAO
if (NAVIGATION_SPAWN):
    from controller import VehiclePIDController
    from basic_agent import BasicAgent
    from carla import VehicleControl

from tools.misc import get_speed,draw_bound_box_actor
import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
from math import sin, cos, pi, sqrt
import hull

# from  import BasicAgent

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

actor_list=[]
agent_list=[]
stopsign_fences = []

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================
def get_transform(vehicle_location, angle, d=6.4):
    a = math.radians(angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
    return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))

def get_current_pose(transform):
    x = transform.location.x
    y = transform.location.y
    yaw = np.deg2rad(transform.rotation.yaw)

    if(yaw>np.pi):
        yaw-=2*np.pi
    elif(yaw<-np.pi):
        yaw+=2*np.pi
    return (x,y,yaw)

def send_control_command(vehicle, throttle, steer, brake, 
						 hand_brake=False, reverse=False,manual_gear_shift = False):

    
	# Clamp all values within their limits
    steer = np.fmax(np.fmin(steer/np.deg2rad(MAX_STEER_ANGLE), 1.0), -1.0)
    throttle = np.fmax(np.fmin(throttle, 1.0), 0)
    brake = np.fmax(np.fmin(brake, 1.0), 0)

    control = carla.VehicleControl(brake = brake , steer = float(steer),throttle = throttle,hand_brake=hand_brake)
    vehicle.apply_control(control)

def trace_route(start_waypoint, end_waypoint,sampling_resolution,vehicle,world):

    # Setting up global router
    dao = GlobalRoutePlannerDAO(vehicle.get_world().get_map(), sampling_resolution)
    grp = GlobalRoutePlanner(dao)
    grp.setup()

    # Obtain route plan
    route = grp.trace_route(start_waypoint.location,end_waypoint.location)

    return route

def debug_print(paths,world,best_index):
	for path in paths:
		le=len(path[0])
		if best_index==None:
			best_index=int(le/2)
		if best_index>=paths.shape[0]:
			best_index=paths.shape[0]-1
		for i in range(le):
            
			if np.all(path==paths[best_index]):
				x=path[0][i]
				y=path[1][i]
				t=path[2][i]

				loc=carla.Location(x=x , y=y,z=0)
				#print(loc)
				world.debug.draw_string(loc, 'X', draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=0.1,persistent_lines=True)
			else:
				x=path[0][i]
				y=path[1][i]
				t=path[2][i]

				loc=carla.Location(x=x , y=y,z=0)
				#print(loc)
				world.debug.draw_string(loc, 'X', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=0.1,persistent_lines=True)


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def remove_dup_wp(wp):
    waypoints = np.empty((0,3))
    waypoints = np.append(waypoints,wp[0].reshape((1,3)),axis=0)
    dist = 0
    for i in range(wp.shape[0]-1):
        # print(lane_change_points)
        dist += np.linalg.norm(wp[i+1,:2]-wp[i,:2])
        if (dist<0.5):

            continue
        else:
            waypoints = np.append(waypoints,wp[i+1].reshape((1,3)),axis=0)
            dist = 0
    return waypoints
    


def find_angle(vect1, vect2): #(x,y)
    vect1 = np.array([vect1[0],vect1[1],0]) # (x y 0)
    vect2 = np.array([vect2[0],vect2[1],0]) # (x y 0)
    u_vect1 = vect1/np.linalg.norm(vect1)
    u_vect2 = vect2/np.linalg.norm(vect2)
    v1_v2_dot = np.dot(u_vect1,u_vect2)

    if(vect2[1]>0):
        return -np.arccos(v1_v2_dot)
    else:
        return np.arccos(v1_v2_dot)

def add_lane_change_waypoints(waypoints,lp,velocity,world,map_):
    updated_waypoints = np.empty((0,3))
    updated_waypoints = np.append(updated_waypoints,waypoints[0].reshape((1,3)),axis=0)
    lane_changes =[]
    lanechange_laneid = []
    i=0
    count= 0
    while(True):
        if(i==(waypoints.shape[0]-1)):
            break
        wp_1 = map_.get_waypoint(carla.Location(x= waypoints[i+1,0],y = waypoints[i+1,1],z=0),project_to_road = True)
        wp_2 = map_.get_waypoint(carla.Location(x= waypoints[i,0],y = waypoints[i,1],z=0),project_to_road = True)

        if((np.linalg.norm(waypoints[i+1, :2]-waypoints[i,:2]) > 2) and (i > 6) and (i<waypoints.shape[0]-6) and wp_1.lane_id != wp_2.lane_id and not(wp_1.is_junction or wp_2.is_junction)):
            

            # world.debug.draw_string(carla.Location(x=waypoints[i,0],y=waypoints[i,1],z=0), 'H', draw_shadow=False,
            #         color=carla.Color(r=0, g=0, b=255), life_time=500,
            #         persistent_lines=True)
            lane_changes.append(waypoints[i,:2])
            lanechange_laneid.append(wp_1.lane_id)
            start_index = i
            end_index = i+10
            count+=1

            start_vector = (waypoints[start_index] - waypoints[start_index-1])
            start_vector[2] = 0

            theta = find_angle(np.array([1,0,0]),start_vector)

            wpe  = waypoints[end_index] - waypoints[start_index]
            wp_e = waypoints[end_index+1] - waypoints[start_index]

            wpe_x = wpe[0] * cos(theta) - wpe[1] * sin(theta)
            wpe_y = wpe[0] * sin(theta) + wpe[1] * cos(theta)
            wp_e_x = wp_e[0] * cos(theta) - wp_e[1] * sin(theta)
            wp_e_y = wp_e[0] * sin(theta) + wp_e[1] * cos(theta)

            delta_x = wp_e_x - wpe_x
            delta_y = wp_e_y - wpe_y

            heading = np.arctan2(delta_y,delta_x)

            goal = [wpe_x,wpe_y,heading]
            path = lp.plan_lane_change(goal)
            
            transform = waypoints[start_index]
            transform[2] = -theta
            path = np.array([path])
            paths = local_planner.transform_paths(path, transform)
            path = paths[0]
            
            path_wp = np.vstack((path[:2],np.full((1,path.shape[1]),velocity)))
            # print(path_wp.shape)
            updated_waypoints = np.append(updated_waypoints,path_wp.T,axis=0)

            i+=10

        else:
            updated_waypoints = np.append(updated_waypoints,waypoints[i+1].reshape((1,3)),axis=0)
            i+=1
    lane_changes = np.array(lane_changes)
    lanechange_laneid = np.array(lanechange_laneid)
    return updated_waypoints,lane_changes,lanechange_laneid

def genarate_global_path(globalPathPoints, world_map):
    # Setting up global router
    dao = GlobalRoutePlannerDAO(world_map, HOP_RESOLUTION)
    grp = GlobalRoutePlanner(dao)
    grp.setup()

    globalPath = []
    globalPathLocations = []
    spawnPoints = world_map.get_spawn_points()
    numberOfSpawnPts = len(spawnPoints)
    for i in range(len(globalPathPoints)):
        if type(globalPathPoints[i]) is list:
            if len(globalPathPoints[i])==2:
                if ((globalPathPoints[i][0]+1)>numberOfSpawnPts) or ((globalPathPoints[i][0]+1)<1) or ((globalPathPoints[i][1]+1)>numberOfSpawnPts) or ((globalPathPoints[i][1]+1)<1):
                    print("Invalid Global path point at globalPathPoints list index %d (value out of range):" %i,globalPathPoints[i])
                    raise Exception 
                else:
                    laneChangeLoc = carla.Location( x=(spawnPoints[globalPathPoints[i][0]].location.x+spawnPoints[globalPathPoints[i][1]].location.x)/2,
                                                    y=(spawnPoints[globalPathPoints[i][0]].location.y+spawnPoints[globalPathPoints[i][1]].location.y)/2,
                                                    z=(spawnPoints[globalPathPoints[i][0]].location.z+spawnPoints[globalPathPoints[i][1]].location.z)/2 )
                    
                    laneChangeLoc = (world_map.get_waypoint(laneChangeLoc,project_to_road=True,lane_type=carla.LaneType.Driving)).transform.location
                    globalPathLocations.append([laneChangeLoc])
            else:
                print("Global path points are not defined properly")
                raise Exception
        else:
            if ((globalPathPoints[i]+1)>numberOfSpawnPts) or ((globalPathPoints[i]+1)<1):
                print("Invalid Global path point at globalPathPoints list index %d (value out of range):" %i,globalPathPoints[i])
                raise Exception
            else:
                globalPathLocations.append(spawnPoints[globalPathPoints[i]].location)

    for n in range(len(globalPathLocations)-1):
        start = globalPathLocations[n]
        end = globalPathLocations[n+1]
        if (type(start) is list) and (type(end) is list):
            continue
        elif (type(start) is list):
            start = start[0]
        elif (type(end) is list):
            end = end[0]
        globalPath += grp.trace_route(start,end)

    # waypoints = np.array(globalPath)[:,0]

    # return waypoints
    return globalPath


### ----- These functions are used for the Model Predictive Controller ----- ###
def find_beta(vel,dir):
    current_velocity = np.zeros(3)
    forward_vector = np.zeros(3)
    current_velocity[:2] = vel[:]
    forward_vector[:2] = dir[:]
    current_velocity = current_velocity/(np.linalg.norm(current_velocity)+10**-10)
    forward_vector = forward_vector/np.linalg.norm(forward_vector)
    cross = np.cross(forward_vector,current_velocity)
    beta = np.arcsin(np.dot(cross,cross))
    if(np.dot([0,0,-1],cross)>0.):
        return beta
    else:
        return -beta


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================
class World(object):
    def __init__(self, carla_world, hud, args,spawn_point):
        self.world = carla_world
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._gamma = args.gamma
        self.start(spawn_point)
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0

    def start(self,spawn_point):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = self.world.get_blueprint_library().filter("model3")[0]
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])
        else:
            print("No recommended values for 'speed' attribute")
        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_point = self.map.get_spawn_points()[spawn_point]
            # spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            # self.player = spawned_player
        # Set up the sensors.
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        actors = [
            self.camera_manager.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()

# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================
class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        # vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name,
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]

        # if len(vehicles) > 1:
        #     self._info_text += ['Nearby vehicles:']
        #     distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
        #     vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
        #     for d, vehicle in sorted(vehicles):
        #         if d > 200.0:
        #             break
        #         vehicle_type = get_actor_display_name(vehicle, truncate=22)
        #         self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        
# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================
class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================
class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        Attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
            (carla.Transform(carla.Location(x=5.5, y=1.5, z=1.5)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-8.0, z=6.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-1, y=-bound_y, z=0.5)), Attachment.Rigid)]
        self.transform_index = 1
        self.sensors = [['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}]]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '50')
            item.append(bp)
        self.index = None


    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))
            
    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 3), 3))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype = int)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            array = array[:, ::-1, :]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)

def get_lane_change_ids(lane_changes, waypoints):
    index = []
    for i in range(lane_changes.shape[0]):
        dist_ = np.sum(np.square(waypoints[:,:2] - lane_changes[i]),axis = 1)
        idx = np.argmin(dist_)
        index.append(idx)
    # print(lane_changes.shape,waypoints.shape)
    index = np.array(index)
    return index

def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    try:
        
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        client.load_world("Town05")

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args,SPAWN_POINT)
        world_map = world.world.get_map()
        clock = pygame.time.Clock()
        world.world.debug.draw_line(carla.Location(x=0 , y=0,z=0),carla.Location(x=200 , y=0,z=0), thickness=0.5, color=carla.Color(r=255, g=0, b=0), life_time=-1.)
        world.world.debug.draw_line(carla.Location(x=0 , y=0,z=0),carla.Location(x=0 , y=200,z=0), thickness=0.5, color=carla.Color(r=0, g=255, b=0), life_time=-1.)

        spectator = world.world.get_spectator()
        specTrans = world.player.get_transform()
        specTrans.rotation.pitch = -90
        specTrans.rotation.yaw = 0
        specTrans.location.z = 20
        spectator.set_transform(specTrans)

    ###################################################################
    #######  Spawn Vehicles and Actors using CARLA Traffic Manager  ###
    ###################################################################
        if (NO_VEHICLES >= 0) and (NO_WALKERS >= 0):
            # Do common things related to spawning walkers and vehicles
            logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
            traffic_manager = client.get_trafficmanager(8000)
            traffic_manager.global_percentage_speed_difference(30.0)

            if NO_VEHICLES >= 0:
                vehicle_id_list = []
                vehicle_actor_list = []
                number_of_vehicles_spwan = NO_VEHICLES

                blueprints = world.world.get_blueprint_library().filter('vehicle.*')
                blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
                blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
                blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
                blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
                blueprints = [x for x in blueprints if not x.id.endswith('t2')]

                spawn_points = world_map.get_spawn_points()
                if ONLY_HIGWAY:
                    # for_highway = [264,265,266,267,268,269,198,254,255,256,261,262,263,194,195,191,192,193,194,144,165,162,257,258,259,270,272,273,179,48,49,50,163,235,234,159,160,161] # 189,169
                    for_highway = [259]#,189]
                    spwn_highway = []
                    for i in for_highway:
                        if i == SPAWN_POINT:
                            continue
                        spwn_highway.append(spawn_points[i])
                    spawn_points = spwn_highway
                else:
                    spawn_points.pop(SPAWN_POINT)
                number_of_spawn_points = len(spawn_points)
                
                if number_of_vehicles_spwan < number_of_spawn_points:
                    random.shuffle(spawn_points) # If Vehicle spawn is to be done in a randomly selected order
                    # pass
                elif number_of_vehicles_spwan > number_of_spawn_points:
                    msg = 'requested %d vehicles, but could only find %d spawn points'
                    logging.warning(msg, NO_VEHICLES, number_of_spawn_points)
                    number_of_vehicles_spwan = number_of_spawn_points

                batch = []
                for n, transform in enumerate(spawn_points):
                    if n >= number_of_vehicles_spwan:
                        break
                    blueprint = random.choice(blueprints)
                    if blueprint.has_attribute('color'):
                        color = random.choice(blueprint.get_attribute('color').recommended_values)
                        blueprint.set_attribute('color', color)
                    if blueprint.has_attribute('driver_id'):
                        driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                        blueprint.set_attribute('driver_id', driver_id)
                    blueprint.set_attribute('role_name', 'autopilot')
                    
                    batch.append(carla.command.SpawnActor(blueprint, transform).then(carla.command.SetAutopilot(carla.command.FutureActor, True, traffic_manager.get_port())))
                
                speeds_percen = [88.0]#,68.85]
                for n, response in enumerate(client.apply_batch_sync(batch, False)):
                    if response.error:
                        logging.error(response.error)
                    else:
                        vehicle_id_list.append(response.actor_id)
                        vehicle_actor_list.append(world.world.get_actor(response.actor_id))
                        traffic_manager.distance_to_leading_vehicle(vehicle_actor_list[-1],2)
                        # traffic_manager.auto_lane_change(vehicle_actor_list[-1], False)
                        # traffic_manager.vehicle_percentage_speed_difference(vehicle_actor_list[-1],speeds_percen[n])
                
            if NO_WALKERS >= 0:
                number_of_walkers_spwan = NO_WALKERS
                blueprintsWalkers = world.world.get_blueprint_library().filter('walker.pedestrian.*')
            
                percentagePedestriansRunning  = 0.0         # how many pedestrians will run
                percentagePedestriansCrossing = 0.0         # how many pedestrians will walk through the road
                # 1. take all the random locations to spawn
                spawn_points = []
                for i in range(number_of_walkers_spwan):
                    spawn_point = carla.Transform()
                    loc = world.world.get_random_location_from_navigation()
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
                    batch.append(carla.command.SpawnActor(walker_bp, spawn_point))
                results = client.apply_batch_sync(batch, False)
                walker_speed_temp = []
                walkers_list = []
                all_id = []
                for i in range(len(results)):
                    if results[i].error:
                        logging.error(results[i].error)
                    else:
                        walkers_list.append({"id": results[i].actor_id})
                        walker_speed_temp.append(walker_speed[i])
                walker_speed = walker_speed_temp
                # 3. we spawn the walker controller
                batch = []
                walker_controller_bp = world.world.get_blueprint_library().find('controller.ai.walker')
                for i in range(len(walkers_list)):
                    batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
                results = client.apply_batch_sync(batch, False)
                for i in range(len(results)):
                    if results[i].error:
                        logging.error(results[i].error)
                    else:
                        walkers_list[i]["con"] = results[i].actor_id
                # 4. we put altogether the walkers and controllers id to get the objects from their id
                for i in range(len(walkers_list)):
                    all_id.append(walkers_list[i]["con"])
                    all_id.append(walkers_list[i]["id"])
                all_actors = world.world.get_actors(all_id)

                world.world.wait_for_tick()

                world.world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
                for i in range(0, len(all_id), 2):
                    # start walker
                    all_actors[i].start()
                    # set walk to random point
                    all_actors[i].go_to_location(world.world.get_random_location_from_navigation())
                    # max speed
                    all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))
    ###################################################################

        # Spawn Vehicles
        # spawnedVehicleActorList, traffic_manager = spawn(NO_VEHICLES,NO_WALKERS,client,traffic_manager,world.world,SPAWN_POINT,ONLY_HIGWAY)
        

        
        # w=world_map.get_spawn_points()
        # for i in range (len(w)):
        #     p = world_map.get_spawn_points()[i]
        #     world.world.debug.draw_string(p.location, str(i), draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)


        start_point = world_map.get_spawn_points()[SPAWN_POINT]
        end_point = world_map.get_spawn_points()[END_POINT]
        # print(start_point)
        #environment = Environment(world.world,world_map,world.player)
        

        # blueprint_library = client.get_world().get_blueprint_library()
        # walker_bp = blueprint_library.filter("model3")[0]

        # walker_transform=carla.Transform(carla.Location(x=-175, y=88, z= 1.8314 ),carla.Rotation(yaw= 1.4203450679814286772))
        # walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

        
        # spawn_new(world.world,NO_VEHICLES)

        # route = trace_route(start_point, end_point,HOP_RESOLUTION, world.player, world.world)
        # waypoints = np.array(route)[:,0]
        # waypoints = genarate_global_path(global_path_points_set,world_map)
        route = genarate_global_path(global_path_points_set,world_map)
        waypoints = np.array(route)[:,0]
        waypoints_np = np.empty((0,3))
        vehicle_speed = 5

        if (NAVIGATION_SPAWN):
            spawn_pts=world_map.get_spawn_points()
            # print(spawn_pts)
            for i in range (len(spawn_pts)):
                p = world_map.get_spawn_points()[i]
                world.world.debug.draw_string(p.location, str(i), draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)
            
            spawn_pts.remove(start_point)
            for i in range (len(spawn_pts)):
                #print(i,len(spawn_pts)-1-i)
                if(i==SPAWN_POINT):
                    continue
                if (i >= NO_AGENT_VEHICLES):
                    break 
                else:
                    blueprint_library = world.world.get_blueprint_library()
                    vehicle_bp=blueprint_library.filter("model3")[0]
                    start_point=world_map.get_spawn_points()[i]
                    end_point = world_map.get_spawn_points()[len(spawn_pts)-1-i]
                    end = [end_point.location.x,end_point.location.y,end_point.location.z]
                    vehicle = world.world.spawn_actor(vehicle_bp, start_point)
                    actor_list.append(vehicle)
                    Agent=BasicAgent(vehicle,20)
                    Agent.set_destination(end)
                    agent_list.append(Agent)

                    #debug=world.debug

                    #all_actor_list = world.get_actors()
                    #vehicle_list = all_actor_list.filter("*vehicle*")
                    
            
            
        #################################################
        #############  Lead spawn  ####################
        #################################################
            
            
        if (LEAD_SPAWN):    
            #spwaning a leading vehicle
            x_lead=waypoints[spawn_wpt_parked].transform.location.x
            y_lead=waypoints[spawn_wpt_parked].transform.location.y 
            z_lead=1.843102
            #1.4203450679814286772

            blueprint_library = client.get_world().get_blueprint_library()
            my_car_bp = blueprint_library.filter("model3")[0]

            lead_vehicle_tansform=carla.Transform(carla.Location(x=x_lead, y=y_lead, z=z_lead),carla.Rotation(yaw= waypoints[spawn_wpt_parked].transform.rotation.yaw,pitch=waypoints[spawn_wpt_parked].transform.rotation.pitch))
            leading_vehicle=world.world.spawn_actor(my_car_bp, lead_vehicle_tansform)
            actor_list.append(leading_vehicle)
            Agent=BasicAgent(leading_vehicle,LEAD_VEHICLE_SPEED)
            # Agent.set_destination(world.world,world_map.get_spawn_points()[50])
            if LEAD_VEHICLE_SPEED>0:
                Agent.set_path(route[spawn_wpt_parked:])
            start_x, start_y, start_yaw = get_current_pose(leading_vehicle.get_transform())

        if OVERTAKE_WALKERS:
            x_overtake_walker=waypoints[spawn_wpt_overtake_wlker].transform.location.x + 1
            y_overtake_walker=waypoints[spawn_wpt_overtake_wlker].transform.location.y + 1
            z_overtake_walker=Z
            #1.4203450679814286772
            blueprintsWalkers = world.world.get_blueprint_library().filter("walker.pedestrian.*")[0]

            overtake_walker_tansform = carla.Transform(carla.Location(x=x_overtake_walker, y=y_overtake_walker, z=z_overtake_walker),carla.Rotation(yaw= waypoints[spawn_wpt_overtake_wlker].transform.rotation.yaw,pitch=waypoints[spawn_wpt_overtake_wlker].transform.rotation.pitch))
            overtake_walker = world.world.spawn_actor(blueprintsWalkers, overtake_walker_tansform)


        #################################################
        #############  Walker spawn  ####################
        #################################################
        if (WALKER_SPAWN):

            # school(client)
            jaywalking(client)

            # NUMBER_OF_STUDENT_IN_ROWS    = 10
            # NUMBER_OF_STUDENT_IN_COLUMNS = 5

            # blueprint_library = client.get_world().get_blueprint_library()
            # blueprintsWalkers = world.world.get_blueprint_library().filter("walker.pedestrian.*")
            # #walker_bp = blueprint_library.filter("walker")[0]

            

            # for i in range(NUMBER_OF_STUDENT_IN_ROWS):
            #     for j in range(i):
            #         walker_bp = random.choice(blueprintsWalkers)
            #         # walker_transform=carla.Transform(carla.Location(x=32-j, y=90+(NUMBER_OF_STUDENT_IN_ROWS-i), z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            #         # walker_transform=carla.Transform(carla.Location(x=40-j, y=0+(NUMBER_OF_STUDENT_IN_ROWS-i), z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            #         # walker_transform=carla.Transform(carla.Location(x=20-j, y=40+(NUMBER_OF_STUDENT_IN_ROWS-i), z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            #         # walker_transform=carla.Transform(carla.Location(x=-15-j, y=8+(NUMBER_OF_STUDENT_IN_ROWS-i), z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            #         #########SCHOOL NEW################
            #         walker_transform=carla.Transform(carla.Location(x=-150-j, y=90+(NUMBER_OF_STUDENT_IN_ROWS-i), z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))

            #         walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

            #         if(walker!=None):

            #             walker_control = carla.WalkerControl()
            #             # walker_control.speed = 0.7+0.1*j
            #             # walker_heading = -90+(i+j-3)*2*((-1)**i)
            #             walker_control.speed = 0.1
            #             # walker_heading = 0+(i+j-3)*2*((-1)**i)
            #             walker_heading = -90+(i+j-3)*2*((-1)**i)
            #             walker_rotation = carla.Rotation(0,walker_heading,0)
            #             walker_control.direction = walker_rotation.get_forward_vector()
            #             walker.apply_control(walker_control)

        '''if (WALKER_SPAWN):
            NUMBER_OF_STUDENT_IN_ROWS    = 10
            NUMBER_OF_STUDENT_IN_COLUMNS = 6

            blueprint_library = client.get_world().get_blueprint_library()
            blueprintsWalkers = world.world.get_blueprint_library().filter("walker.pedestrian.*")
            #walker_bp = blueprint_library.filter("walker")[0]

            

            for i in range(NUMBER_OF_STUDENT_IN_ROWS):
                for j in range(i):
                    walker_bp = random.choice(blueprintsWalkers)
                    # walker_transform=carla.Transform(carla.Location(x=32-j, y=90+(NUMBER_OF_STUDENT_IN_ROWS-i), z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
                    walker_transform=carla.Transform(carla.Location(x=-120-j, y=140+(NUMBER_OF_STUDENT_IN_ROWS-i), z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
                    walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

                    if(walker!=None):

                        walker_control = carla.WalkerControl()
                        # walker_control.speed = 0.7+0.1*j
                        # walker_heading = -90+(i+j-3)*2*((-1)**i)
                        walker_control.speed = 0.21
                        walker_heading = 180+(i+j-3)*2*((-1)**i)
                        walker_rotation = carla.Rotation(0,walker_heading,0)
                        walker_control.direction = walker_rotation.get_forward_vector()
                        walker.apply_control(walker_control)'''
        
        if (DANGER_CAR):
            #spwaning a leading vehicle
            spawn_pts=world_map.get_spawn_points()
            
            strt = 69
            x_lead = spawn_pts[strt].location.x
            y_lead = spawn_pts[strt].location.y
            z_lead = 1.843102

            danger_route = genarate_global_path([strt,95],world_map)

            blueprint_library = client.get_world().get_blueprint_library()
            danger_car_bp = blueprint_library.filter("model3")[0]

            danger_vehicle_tansform=carla.Transform(carla.Location(x=x_lead, y=y_lead, z=z_lead),carla.Rotation(yaw= 180,pitch=0))
            danger_vehicle=world.world.spawn_actor(danger_car_bp, danger_vehicle_tansform)
            actor_list.append(danger_vehicle)
            danger_car_agent=BasicAgent(danger_vehicle,100)
            danger_car_agent.set_path(danger_route)


        ################################################################
        ############        Initializing Local Planner     #############
        ################################################################

        wp_goal_index   = 0
        local_waypoints = None
        path_validity   = np.zeros((NUM_PATHS, 1), dtype=bool)

        LENGTH = (world.player.bounding_box.extent.x*2)
        WIDTH = (world.player.bounding_box.extent.y*2)

        ################################################################
		###  Obtaining Global Route with hop of given resolution     ###
		################################################################

        lp = local_planner.LocalPlanner(NUM_PATHS,
                        PATH_OFFSET,
                        LENGTH,
                        WIDTH,
                        PATH_SELECT_WEIGHT,
                        TIME_GAP,
                        A_MAX,
                        SLOW_SPEED,
                        STOP_LINE_BUFFER,
                        NUMBER_OF_LAYERS)


        # route = trace_route(start_point, end_point,HOP_RESOLUTION, world.player, world.world)
        # waypoints = np.array(route)[:,0]
        # waypoints_np = np.empty((0,3))
        # vehicle_speed = 5

        for i in range(waypoints.shape[0]):
            waypoints_np = np.append(waypoints_np, np.array([[waypoints[i].transform.location.x, waypoints[i].transform.location.y, vehicle_speed]]),axis=0)

        waypoints_np = remove_dup_wp(waypoints_np)

        waypoints_np,lane_changes,lane_change_lane_ids = add_lane_change_waypoints(waypoints_np,lp,vehicle_speed, world.world,world_map)

        waypoints_np = remove_dup_wp(waypoints_np)

        lane_change_idx = get_lane_change_ids(lane_changes,waypoints_np)

        for i in range (waypoints_np.shape[0]):

            if(i in lane_change_idx):
                world.world.debug.draw_point(carla.Location(x=waypoints_np[i,0],y=waypoints_np[i,1],z=0.2), size=0.04,
                    color=carla.Color(r=255, g=0, b=0), life_time=500,
                    persistent_lines=True)
            else:
                world.world.debug.draw_point(carla.Location(x=waypoints_np[i,0],y=waypoints_np[i,1],z=0.2),size=0.04,
                                    color=carla.Color(r=0, g=255, b=0), life_time=500,
                                    persistent_lines=True)

        # print(lane_changes , lane_changes.shape)

        #################################################
        #############  Walker spawn  ####################
        #################################################

        # lp.waypoints_update(waypoints_np)
        
     
   
        #     actor_list.append(walker)
        # x = -70 - 5 junction
        #3 junction x = -70, y =150

        ############
        ### School = (x = 20, y = 45, z = 0)
        ############

        # loc = carla.Location(x = -120, y = 140, z = 0)
        # world.world.debug.draw_string(loc, 'X', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)
       
        # raise Exception
        # get_line(np.array([(1,1),(2,2),(2,3),(5,3)]))

        # raise Exception

        # waypoint = world_map.get_waypoint(loc,project_to_road = True,lane_type = (carla.LaneType.Driving|carla.LaneType.Sidewalk|carla.LaneType.Parking|carla.LaneType.Parking))
        
        # junc_points = misc.print_junction(world,waypoint)

        # loc = carla.Location(x = 22, y = 140,z = 0 )
        # world.world.debug.draw_string(loc, 'X', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)
        # waypoint = world_map.get_waypoint(loc,project_to_road = True,lane_type = (carla.LaneType.Driving|carla.LaneType.Sidewalk|carla.LaneType.Parking|carla.LaneType.Parking))

        # loc = carla.Location(x = 27, y = 190,z = 0 )
        # world.world.debug.draw_string(loc, 'X', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)

        # waypoint = world_map.get_waypoint(loc,project_to_road = True,lane_type = (carla.LaneType.Driving|carla.LaneType.Sidewalk|carla.LaneType.Parking|carla.LaneType.Parking))
        # junc_points = misc.print_junction(world,waypoint)



        # loc = carla.Location(x = 155, y = 0,z = 0 )
        # world.world.debug.draw_string(loc, 'X', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)

        # waypoint = world_map.get_waypoint(loc,project_to_road = True,lane_type = (carla.LaneType.Driving|carla.LaneType.Sidewalk|carla.LaneType.Parking|carla.LaneType.Parking))
        
        # junc_points = misc.print_junction(world,waypoint)

        # loc = carla.Location(x = -270, y = 0,z = 0 )
        # world.world.debug.draw_string(loc, 'X', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)

        # waypoint = world_map.get_waypoint(loc,project_to_road = True,lane_type = (carla.LaneType.Driving|carla.LaneType.Sidewalk|carla.LaneType.Parking|carla.LaneType.Parking))
        
        # junc_points = misc.print_junction(world,waypoint)

        # loc = carla.Location(x = -270, y = 6,z = 0 )
        # world.world.debug.draw_string(loc, 'X', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)

        # waypoint = world_map.get_waypoint(loc,project_to_road = True,lane_type = (carla.LaneType.Driving|carla.LaneType.Sidewalk|carla.LaneType.Parking|carla.LaneType.Parking))
        
        # print(junc_points)

        # lines = misc.get_line(junc_points)
        # misc.draw_hex(world,lines)
        # inter_junc_points = misc.solve_lines(lines)

        # box_points = misc.get_box(world_map,inter_junc_points)

        # for i in range(box_points.shape[0]):
        #     print(inter_junc_points[i])
        #     world.world.debug.draw_string(carla.Location(x=box_points[i,0],y = box_points[i,1],z = 1),"A", draw_shadow=False,color=carla.Color(r=255, g=255, b=0), life_time=10000,persistent_lines=True)
        

        # raise Exception
                    # print(waypoint.lane_type)
            


        # # print()


    
        

        # raise Exception
        # print(waypoint.get_junction())
        # L = waypoint.get_junction().get_waypoints(carla.LaneType.Driving)
        # L = L[:len(L)//2]

        # import random

        # print(len(L))
        # for i in L:

        #     # rand_r = random.randint(0,255)
        #     # rand_g = random.randint(0,255)
        #     # rand_b = random.randint(0,255)
        #     for j in i:
        #         # print(j.lane_id,j.road_id,j.section_id,j.lane_type)
        #         world.world.debug.draw_string(j.transform.location,"A", draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=10000,persistent_lines=True)
            # time.sleep(5)
            # print("")

        # raise Exception
        # # print(waypoint.lane_type,waypoint.lane_id,waypoint.section_id,waypoint.is_junction)

        # world.world.debug.draw_string(waypoint.transform.location, 'X', draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=100,persistent_lines=True)
        # print(waypoint.lane_id,waypoint.lane_type)

        # raise Exception
        # world.world.debug.draw_string(waypoint.transform.location, 'X', draw_shadow=False,color=carla.Color(r=0, g=255, b=0), life_time=10000,persistent_lines=True)
        # print(waypoint.lane_id,waypoint.lane_type)

        # walker_transform=carla.Transform(carla.Location(x= -6.5, y=-125.1, z= 1.152402 ),carla.Rotation(yaw= 1.4203450679814286772))
        
        # # walker_transform=carla.Transform(carla.Location(x= -12.5, y=-125.1, z= 1.152402 ),carla.Rotation(yaw= 1.4203450679814286772))
        
        # # debug_print()
        # walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

        # walker.go_to_location(carla.Location(x = 1951,y = -12672 ,z= 14.6974))
        # walker.set_max_speed(0.2)



        
        # raise Exception

        time.sleep(20)
    
        # sleep_time_start= time.time()
        # while(time.time()-sleep_time_start<30):
        #     world.world.wait_for_tick()
        
        # time.sleep(40)

        environment = Environment(world.world,world.player,world_map)
        ################################################################
        #############        Initializing Controller      ##############
        ################################################################
        
        controller = controller2d.Controller2D(waypoints_np, world.world)


        ################################################################
        #########        Initializing Behavioural Planner      #########
        ################################################################


        bp = BehaviouralPlanner(world.world, world_map, world.player, environment, lp, waypoints_np, HOP_RESOLUTION ,lane_change_idx,lane_change_lane_ids)

        sim_start_timestamp = world.world.get_snapshot().timestamp.elapsed_seconds

        # Initialize the current timestamp.
        current_timestamp = sim_start_timestamp

        #############################################
        # Frame-by-Frame Iteration and Initialization
        #############################################
        # Store pose history starting from the start position
        
        start_x, start_y, start_yaw = get_current_pose(world.player.get_transform())
        send_control_command(world.player, throttle=0.0, steer=0.0, brake=1.0)

        

        #############################################
        # Scenario Execution Loop
        #############################################

        reached_the_end = False
        skip_first_frame = True

        # Initialize history
        frame=0
        
        ## define car ellipse points wrt to cars frame of referance
        # car_ellipse = np.array([[ LENGTH/(2**0.5),0],
        #                        [-LENGTH/(2**0.5),0],
        #                        [ 0,WIDTH/(2**0.5)],
        #                        [ 0,WIDTH/(2**0.5)],
        #                        [ LENGTH/(2*(2**0.5)), WIDTH*(7**0.5)/(4)],
        #                        [-LENGTH/(2*(2**0.5)),-WIDTH*(7**0.5)/(4)],
        #                        [-LENGTH/(2*(2**0.5)), WIDTH*(7**0.5)/(4)],
        #                        [ LENGTH/(2*(2**0.5)),-WIDTH*(7**0.5)/(4)]])
        count=0
        while True:
            # if (count%50==0):
            
            #     specTrans = world.player.get_transform()
            #     specTrans.rotation.pitch = -90
            #     specTrans.rotation.yaw = 0
            #     specTrans.location.z = 20
            #     spectator.set_transform(specTrans)
            # count+=1
            # print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
            # for veh in vehicle_actor_list:
            #     print(veh,get_speed(veh))
            
            if (LEAD_SPAWN):

                cmd=Agent.run_step(False)
                send_control_command(leading_vehicle,cmd.throttle,cmd.steer,cmd.brake, hand_brake=False, reverse=False,manual_gear_shift = False)

            if (NAVIGATION_SPAWN):
                for j in range (len(actor_list)):
                    cmd=agent_list[j].run_step(False)
                    send_control_command(actor_list[j],cmd.throttle,cmd.steer,cmd.brake, hand_brake=False, reverse=False,manual_gear_shift = False)
            
            if (DANGER_CAR):
                cmd=danger_car_agent.danger_step(False)
                send_control_command(danger_vehicle,0.20,cmd.steer,cmd.brake, hand_brake=False, reverse=False,manual_gear_shift = False)
            # lead_waypoint = world_map.get_waypoint(leading_vehicle.get_transform().location,project_to_road=True)
            # lead_lane = lead_waypoint.lane_id   
            # print("lead",lead_lane)   

            # ego_waypoint = world_map.get_waypoint(world.player.get_transform().location,project_to_road=True)
            # ego_lane = ego_waypoint.lane_id   
            # print("ego",ego_lane)   

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return 
                else:
                    pass

            # clock.tick_busy_loop(60)

            tic = time.time()

            start_snapshot=world.world.get_snapshot()
            sim_start_timestamp = start_snapshot.timestamp.elapsed_seconds
            
            # spectator.set_transform(camera.get_transform())

            
            current_x, current_y, current_yaw = get_current_pose(world.player.get_transform())
            current_speed = get_speed(world.player)

            # ####!!!!
            
            # posss = world.player.get_transform().location
            # da_width = world_map.get_waypoint(posss)#.right_lane_marking.type
            # # print("XXX",da_width)
            # poss1 = da_width.get_right_lane()
            # print(da_width.lane_id,"0000000")
            # # if poss1 != None :   
            # #     print(poss1.lane_id,"111111111111111")
            # #     poss2 = poss1.get_right_lane()
            # #     if poss2 != None : 
            # #         print(poss2.lane_id,"22222222222222")
            
            # ####!!!!

            # update timestamps
            prev_timestamp = current_timestamp
            current_timestamp = world.world.get_snapshot().timestamp.elapsed_seconds


            if current_timestamp <= WAIT_TIME_BEFORE_START:   
                send_control_command(world.player, throttle=0.0, steer=0.0, brake=1.0)
                continue
            else:
                current_timestamp = current_timestamp - WAIT_TIME_BEFORE_START
            

            if frame % LP_FREQUENCY_DIVISOR == 0:
                
                ego_state = [current_x, current_y, current_yaw]

                local_waypoints = bp.state_machine(ego_state,current_timestamp,prev_timestamp,current_speed)
                # print(len(local_waypoints),len(local_waypoints[0]))




                # --------------------------------------------------------------
                if local_waypoints != None:

                    # Update the controller waypoint path with the best local path.
                    # This controller is similar to that developed in Course 1 of this
                    # specialization.  Linear interpolation computation on the waypoints
                    # is also used to ensure a fine resolution between points.
                    wp_distance = []   # distance array
                    local_waypoints_np = np.array(local_waypoints)
                    for i in range(1, local_waypoints_np.shape[0]):
                        wp_distance.append(
                                np.sqrt((local_waypoints_np[i, 0] - local_waypoints_np[i-1, 0])**2 +
                                        (local_waypoints_np[i, 1] - local_waypoints_np[i-1, 1])**2))
                    wp_distance.append(0)  # last distance is 0 because it is the distance
                                           # from the last waypoint to the last waypoint

                    # Linearly interpolate between waypoints and store in a list
                    wp_interp = np.empty((0,3))    # interpolated values 
                                           # (rows = waypoints, columns = [x, y, v])
                    for i in range(local_waypoints_np.shape[0] - 1):
                        # Add original waypoint to interpolated waypoints list (and append
                        # it to the hash table)
                        wp_interp = np.append(wp_interp,local_waypoints_np[i].reshape((1,3)),axis=0)
                        
                
                        # Interpolate to the next waypoint. First compute the number of
                        # points to interpolate based on the desired resolution and
                        # incrementally add interpolated points until the next waypoint
                        # is about to be reached.
                        num_pts_to_interp = int(np.floor(wp_distance[i] / float(INTERP_DISTANCE_RES)) - 1)
                        wp_vector = local_waypoints_np[i+1] - local_waypoints_np[i]
                        wp_uvector = wp_vector / np.linalg.norm(wp_vector[0:2])

                        for j in range(num_pts_to_interp):
                            next_wp_vector = INTERP_DISTANCE_RES * float(j+1) * wp_uvector
                            wp_interp = np.append(wp_interp,(local_waypoints_np[i] + next_wp_vector).reshape((1,3)),axis=0)
                    # add last waypoint at the end
                    wp_interp = np.append(wp_interp,local_waypoints_np[-1].reshape((1,3)),axis=0)
                    # Update the other controller values and controls
                    controller.update_waypoints(wp_interp)
                    pass
                else:
                    print("Local waypoints NONE returned there is an error in behaviour planner")
                
                # tt5 = time.time()
                
            
            ###
            # Controller Update
            ###

            # x, y, yaw, speed, timestamp, frame,velocity,beta,d_shi#########################################################
            if local_waypoints != None and local_waypoints != []:
                # controller.update_values(current_x, current_y, current_yaw, 
                #                          current_speed,
                #                          current_timestamp, frame)

                ###!!!###
                current_velocity = np.array([world.player.get_velocity().x,world.player.get_velocity().y])
                velocity_mag = np.linalg.norm(current_velocity)
                forward_vector = np.array([world.player.get_transform().get_forward_vector().x,world.player.get_transform().get_forward_vector().y])
                beta = find_beta(current_velocity,forward_vector)
                diff_shi = np.deg2rad(world.player.get_angular_velocity().z)

                controller.update_values(current_x, current_y, current_yaw, 
                                    current_speed,
                                    current_timestamp, frame, velocity_mag, beta, diff_shi)
                ###!!!###

                controller.update_controls()
                cmd_throttle, cmd_steer, cmd_brake = controller.get_commands()
                # Output controller command to CARLA server
                # print(cmd_throttle,cmd_steer,cmd_brake)
                send_control_command(world.player, throttle=cmd_throttle, steer= cmd_steer, brake=cmd_brake)

            else:
                cmd_throttle = 0.0
                cmd_steer = 0.0
                cmd_brake = 0.0
                # Output controller command to CARLA server
                # print(cmd_throttle,cmd_steer,cmd_brake)
                send_control_command(world.player, throttle=cmd_throttle, steer= cmd_steer, brake=cmd_brake,hand_brake=True)
            


            
            # Find if reached the end of waypoint. If the car is within
            # DIST_THRESHOLD_TO_LAST_WAYPOINT to the last waypoint,
            # the simulation will end.
            dist_to_last_waypoint = np.linalg.norm(np.array([
                waypoints_np[-1][0] - current_x,
                waypoints_np[-1][1] - current_y]))
            if  dist_to_last_waypoint < DIST_THRESHOLD_TO_LAST_WAYPOINT:
                reached_the_end = True
            if reached_the_end:
                break
            # spectator.set_transform(camera.get_transform())

            frame+=1

            toc = time.time()
            # print(toc-tic)
            

            world.tick(clock)
            world.render(display)
            pygame.display.flip()
            # print(1/(toc - tic))
            if(toc-tic>SIMULATION_TIME_STEP):
                continue
            else:

                time.sleep(SIMULATION_TIME_STEP - (toc-tic))

    finally:
        if NO_VEHICLES>0:
            # Destroy Spawned Vehicels
            print('\ndestroying %d vehicles' % len(vehicle_id_list))
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_id_list])

        if NO_WALKERS>0:
            # Destroy Spawned Walkers
            for i in range(0, len(all_id), 2):
                all_actors[i].stop()
            print('\ndestroying %d walkers' % len(walkers_list))
            client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()

        # if sync and synchronous_master:
        #     settings = world.get_settings()
        #     settings.synchronous_mode = False
        #     settings.fixed_delta_seconds = None
        #     world.apply_settings(settings)

        # print('\ndestroying %d vehicles' % len(vehicles_list))
        # client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # # stop walker controllers (list is [controller, actor, controller, actor ...])
        # for i in range(0, len(all_id), 2):
        #     all_actors[i].stop()
        # print('\ndestroying %d walkers' % len(walkers_list))
        # client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

        # time.sleep(0.5)

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        # default='400x300',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
