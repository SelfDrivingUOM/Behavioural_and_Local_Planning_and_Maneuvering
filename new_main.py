from __future__ import print_function
from tools import misc

ITER_FOR_SIM_TIMESTEP  = 100     # no. iterations to compute approx sim timestep
WAIT_TIME_BEFORE_START = 0       # game seconds (time before controller start)
TOTAL_RUN_TIME         = 100.00  # game seconds (total runtime before sim end)
TOTAL_FRAME_BUFFER     = 300     # number of frames to buffer after total runtime
SIMULATION_TIME_STEP   = 0.034
# ==============================================================================
# --  Planning Constants -------------------------------------------------------aaaaaaaaaaaaaaaaa
# ==============================================================================

HOP_RESOLUTION = 1
DIST_THRESHOLD_TO_LAST_WAYPOINT = 2.0  # some distance from last position before
                                       # simulation ends

NUM_PATHS              = 11               # 
BP_LOOKAHEAD_BASE      = 10.0             # m
BP_LOOKAHEAD_TIME      = 1.0              # s
PATH_OFFSET            = 0.1              # m
NUMBER_OF_LAYERS       = 1
CIRCLE_OFFSETS         = [-1.0, 1.0, 3.0] # m
CIRCLE_RADII           = [1.8, 1.8, 1.8]  # m
TIME_GAP               = 1.0              # s
PATH_SELECT_WEIGHT     = 10               #
A_MAX                  = 5              # m/s^2
SLOW_SPEED             = 0              # m/s
STOP_LINE_BUFFER       = 1.5              # m
LEAD_VEHICLE_SPEED     = 1
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
INTERP_DISTANCE_RES       = 0.1 # distance between interpolated points

NO_VEHICLES = 300
NO_WALKERS = 0

import glob
import os
import sys
import time
import copy
# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('/home/selfdriving/carla-precompiled/CARLA_0.9.9/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg' )[0])
except IndexError:
    pass

# try:
#     sys.path.append('/home/selfdriving/yasintha/Path_planner_6/')
# except IndexError:
#     pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla



from global_route_planner import GlobalRoutePlanner
from global_route_planner_dao import GlobalRoutePlannerDAO
from carla import ColorConverter as cc
import controller2d
import local_planner
from basic_agent.basic_agent import BasicAgent
# import ogm_generator
from local_planner import get_closest_index
from environment import Environment
from Behavioural_planner import BehaviouralPlanner
from spawn import spawn

from tools.misc import get_speed
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
stopsign_fences = []

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def get_current_pose(transform):
    x = transform.location.x
    y = transform.location.y
    yaw = np.deg2rad(transform.rotation.yaw)

    return (x,y,yaw)

def send_control_command(vehicle, throttle, steer, brake, 
						 hand_brake=False, reverse=False,manual_gear_shift = False):

    
	# Clamp all values within their limits
    steer = np.fmax(np.fmin(steer*(30/np.pi), 1.0), -1.0)
    throttle = np.fmax(np.fmin(throttle, 1.0), 0)
    brake = np.fmax(np.fmin(brake, 1.0), 0)

    control = carla.VehicleControl(brake = brake , steer = float(steer),throttle = throttle)
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

def add_lane_change_waypoints(waypoints,lp,velocity,world):
    updated_waypoints = np.empty((0,3))
    updated_waypoints = np.append(updated_waypoints,waypoints[0].reshape((1,3)),axis=0)

    i=0
    while(True):
        if(i==(waypoints.shape[0]-1)):
            break

        if((np.linalg.norm(waypoints[i+1, :2]-waypoints[i,:2]) > 2) and (i > 6) and (i<waypoints.shape[0]-6)):
            
            start_index = i
            end_index = i+10

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

            updated_waypoints = np.append(updated_waypoints,path_wp.T,axis=0)

            i+=10

        else:
            updated_waypoints = np.append(updated_waypoints,waypoints[i+1].reshape((1,3)),axis=0)
            i+=1

    return updated_waypoints

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


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    spawn_point = 26  ##20/40-best
    try:
        
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        client.load_world("Town05")
        # client.generate_opendrive_world("/home/selfdriving/carla-precompiled/CARLA_0.9.9/CarlaUE4/Content/Carla/Maps/OpenDrive/Town03.xodr")

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args,spawn_point)
        # world = client.load_world('Town02')
        
        clock = pygame.time.Clock()

        world_map = world.world.get_map()
        world.world.debug.draw_line(carla.Location(x=-50 , y=0,z=0),carla.Location(x=200 , y=0,z=0), thickness=0.5, color=carla.Color(r=255, g=0, b=0), life_time=-1.)
        world.world.debug.draw_line(carla.Location(x=0 , y=-50,z=0),carla.Location(x=0 , y=200,z=0), thickness=0.5, color=carla.Color(r=255, g=0, b=0), life_time=-1.)
        

        start_point = world_map.get_spawn_points()[spawn_point]
        end_point = world_map.get_spawn_points()[0]
        # print(start_point)
        #environment = Environment(world.world,world_map,world.player)
        

        # blueprint_library = client.get_world().get_blueprint_library()
        # walker_bp = blueprint_library.filter("model3")[0]

        # walker_transform=carla.Transform(carla.Location(x=-175, y=88, z= 1.8314 ),carla.Rotation(yaw= 1.4203450679814286772))
        # walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

        spawn(NO_VEHICLES,NO_WALKERS)
        route = trace_route(start_point, end_point,HOP_RESOLUTION, world.player, world.world)
        waypoints = np.array(route)[:,0]
        waypoints_np = np.empty((0,3))
        vehicle_speed = 5

        # #spwaning a leading vehicle
        # x_lead=waypoints[10].transform.location.x
        # y_lead=waypoints[10].transform.location.y
        # z_lead=1.843102
        # #1.4203450679814286772

        # blueprint_library = client.get_world().get_blueprint_library()
        # my_car_bp = blueprint_library.filter("model3")[0]

        # lead_vehicle_tansform=carla.Transform(carla.Location(x=x_lead, y=y_lead, z=z_lead),carla.Rotation(yaw= waypoints[10].transform.rotation.yaw,pitch=waypoints[10].transform.rotation.pitch))
        # leading_vehicle=world.world.spawn_actor(my_car_bp, lead_vehicle_tansform)
        # actor_list.append(leading_vehicle)
        # Agent=BasicAgent(leading_vehicle)
        # Agent.set_path(route[10:])
        # start_x, start_y, start_yaw = get_current_pose(leading_vehicle.get_transform())


        environment = Environment(world.world,world.player,world_map)

        ################################################################
        ############        Initializing Local Planner     #############
        ################################################################

        wp_goal_index   = 0
        local_waypoints = None
        path_validity   = np.zeros((NUM_PATHS, 1), dtype=bool)

        LENGTH = world.player.bounding_box.extent.x*2
        WIDTH = world.player.bounding_box.extent.y*2

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

        waypoints_np = add_lane_change_waypoints(waypoints_np,lp,vehicle_speed, world.world)

        waypoints_np = remove_dup_wp(waypoints_np)

        # lp.waypoints_update(waypoints_np)
        # blueprint_library = client.get_world().get_blueprint_library()
        # walker_bp = blueprint_library.filter("walker")[0]

        # walker_transform=carla.Transform(carla.Location(x=8.5, y=-145.49017333984375, z= 0 ),carla.Rotation(yaw= 1.4203450679814286772))
        # walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)


        # if walker!=None:
        #     ##!!!###
        #     walker_control = carla.WalkerControl()
        #     walker_control.speed = 0.16

        #     walker_heading = -90
        #     walker_rotation = carla.Rotation(0,walker_heading,0)
        #     walker_control.direction = walker_rotation.get_forward_vector()
        #     walker.apply_control(walker_control)
        #     ##!!!###

        #     actor_list.append(walker)
        # x = -70 - 5 junction
        #3 junction x = -70, y =150

        # loc = carla.Location(x = -50, y = -90,z = 0 )
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



        for w in waypoints_np:
            world.world.debug.draw_string(carla.Location(x=w[0],y=w[1],z=0), 'O', draw_shadow=False,
                                   color=carla.Color(r=0, g=255, b=0), life_time=500,
                                   persistent_lines=True)

        ################################################################
        #############        Initializing Controller      ##############
        ################################################################
        
        controller = controller2d.Controller2D(waypoints_np)


        ################################################################
        #########        Initializing Behavioural Planner      #########
        ################################################################


        bp = BehaviouralPlanner(lp,waypoints_np,environment,world.world,HOP_RESOLUTION,world_map,world.player)





        sim_start_timestamp = world.world.get_snapshot().timestamp.elapsed_seconds

        # Initialize the current timestamp.
        current_timestamp = sim_start_timestamp

        #############################################
        # Frame-by-Frame Iteration and Initialization
        #############################################
        # Store pose history starting from the start position
        
        start_x, start_y, start_yaw = get_current_pose(world.player.get_transform())
        send_control_command(world.player, throttle=0.0, steer=0.0, brake=1.0)

        spectator = world.world.get_spectator()
        spectator.set_transform(world.player.get_transform())

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
        
        while True:
            # cmd=Agent.run_step(False)
            # send_control_command(leading_vehicle,cmd.throttle,cmd.steer,cmd.brake, hand_brake=False, reverse=False,manual_gear_shift = False)

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
            else:
                cmd_throttle = 0.0
                cmd_steer = 0.0
                cmd_brake = 0.0


            # Output controller command to CARLA server
            # print(cmd_throttle,cmd_steer,cmd_brake)
            send_control_command(world.player, throttle=cmd_throttle, steer= cmd_steer, brake=cmd_brake)
          
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
