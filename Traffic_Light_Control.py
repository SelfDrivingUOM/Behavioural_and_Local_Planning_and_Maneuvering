import carla
try:
    import pygame
   

    from pygame.locals import K_r
    from pygame.locals import K_y
    from pygame.locals import K_g


except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')


class TrafficLightControl(object):
    """Class that handles keyboard input."""
    def __init__(self,traffic_lights):
        
        self._author = "Nigga G"
        self._junc_id = None
        self._traffic_lights = traffic_lights
        self._ego_light = None
        self.set_time = None

    def update_data(self,junc_id,ego_light,current_time):
        
        if(self._junc_id is not None):
            for traffic_light in self._traffic_lights[self._junc_id]:
                traffic_light.freeze(False)

        self._junc_id = junc_id
        self._ego_light = ego_light
        self.set_time = current_time

        
    def parse_events(self,simulation_time):

        if(self.set_time is not None):
            if(simulation_time-self.set_time >2):
                if(self._junc_id is not None):
                    for traffic_light in self._traffic_lights[self._junc_id]:
                        traffic_light.freeze(False)
                self._junc_id = None
                self._junc_id = None
                self._ego_light = None

        if(self._junc_id is not None ):

            for event in pygame.event.get():
                if event.type == pygame.KEYUP:
                    if event.key == K_g:

                        for traffic_light in self._traffic_lights[self._junc_id]:
                            if traffic_light == self._ego_light:
                                traffic_light.set_state(carla.TrafficLightState.Green)
                                traffic_light.freeze(True)
                            else:
                                traffic_light.set_state(carla.TrafficLightState.Red)
                                traffic_light.freeze(True)
                    elif event.key == K_y:
                    
                        for traffic_light in self._traffic_lights[self._junc_id]:
                            if traffic_light == self._ego_light:
                                traffic_light.set_state(carla.TrafficLightState.Yellow)
                                traffic_light.freeze(True)
                            else:
                                traffic_light.set_state(carla.TrafficLightState.Red)
                                traffic_light.freeze(True)

                    elif event.key == K_r:
                        
                        for traffic_light in self._traffic_lights[self._junc_id]:
                            if traffic_light == self._ego_light:
                                traffic_light.set_state(carla.TrafficLightState.Red)
                                traffic_light.freeze(True)
                            else:
                                traffic_light.set_state(carla.TrafficLightState.Red)
                                traffic_light.freeze(True)

            


    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

