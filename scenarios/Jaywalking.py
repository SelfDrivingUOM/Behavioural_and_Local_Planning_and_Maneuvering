import carla
import random

def jaywalking(client):

    blueprint_library = client.get_world().get_blueprint_library()
    blueprintsWalkers = blueprint_library.filter("walker.pedestrian.*")

    walker_bp = random.choice(blueprintsWalkers)
    walker_transform=carla.Transform(carla.Location(x=-158, y=95, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
    walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

    if(walker!=None):

        walker_control = carla.WalkerControl()
        walker_control.speed = 0.13 
        walker_heading = -45
        walker_rotation = carla.Rotation(0,walker_heading,0)
        walker_control.direction = walker_rotation.get_forward_vector()
        walker.apply_control(walker_control)