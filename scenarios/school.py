import carla
import random
import numpy as np

def school(client,ego_state):

    NUMBER_OF_STUDENT_IN_RIGHT    = 6
    NUMBER_OF_STUDENT_IN_LEFT     = 6

    blueprint_library = client.get_world().get_blueprint_library()
    blueprintsWalkers = blueprint_library.filter("walker.pedestrian.*")
    #walker_bp = blueprint_library.filter("walker")[0]
    x_walk2=20
    y_walk2=47
    walkers=[]

    dist = np.linalg.norm([ego_state[0]-x_walk2,ego_state[1]-y_walk2])
    if (dist<70):

        for i in range(NUMBER_OF_STUDENT_IN_RIGHT):
            for j in range(i):
                walker_bp = random.choice(blueprintsWalkers)
                walker_transform=carla.Transform(carla.Location(x=35+(NUMBER_OF_STUDENT_IN_RIGHT-i),y=50+j, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
                walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

                if(walker!=None):
                    walkers.append(walker)
                    walker_control = carla.WalkerControl()
                    # walker_control.speed = 0.7+0.1*j
                    # walker_heading = -90+(i+j-3)*2*((-1)**i)
                    walker_control.speed = 0.4 + 0.025*j
                    # walker_heading = 0+(i+j-3)*2*((-1)**i)
                    walker_heading = -180+(i+j-3)*5*((-1)**i)
                    walker_rotation = carla.Rotation(0,walker_heading,0)
                    walker_control.direction = walker_rotation.get_forward_vector()
                    walker.apply_control(walker_control)

        for i in range(NUMBER_OF_STUDENT_IN_LEFT):
            for j in range(i):
                walker_bp = random.choice(blueprintsWalkers)
                walker_transform=carla.Transform(carla.Location(x=x_walk2+(NUMBER_OF_STUDENT_IN_LEFT-i), y=y_walk2-j, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
                walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

                if(walker!=None):
                    walkers.append(walker)
                    walker_control = carla.WalkerControl()
                    # walker_control.speed = 0.7+0.1*j
                    # walker_heading = -90+(i+j-3)*2*((-1)**i)
                    walker_control.speed = 0.3 + 0.015*j
                    # walker_heading = 0+(i+j-3)*2*((-1)**i)
                    walker_heading = 0+(i+j-2)*-9*((1)**i)
                    walker_rotation = carla.Rotation(0,walker_heading,0)
                    walker_control.direction = walker_rotation.get_forward_vector()
                    walker.apply_control(walker_control)
        spawned = True
        return np.array(walkers),spawned
    else:
        return None,False
