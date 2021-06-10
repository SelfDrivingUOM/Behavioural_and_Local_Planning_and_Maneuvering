import carla
import random
import numpy as np
import time

def school(client,ego_state):

    NUMBER_OF_STUDENT_IN_RIGHT    = 7
    NUMBER_OF_STUDENT_IN_LEFT     = 6

    blueprint_library = client.get_world().get_blueprint_library()
    blueprintsWalkers = blueprint_library.filter("walker.pedestrian.*")
    #walker_bp = blueprint_library.filter("walker")[0]
    x_walk1= -90+9
    y_walk1= -97
    x_walk2=-94+7
    y_walk2=-89
    x_walk3=-62
    y_walk3=-94

    x_walk4=-72
    y_walk4=-86

    x_walk5=-82
    y_walk5=-82

    x_walk6=-92
    y_walk6=-82

    walkers=[]

    dist = np.linalg.norm([ego_state[0]-x_walk2,ego_state[1]-y_walk2])
    if (dist<50):

        for i in range(NUMBER_OF_STUDENT_IN_RIGHT):
            for j in range(i):
                walker_bp = random.choice(blueprintsWalkers)
                # walker_transform=carla.Transform(carla.Location(x=35+(NUMBER_OF_STUDENT_IN_RIGHT-i),y=50+j, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772)) #near scl
                walker_transform=carla.Transform(carla.Location(x=x_walk1-j,y=y_walk1+(NUMBER_OF_STUDENT_IN_RIGHT-i), z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))

                walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

                if(walker!=None):
                    walkers.append(walker)
                    walker_control = carla.WalkerControl()
                    # walker_control.speed = 0.7+0.1*j
                    # walker_heading = -90+(i+j-3)*2*((-1)**i)
                    walker_control.speed = 0.25 + 0.015*j
                    # walker_heading = 0+(i+j-3)*2*((-1)**i)
                    # walker_heading = -180+(i+j-3)*5*((-1)**i) # near scl
                    walker_heading = -270-(i+j-3)*5*((-1)**i) 
                    walker_rotation = carla.Rotation(0,walker_heading,0)
                    walker_control.direction = walker_rotation.get_forward_vector()
                    walker.apply_control(walker_control)
                # time.sleep(1)

        for i in range(NUMBER_OF_STUDENT_IN_LEFT):
            for j in range(NUMBER_OF_STUDENT_IN_LEFT-i):
                walker_bp = random.choice(blueprintsWalkers)
                # walker_transform=carla.Transform(carla.Location(x=x_walk2+(NUMBER_OF_STUDENT_IN_LEFT-i), y=y_walk2-j, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
                walker_transform=carla.Transform(carla.Location(x=x_walk2-j, y=y_walk2+(NUMBER_OF_STUDENT_IN_LEFT-i), z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))

                walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

                if(walker!=None):
                    walkers.append(walker)
                    walker_control = carla.WalkerControl()
                    # walker_control.speed = 0.7+0.1*j
                    # walker_heading = -90+(i+j-3)*2*((-1)**i)
                    walker_control.speed = 0.3 + 0.015*i
                    #walker_control.speed = 0.205+0.005*(j-3)**2
                    # walker_heading = 0+(i+j-3)*2*((-1)**i) #near scl
                    walker_heading = -90-(i+j-3)*5*((-1)**i)  #+(i+j-2)*-5*((1)**i)
                    walker_rotation = carla.Rotation(0,walker_heading,0)
                    walker_control.direction = walker_rotation.get_forward_vector()
                    walker.apply_control(walker_control)
        
        for i in range(1):
            walker_bp = random.choice(blueprintsWalkers)
            # walker_transform=carla.Transform(carla.Location(x=x_walk2+(NUMBER_OF_STUDENT_IN_LEFT-i), y=y_walk2-j, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker_transform=carla.Transform(carla.Location(x=x_walk3, y=y_walk3-2*i, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

            if(walker!=None):
                walkers.append(walker)
                walker_control = carla.WalkerControl()
                # walker_control.speed = 0.7+0.1*j
                # walker_heading = -90+(i+j-3)*2*((-1)**i)
                walker_control.speed = 1
                #walker_control.speed = 0.205+0.005*(j-3)**2
                # walker_heading = 0+(i+j-3)*2*((-1)**i) #near scl
                walker_heading = 90
                walker_rotation = carla.Rotation(0,walker_heading,0)
                walker_control.direction = walker_rotation.get_forward_vector()
                walker.apply_control(walker_control)
            # time.sleep(1)

        for i in range(3):
            walker_bp = random.choice(blueprintsWalkers)
            # walker_transform=carla.Transform(carla.Location(x=x_walk2+(NUMBER_OF_STUDENT_IN_LEFT-i), y=y_walk2-j, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker_transform=carla.Transform(carla.Location(x=x_walk4-2*i, y=y_walk4, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

            if(walker!=None):
                walkers.append(walker)
                walker_control = carla.WalkerControl()
                # walker_control.speed = 0.7+0.1*j
                # walker_heading = -90+(i+j-3)*2*((-1)**i)
                walker_control.speed = 0.7-i*0.1
                #walker_control.speed = 0.205+0.005*(j-3)**2
                # walker_heading = 0+(i+j-3)*2*((-1)**i) #near scl
                walker_heading = -90
                walker_rotation = carla.Rotation(0,walker_heading,0)
                walker_control.direction = walker_rotation.get_forward_vector()
                walker.apply_control(walker_control)

        for i in range(4):
            walker_bp = random.choice(blueprintsWalkers)
            # walker_transform=carla.Transform(carla.Location(x=x_walk2+(NUMBER_OF_STUDENT_IN_LEFT-i), y=y_walk2-j, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker_transform=carla.Transform(carla.Location(x=x_walk5-i, y=y_walk5-i, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

            if(walker!=None):
                walkers.append(walker)
                walker_control = carla.WalkerControl()
                # walker_control.speed = 0.7+0.1*j
                # walker_heading = -90+(i+j-3)*2*((-1)**i)
                walker_control.speed = 0.5
                #walker_control.speed = 0.205+0.005*(j-3)**2
                # walker_heading = 0+(i+j-3)*2*((-1)**i) #near scl
                walker_heading = -135
                walker_rotation = carla.Rotation(0,walker_heading,0)
                walker_control.direction = walker_rotation.get_forward_vector()
                walker.apply_control(walker_control)

        for i in range(4):
            walker_bp = random.choice(blueprintsWalkers)
            # walker_transform=carla.Transform(carla.Location(x=x_walk2+(NUMBER_OF_STUDENT_IN_LEFT-i), y=y_walk2-j, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker_transform=carla.Transform(carla.Location(x=x_walk6-i, y=y_walk6-i, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

            if(walker!=None):
                walkers.append(walker)
                walker_control = carla.WalkerControl()
                # walker_control.speed = 0.7+0.1*j
                # walker_heading = -90+(i+j-3)*2*((-1)**i)
                walker_control.speed = 0.2
                #walker_control.speed = 0.205+0.005*(j-3)**2
                # walker_heading = 0+(i+j-3)*2*((-1)**i) #near scl
                walker_heading = -110
                walker_rotation = carla.Rotation(0,walker_heading,0)
                walker_control.direction = walker_rotation.get_forward_vector()
                walker.apply_control(walker_control)

        spawned = True
        return np.array(walkers),spawned
    else:
        return None,False
