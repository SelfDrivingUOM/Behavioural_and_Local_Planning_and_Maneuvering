import carla
import random
import numpy as np

def jaywalking(client,ego_state,vehicle_id_list,all_id):
    x_walk2=-14 # 33 near scl
    y_walk2=-93#126
    head = 135#-135
    walkers=[]
    dist = np.linalg.norm([ego_state[0]-x_walk2,ego_state[1]-y_walk2])
    if (dist<40):
        print('\ndestroying %d vehicles' % len(vehicle_id_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_id_list])
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])
        
        blueprint_library = client.get_world().get_blueprint_library()
        blueprintsWalkers = blueprint_library.filter("walker.pedestrian.*")

        walker_bp = random.choice(blueprintsWalkers)
        walker_transform=carla.Transform(carla.Location(x=x_walk2, y=y_walk2, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
        walker = client.get_world().spawn_actor(walker_bp, walker_transform)

        if(walker!=None):

            walker_control = carla.WalkerControl()
            walker_control.speed = 0.2
            walker_heading = head
            walker_rotation = carla.Rotation(0,walker_heading,0)
            walker_control.direction = walker_rotation.get_forward_vector()
            walker.apply_control(walker_control)
            walkers.append(walker)



        x_walk3=-5
        y_walk3=-81


        for i in range(2):
            walker_bp = random.choice(blueprintsWalkers)
            # walker_transform=carla.Transform(carla.Location(x=x_walk2+(NUMBER_OF_STUDENT_IN_LEFT-i), y=y_walk2-j, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker_transform=carla.Transform(carla.Location(x=x_walk3+i, y=y_walk3-i, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

            if(walker!=None):
                walkers.append(walker)
                walker_control = carla.WalkerControl()
                # walker_control.speed = 0.7+0.1*j
                # walker_heading = -90+(i+j-3)*2*((-1)**i)
                walker_control.speed = 1.5
                #walker_control.speed = 0.205+0.005*(j-3)**2
                # walker_heading = 0+(i+j-3)*2*((-1)**i) #near scl
                walker_heading = -110
                walker_rotation = carla.Rotation(0,walker_heading,0)
                walker_control.direction = walker_rotation.get_forward_vector()
                walker.apply_control(walker_control)


        x_walk4= 0
        y_walk4=-97


        for i in range(4):
            walker_bp = random.choice(blueprintsWalkers)
            # walker_transform=carla.Transform(carla.Location(x=x_walk2+(NUMBER_OF_STUDENT_IN_LEFT-i), y=y_walk2-j, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker_transform=carla.Transform(carla.Location(x=x_walk4-i, y=y_walk4-i, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

            if(walker!=None):
                walkers.append(walker)
                walker_control = carla.WalkerControl()
                # walker_control.speed = 0.7+0.1*j
                # walker_heading = -90+(i+j-3)*2*((-1)**i)
                walker_control.speed = 3
                #walker_control.speed = 0.205+0.005*(j-3)**2
                # walker_heading = 0+(i+j-3)*2*((-1)**i) #near scl
                walker_heading = 90
                walker_rotation = carla.Rotation(0,walker_heading,0)
                walker_control.direction = walker_rotation.get_forward_vector()
                walker.apply_control(walker_control)

        x_walk5= -62
        y_walk5= -81
        for i in range(3):
            walker_bp = random.choice(blueprintsWalkers)
            # walker_transform=carla.Transform(carla.Location(x=x_walk2+(NUMBER_OF_STUDENT_IN_LEFT-i), y=y_walk2-j, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker_transform=carla.Transform(carla.Location(x=x_walk5-i, y=y_walk5, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

            if(walker!=None):
                walkers.append(walker)
                walker_control = carla.WalkerControl()
                # walker_control.speed = 0.7+0.1*j
                # walker_heading = -90+(i+j-3)*2*((-1)**i)
                walker_control.speed = 1.5
                #walker_control.speed = 0.205+0.005*(j-3)**2
                # walker_heading = 0+(i+j-3)*2*((-1)**i) #near scl
                walker_heading = -17
                walker_rotation = carla.Rotation(0,walker_heading,0)
                walker_control.direction = walker_rotation.get_forward_vector()
                walker.apply_control(walker_control)

        x_walk6= 7
        y_walk6=-81

        for i in range(3):
            walker_bp = random.choice(blueprintsWalkers)
            # walker_transform=carla.Transform(carla.Location(x=x_walk2+(NUMBER_OF_STUDENT_IN_LEFT-i), y=y_walk2-j, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker_transform=carla.Transform(carla.Location(x=x_walk6-i, y=y_walk6, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
            walker = client.get_world().try_spawn_actor(walker_bp, walker_transform)

            if(walker!=None):
                walkers.append(walker)
                walker_control = carla.WalkerControl()
                # walker_control.speed = 0.7+0.1*j
                # walker_heading = -90+(i+j-3)*2*((-1)**i)
                walker_control.speed = 5.7
                #walker_control.speed = 0.205+0.005*(j-3)**2
                # walker_heading = 0+(i+j-3)*2*((-1)**i) #near scl
                walker_heading = -90
                walker_rotation = carla.Rotation(0,walker_heading,0)
                walker_control.direction = walker_rotation.get_forward_vector()
                walker.apply_control(walker_control)



        spawned = True
        return np.array(walkers),spawned

    else:
        return None,False