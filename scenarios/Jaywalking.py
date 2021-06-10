import carla
import random
import numpy as np

def jaywalking(client,ego_state,vehicle_id_list):
    x_walk2=-14 # 33 near scl
    y_walk2=-93#126
    head = 135#-135

    dist = np.linalg.norm([ego_state[0]-x_walk2,ego_state[1]-y_walk2])
    if (dist<40):
        print('\ndestroying %d vehicles' % len(vehicle_id_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_id_list])

        
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

        # walker_transform2=carla.Transform(carla.Location(x=x_walk2, y=y_walk2, z= 1.438 ),carla.Rotation(yaw= 1.4203450679814286772))
        # walker2 = client.get_world().spawn_actor(walker_bp, walker_transform2)
        

        # if(walker2!=None):

        #     walker_control2 = carla.WalkerControl()
        #     walker_control2.speed = 0.3
        #     walker_heading2 = -90
        #     walker_rotation2 = carla.Rotation(0,walker_heading2,0)
        #     walker_control2.direction = walker_rotation2.get_forward_vector()
        #     walker2.apply_control(walker_control2)

        walkers=np.array([walker])
        spawned = True


        return walkers,spawned

    else:
        return None,False
        