#!/usr/bin/env python

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import time
from os_carla import WINDOWS,YASINTHA_WINDOWS,GERSHOM_WINDOWS

if WINDOWS:

    try:
        sys.path.append(glob.glob('C:/Carla0.99/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])

    except IndexError:
        pass

elif YASINTHA_WINDOWS:

    try:
        sys.path.append(glob.glob('C:/Users/4Axis/Desktop/Project/Carla/WindowsNoEditor/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])
    except IndexError:
        pass

elif GERSHOM_WINDOWS:
    try:
        sys.path.append(glob.glob('D:/WindowsNoEditor/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])
    except IndexError:
        pass
else:
    try:
        sys.path.append(glob.glob('/home/selfdriving/carla-precompiled/CARLA_0.9.9/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg' )[0])
    except IndexError:
        pass

  


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

_HOST_ = '127.0.0.1'
_PORT_ = 2000
_SLEEP_TIME_ = 0.2


def main():
	client = carla.Client(_HOST_, _PORT_)
	client.set_timeout(2.0)
	world = client.get_world()

	while(True):
            spec = world.get_spectator()
            t = spec.get_transform()
            # t.location.z=20
            t.rotation.roll =0
            t.rotation.pitch = -90
            t.rotation.yaw = 0
            spec.set_transform(t)

            p=t
            p.location.z=22
            world.debug.draw_string(p.location,'X', draw_shadow=False,color=carla.Color(r=0, g=255, b=0), life_time=0.1,persistent_lines=True)
            world.debug.draw_line(p.location,carla.Location(x=p.location.x+1 , y=p.location.y,z=p.location.z), thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=0.1)
            world.debug.draw_line(p.location,carla.Location(x=p.location.x , y=p.location.y+1,z=p.location.z), thickness=0.1, color=carla.Color(r=0, g=255, b=0), life_time=0.1)
            crd=p
            crd.location.x=crd.location.x+0.7
            crd.location.crd=p.location.y+0.7
            x='x='+str(p.location.x)+',y='+str(p.location.y)+',z='+str(p.location.z)
            world.debug.draw_string(crd.location,x, draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=0.1,persistent_lines=True)
            # coordinate_str = "(x,y) = ({},{})".format(t.location.x, t.location.y)
            coordinate_str = "(x,y,z) = ({},{},{})".format(t.location.x, t.location.y,t.location.z)
            print (coordinate_str)
            time.sleep(_SLEEP_TIME_)



if __name__ == '__main__':
	main()