import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# connect to Carla
client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()

world.unload_map_layer(carla.MapLayer.GuardRails)
    
