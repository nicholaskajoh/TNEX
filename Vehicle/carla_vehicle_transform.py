#!/usr/bin/env python

'''
Send vehicle transform from CARLA to carla_transform channel.
'''

import glob
import os
import sys

try:
    sys.path.append(glob.glob('%s/dist/carla-*%d.%d-%s.egg' % (
        os.path.dirname(os.path.realpath(__file__)),
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import redis
import time
import pickle


try:
    r = redis.Redis()
except redis.ConnectionError as e:
    print(e)

def main():
    # create simulator client
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)

    # get simulator world
    world = client.get_world()

    # get vehicle
    vehicle = None
    for actor in world.get_actors():
        if actor.attributes.get('role_name') == 'autonomous_car':
            vehicle = actor

    if vehicle is None:
        raise Exception('Could not find vehicle')

    print('Streaming vehicle transform')

    try:
        while True:
            carla_transform = vehicle.get_transform()
            transform = {
                'location': {'x': carla_transform.location.x, 'y': carla_transform.location.y, 'z': carla_transform.location.z},
                'rotation': {'pitch': carla_transform.rotation.pitch, 'yaw': carla_transform.rotation.yaw, 'roll': carla_transform.rotation.roll},
            }

            transform_message = pickle.dumps(transform)
            r.publish('carla_transform', transform_message)

            time.sleep(0.25)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
