#!/usr/bin/env python

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
import numpy as np
import time
import pickle


try:
    r = redis.Redis()
except redis.ConnectionError as e:
    print(e)

def publish(carla_image, topic):
    image = np.frombuffer(carla_image.raw_data, dtype=np.dtype('uint8'))
    image = np.reshape(image, (carla_image.height, carla_image.width, 4))
    image = image[:, :, :3]
    image = image[:, :, ::-1]
    image_message = pickle.dumps(image, protocol=0)
    r.publish(topic, image_message)


def carla_video_capture():
    # create simulator client
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)

    # get simulator world
    world = client.get_world()

    # get camera sensors
    cam_left = None; cam_right = None; cam_top = None
    for actor in world.get_actors():
        if actor.attributes.get('role_name') == 'autonomous_car_camera_left':
            cam_left = actor

        if actor.attributes.get('role_name') == 'autonomous_car_camera_right':
            cam_right = actor

        if actor.attributes.get('role_name') == 'autonomous_car_camera_top':
            cam_top = actor
    if cam_left is None or cam_right is None or cam_top is None:
        raise Exception('Could not find one or more camera sensors')


    cam_left.listen(lambda carla_image: publish(carla_image, 'cam_left'))
    cam_right.listen(lambda carla_image: publish(carla_image, 'cam_right'))
    cam_top.listen(lambda carla_image: publish(carla_image, 'cam_top'))

    print('Streaming video from cameras')

    try:
        while True:
            time.sleep(5)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    carla_video_capture()
