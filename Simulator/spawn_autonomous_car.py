#!/usr/bin/env python

"""Spawn autonomous car (vehicle that will be controlled by our software) into the simulation"""

import glob
import os
import sys

try:
    sys.path.append(glob.glob('./dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random
import numpy as np
import pygame
import argparse


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('-s', '--show', action='store_true', help='Display data from camera sensors')
    args = argparser.parse_args()

    autonomous_car = None
    camera_left = None; camera_right = None # stereo pair
    camera_top = None # for observation and monitoring

    try:
        # create simulator client
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)

        # get simulator world
        world = client.get_world()

        blueprint_library = world.get_blueprint_library()

        # spawn vehicle
        vehicle_blueprint = blueprint_library.find('vehicle.toyota.prius') # https://carla.readthedocs.io/en/latest/bp_library/#vehicle
        vehicle_blueprint.set_attribute('role_name', 'autonomous_car')
        if vehicle_blueprint.has_attribute('color'):
            color = random.choice(vehicle_blueprint.get_attribute('color').recommended_values)
            vehicle_blueprint.set_attribute('color', color)
        vehicle_transform = random.choice(world.get_map().get_spawn_points())
        autonomous_car = world.spawn_actor(vehicle_blueprint, vehicle_transform)
        autonomous_car.set_autopilot(False)
        
        def create_camera_blueprint(role_name, fov):
            camera_blueprint = blueprint_library.find('sensor.camera.rgb')
            camera_blueprint.set_attribute('role_name', role_name)
            camera_blueprint.set_attribute('image_size_x', '1920')
            camera_blueprint.set_attribute('image_size_y', '1080')
            camera_blueprint.set_attribute('fov', fov) # sensor field of view
            camera_blueprint.set_attribute('sensor_tick', '0.0417') # time in seconds between sensor captures
            return camera_blueprint

        camera_left_blueprint = create_camera_blueprint('autonomous_car_camera_left', '60')
        # location: https://carla.readthedocs.io/en/latest/python_api/#carlalocationcarlavector3d-class
        # rotation: https://carla.readthedocs.io/en/latest/python_api/#carlarotation-class
        camera_left_transform = carla.Transform(carla.Location(x=0, y=-0.5, z=2), carla.Rotation(pitch=0, yaw=0, roll=0))
        camera_left = world.spawn_actor(camera_left_blueprint, camera_left_transform, attach_to=autonomous_car)

        camera_right_blueprint = create_camera_blueprint('autonomous_car_camera_right', '60')
        camera_right_transform = carla.Transform(carla.Location(x=0, y=0.5, z=2), carla.Rotation(pitch=0, yaw=0, roll=0))
        camera_right = world.spawn_actor(camera_right_blueprint, camera_right_transform, attach_to=autonomous_car)

        camera_top_blueprint = create_camera_blueprint('autonomous_car_camera_top', '90')
        camera_top_transform = carla.Transform(carla.Location(x=-8, y=0, z=4), carla.Rotation(pitch=-15, yaw=0, roll=0))
        camera_top = world.spawn_actor(camera_top_blueprint, camera_top_transform, attach_to=autonomous_car)

        # test
        if args.show:
            pygame.init()
            pygame.font.init()
            font = pygame.font.Font(pygame.font.get_default_font(), 20)
            clock = pygame.time.Clock()
            display = pygame.display.set_mode((852, 600))
            pygame.display.set_caption('Cam test')

            def display_image(carla_image, display_scale, start_position, cam_name):
                array = np.frombuffer(carla_image.raw_data, dtype=np.dtype('uint8'))
                array = np.reshape(array, (carla_image.height, carla_image.width, 4))
                array = array[:, :, :3]
                array = array[:, :, ::-1]
                surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
                surface = pygame.transform.scale(surface, display_scale)
                display.blit(surface, start_position)
                text = font.render(cam_name, True, (255, 255, 255))
                display.blit(text, start_position)

            camera_left.listen(lambda data: display_image(data, (426, 240), (0, 0), 'camera_left'))
            camera_right.listen(lambda data: display_image(data, (426, 240), (426, 0), 'camera_right'))
            camera_top.listen(lambda data: display_image(data, (852, 360), (0, 240), 'camera_top'))

            while True:
                pygame.display.flip()
                clock.tick(24)
        else:
            print('autonomous car and cameras spawned')
            while True:
                world.wait_for_tick()

    finally:
        if autonomous_car is not None:
            autonomous_car.destroy()
        if camera_left is not None:
            camera_left.destroy()
        if camera_right is not None:
            camera_right.destroy()
        if camera_top is not None:
            camera_top.destroy()
        if args.show:
            pygame.quit()
        print('autonomous car and cameras destroyed')


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
