import redis
import pickle
import cv2
import json

import pygame
from pygame.locals import K_RIGHT
from pygame.locals import K_LEFT
from pygame.locals import K_UP
from pygame.locals import K_DOWN
from pygame.locals import K_r


try:
    r = redis.Redis()
except redis.ConnectionError as e:
    print(e)

def main():
    pygame.init()
    pygame.font.init()
    font = pygame.font.Font(pygame.font.get_default_font(), 20)
    clock = pygame.time.Clock()
    display_scale = (1280, 720)
    display = pygame.display.set_mode(display_scale)
    pygame.display.set_caption('Cam top')

    cam_top = r.pubsub()
    cam_top.subscribe('cam_top')

    throttle = 0.0
    steer = 0.0
    brake = 0.0
    reverse = False

    while True:
        cam_top_message = cam_top.get_message()
        if cam_top_message and type(cam_top_message['data']) is not int:
            image = pickle.loads(cam_top_message['data'])
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            surface = pygame.surfarray.make_surface(image.swapaxes(0, 1))
            surface = pygame.transform.scale(surface, display_scale)
            display.blit(surface, (0, 0))

        keys = pygame.key.get_pressed()

        if keys[K_UP]:
            throttle = 0.5
        else:
            throttle = 0.0

        if keys[K_DOWN]:
            brake = 1.0
        else:
            brake = 0.0

        if keys[K_LEFT]:
            steer = -0.25
        elif keys[K_RIGHT]:
            steer = 0.25
        else:
            steer = 0.0

        if keys[K_r]:
            reverse = False if reverse else True

        pygame.event.pump()

        control_params = {'throttle': throttle, 'steer': steer, 'brake': brake, 'reverse': reverse}
        control_message = pickle.dumps(control_params, protocol=0)
        r.publish('carla_control', control_message)

        text = font.render(json.dumps(control_params), True, (255, 255, 255))
        display.blit(text, (0, 0))

        pygame.display.flip()
        clock.tick(30)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
