import pygame
from pygame.locals import K_RIGHT
from pygame.locals import K_LEFT
from pygame.locals import K_UP
from pygame.locals import K_DOWN
from pygame.locals import K_r

import rospy
from tnex_driver.msg import VehicleControl


pub = rospy.Publisher('vehicle_control', VehicleControl, queue_size=10)
throttle = 0.0
steer = 0.0
brake = 0.0
reverse = False

def send_vehicle_controls():
    global throttle
    global steer
    global brake
    global reverse

    keys = pygame.key.get_pressed()

    if keys[K_UP]:
        throttle = 1.0
    else:
        throttle = 0.0

    if keys[K_DOWN]:
        brake = 1.0
    else:
        brake = 0.0

    if keys[K_LEFT]:
        steer = -1.0
    elif keys[K_RIGHT]:
        steer = 1.0
    else:
        steer = 0.0

    if keys[K_r]:
        reverse = False if reverse else True

    pygame.event.pump()

    msg = VehicleControl()
    msg.throttle = throttle
    msg.steer = steer
    msg.brake = brake
    msg.reverse = reverse
    pub.publish(msg)