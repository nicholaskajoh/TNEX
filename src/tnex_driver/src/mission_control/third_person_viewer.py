#!/usr/bin/env python

import pygame
from pygame.locals import K_RIGHT
from pygame.locals import K_LEFT
from pygame.locals import K_UP
from pygame.locals import K_DOWN
from pygame.locals import K_r

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tnex_driver.msg import VehicleControl

rospy.init_node('mctrl_3pviewer')

cv_bridge = CvBridge()
throttle = 0.0
steer = 0.0
brake = 0.0
reverse = False

pygame.init()
pygame.font.init()
font = pygame.font.Font(pygame.font.get_default_font(), 20)
display_scale = (720, 480)
display = pygame.display.set_mode(display_scale)
clock = pygame.time.Clock()
pygame.display.set_caption('Third-person Viewer')


def send_controls(pub):
    global throttle
    global steer
    global brake
    global reverse

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

    msg = VehicleControl()
    msg.throttle = throttle
    msg.steer = steer
    msg.brake = brake
    msg.reverse = reverse
    pub.publish(msg)

def show_3pv(image):
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.logerr(e)
    surface = pygame.surfarray.make_surface(cv_image.swapaxes(0, 1))
    surface = pygame.transform.scale(surface, display_scale)
    display.blit(surface, (0, 0))

def start():
    rospy.Subscriber('camera_3pv_rgb', Image, show_3pv)
    pub = rospy.Publisher('vehicle_control', VehicleControl, queue_size=10)

    while not rospy.is_shutdown():
        send_controls(pub)

        pygame.display.flip()
        clock.tick(30)

def stop():
    pygame.quit()

try:
    rospy.on_shutdown(stop)
    start()
except Exception as e:
    rospy.logerr('Error occured: ' + str(e))
