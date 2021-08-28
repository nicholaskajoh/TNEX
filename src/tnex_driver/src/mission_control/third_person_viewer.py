#!/usr/bin/env python

import pygame
import numpy as np

import rospy
from sensor_msgs.msg import Image
from tnex_driver.msg import VehicleControl
from cv_bridge import CvBridge, CvBridgeError

from mission_control import ManualVehicleControl
from utils import sprites

rospy.init_node('mctrl_3pviewer')

cv_bridge = CvBridge()

pygame.init()
display_scale = (720, 480)
display = pygame.display.set_mode(display_scale)
clock = pygame.time.Clock()
pygame.display.set_caption('Third-person Viewer')
sprite_group = pygame.sprite.LayeredUpdates()
text_layer = 1

threepv_frame = sprites.Image(pygame.surfarray.make_surface(np.zeros(display_scale)), (0, 0), sprite_group, 0)
ctrls_txt = sprites.Text('CONTROLS', (600, 0), sprite_group, text_layer)
throttle_txt = sprites.Text('THROTTLE: 0', (600, 15), sprite_group, text_layer)
steer_txt = sprites.Text('STEER: 0', (600, 30), sprite_group, text_layer)
brake_txt = sprites.Text('BRAKE: 0', (600, 45), sprite_group, text_layer)
reverse_txt = sprites.Text('REVERSE: 0', (600, 60), sprite_group, text_layer)

def show_3pv(image):
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.logerr(e)
    surface = pygame.surfarray.make_surface(cv_image.swapaxes(0, 1))
    surface = pygame.transform.scale(surface, display_scale)
    threepv_frame.updateImage(surface)

def show_controls(vehicle_control):
    throttle_txt.updateText('THROTTLE: ' + str(round(vehicle_control.throttle, 2)))
    steer_txt.updateText('STEER: ' + str(round(vehicle_control.steer, 2)))
    brake_txt.updateText('BRAKE: ' + str(round(vehicle_control.brake, 2)))
    reverse_txt.updateText('REVERSE: ' + str(int(vehicle_control.reverse)))

def start():
    rospy.Subscriber('camera_3pv_rgb', Image, show_3pv)
    rospy.Subscriber('vehicle_control', VehicleControl, show_controls)

    mvc = ManualVehicleControl()

    while not rospy.is_shutdown():
        mvc.send()

        sprite_group.update()
        sprite_group.draw(display)
        pygame.display.flip()
        clock.tick(30)

def stop():
    pygame.quit()

try:
    rospy.on_shutdown(stop)
    start()
except Exception as e:
    rospy.logerr('Error occured: ' + str(e))
