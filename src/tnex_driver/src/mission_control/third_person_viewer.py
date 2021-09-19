#!/usr/bin/env python

import pygame
import numpy as np

import rospy
from sensor_msgs.msg import Image
from tnex_driver.msg import VehicleControl
from cv_bridge import CvBridge, CvBridgeError

from mission_control import ManualVehicleControl, listen_for_commands, vehicle_state
from sensors import Speedometer
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

telem_txt = sprites.Text('TELEMETRY', (1, 0), sprite_group, text_layer)
vmode_txt = sprites.Text('MODE: MANUAL', (1, 15), sprite_group, text_layer)
speed_txt = sprites.Text('SPEED: 0 M/S', (1, 30), sprite_group, text_layer)
threepv_frame = sprites.Image(pygame.surfarray.make_surface(np.zeros(display_scale)), (0, 0), sprite_group, 0)
ctrls_txt = sprites.Text('CONTROLS', (1, 60), sprite_group, text_layer)
throttle_txt = sprites.Text('THROTTLE: 0', (1, 75), sprite_group, text_layer)
steer_txt = sprites.Text('STEER: 0', (1, 90), sprite_group, text_layer)
brake_txt = sprites.Text('BRAKE: 0', (1, 105), sprite_group, text_layer)
reverse_txt = sprites.Text('REVERSE: 0', (1, 120), sprite_group, text_layer)
sprites.Text('COMMANDS', (450, 0), sprite_group, text_layer)
sprites.Text('THROTTLE: UP', (450, 15), sprite_group, text_layer)
sprites.Text('STEER: LEFT, RIGHT', (450, 30), sprite_group, text_layer)
sprites.Text('BRAKE: DOWN', (450, 45), sprite_group, text_layer)
sprites.Text('REVERSE TOGGLE: R', (450, 60), sprite_group, text_layer)
sprites.Text('MANUAL: M', (450, 75), sprite_group, text_layer)
sprites.Text('CRUISE CONTROL (5, 10): C, V', (450, 90), sprite_group, text_layer)

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
    speedometer = Speedometer()

    while not rospy.is_shutdown():
        mvc.send()
        listen_for_commands()

        speed_txt.updateText('SPEED: ' + str(round(speedometer.get_speed(), 2)) + ' M/S')
        vmode_txt.updateText('MODE: ' + vehicle_state.get_state('vehicle_mode', 'MANUAL'))

        sprite_group.update()
        sprite_group.draw(display)
        pygame.event.pump()
        pygame.display.flip()
        clock.tick(30)

def stop():
    pygame.quit()

try:
    rospy.on_shutdown(stop)
    start()
except Exception as e:
    rospy.logerr('Error occured: ' + str(e))
