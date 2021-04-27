#!/usr/bin/env python

import pygame

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import mission_control

rospy.init_node('mctrl_3pviewer')

cv_bridge = CvBridge()

pygame.init()
pygame.font.init()
font = pygame.font.Font(pygame.font.get_default_font(), 20)
display_scale = (720, 480)
display = pygame.display.set_mode(display_scale)
clock = pygame.time.Clock()
pygame.display.set_caption('Third-person Viewer')

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

    while not rospy.is_shutdown():
        mission_control.send_vehicle_controls()

        pygame.display.flip()
        clock.tick(30)

def stop():
    pygame.quit()

try:
    rospy.on_shutdown(stop)
    start()
except Exception as e:
    rospy.logerr('Error occured: ' + str(e))
