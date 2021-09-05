import time

import pygame
from pygame.locals import K_RIGHT
from pygame.locals import K_LEFT
from pygame.locals import K_UP
from pygame.locals import K_DOWN
from pygame.locals import K_r

import rospy
from tnex_driver.msg import VehicleControl
from .vehicle_state import get_state


class ManualVehicleControl:
    throttle = 0.0
    steer = 0.0
    brake = 0.0
    reverse = False
    throttle_gain = 0.1
    steer_gain = 0.1
    brake_gain = 0.2
    max_throttle = 1.0
    min_throttle = 0.0
    max_steer = 1.0
    min_steer = -1.0
    no_steer = 0.0
    max_brake = 1.0
    min_brake = 0.0
    min_time_btw_inputs = 1 / 5 # seconds
    current_time = time.time()
    cc_enabled = False # DON'T touch throttle and brakes when cruise control is enabled!

    def __init__(self):
        self.pub = rospy.Publisher('vehicle_control', VehicleControl, queue_size=10)
        rospy.Subscriber('vehicle_control', VehicleControl, self.set_controls)

    def set_controls(self, vehicle_control):
        self.throttle = vehicle_control.throttle
        self.steer = vehicle_control.steer
        self.brake = vehicle_control.brake
        self.reverse = vehicle_control.reverse

    def send(self):
        self.cc_enabled = bool(int(get_state('cruise_control_enabled', '0')))

        new_current_time = time.time()
        time_delta = new_current_time - self.current_time
        if time_delta >= self.min_time_btw_inputs:
            self.current_time = new_current_time

        keys = pygame.key.get_pressed()

        # throttle
        if keys[K_UP]:
            if time_delta >= self.min_time_btw_inputs:
                self.throttle += self.throttle_gain
                if self.throttle > self.max_throttle:
                    self.throttle = self.max_throttle
        else:
            if time_delta >= self.min_time_btw_inputs and not self.cc_enabled:
                self.throttle -= self.throttle_gain
                if self.throttle < self.min_throttle:
                    self.throttle = self.min_throttle

        # brakes
        if keys[K_DOWN]:
            if time_delta >= self.min_time_btw_inputs:
                self.brake += self.brake_gain
                if self.brake > self.max_brake:
                    self.brake = self.max_brake
        else:
            if time_delta >= self.min_time_btw_inputs and not self.cc_enabled:
                self.brake -= self.brake_gain
                if self.brake < self.min_brake:
                    self.brake = self.min_brake

        # steer
        if keys[K_LEFT]:
            if time_delta >= self.min_time_btw_inputs:
                self.steer -= self.steer_gain
                if self.steer < self.min_steer:
                    self.steer = self.min_steer
        elif keys[K_RIGHT]:
            if time_delta >= self.min_time_btw_inputs:
                self.steer += self.steer_gain
                if self.steer > self.max_steer:
                    self.steer = self.max_steer
        else:
            if time_delta >= self.min_time_btw_inputs:
                if self.steer > self.no_steer and self.steer - self.steer_gain > self.no_steer:
                    self.steer -= self.steer_gain
                elif self.steer < self.no_steer and self.steer + self.steer_gain < self.no_steer:
                    self.steer += self.steer_gain
                else:
                    self.steer = self.no_steer

        # reverse
        if keys[K_r]:
            if time_delta >= self.min_time_btw_inputs:
                self.reverse = False if self.reverse else True

        msg = VehicleControl()
        msg.throttle = self.throttle
        msg.steer = self.steer
        msg.brake = self.brake
        msg.reverse = self.reverse
        self.pub.publish(msg)
