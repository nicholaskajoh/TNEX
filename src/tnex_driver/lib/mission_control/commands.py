'''Execute commands from 3rd person viewer'''

import pygame
from .vehicle_state import get_state, set_state


def listen_for_commands():
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_c:
                toggle_cruise_control(5)

            if event.key == pygame.K_v:
                toggle_cruise_control(10)


def toggle_cruise_control(target_speed):
    enabled = bool(int(get_state('cruise_control_enabled', '0')))
    current_target_speed = int(get_state('cruise_control_target_speed', 0))
    if enabled and target_speed == current_target_speed:
        set_state('cruise_control_enabled', 0)
        set_state('cruise_control_target_speed', 0)
    else:
        set_state('cruise_control_enabled', 1)
        set_state('cruise_control_target_speed', target_speed)
