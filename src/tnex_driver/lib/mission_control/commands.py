'''Execute commands from 3rd person viewer'''

import pygame
from .vehicle_state import get_state, set_state


def listen_for_commands():
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_c:
                toggle_cruise_control()


def toggle_cruise_control():
    enabled = bool(int(get_state('cruise_control_enabled', '0')))
    if not enabled:
        set_state('cruise_control_enabled', 1)
    else:
        set_state('cruise_control_enabled', 0)
