#!/usr/bin/env python

from enum import Enum
import math
import time
import pygame
import numpy as np

# Import the new ros2 libraries
import rclpy
from rclpy.node import Node
from rclpy import logging

from pygame.locals import *
import pygame.transform
import pygame.image

from marvelmind_ros2_msgs.msg import HedgePositionAngle
from rover_messages.msg import Command, Odometry, RoverHeading

HEADING_LINE_LENGTH_PX = 25
POSITION_CIRCLE_RADIUS_PX = 5

class RGBColors(Enum):
    COLOR_RED = (255, 0, 0)
    COLOR_GREEN = (0, 255, 0)
    COLOR_BLUE = (0, 0, 255)
    COLOR_LIGHT_GRAY = (204, 204, 204)
    COLOR_BLACK = (0, 0, 0)
    COLOR_WHITE = (255, 255, 255)

def get_shade_of_color(color, shade_percent):
    return (
        color[0] * shade_percent,
        color[1] * shade_percent,
        color[2] * shade_percent
    )


class ToolEngine(Node):
    x_min = -1
    x_max = 1
    y_min = -1
    y_max = 1
    screen_width_px = 480
    screen_height_px = 480
    color_bg = RGBColors.COLOR_LIGHT_GRAY.value
    refresh_rate_hz = 100

    left_beacon_pos = (0, 0)
    right_beacon_pos = (0, 0)
    center_beacon_pos = (0, 0)
    center_beacon_pos_estimate = (0, 0)
    rover_heading_angle_estimate_deg = 135
    setpoint_pos = (0, 0)
    relative_heading_angle_deg = 0
    speed_m_s = 0

    previous_iteration_timestamp_s = 0

    command_duration_timer_s = 0
    command_duration_s = 0

    position_history = []

    def __init__(
        self,
        initialize_ros_callback, draw_callback, initialize_variables_callback,
        window_title, node_name,
        x_min, x_max,
        y_min, y_max,
        screen_width_px, screen_height_px,
        color_bg = RGBColors.COLOR_LIGHT_GRAY.value,
        refresh_rate_hz = 100
    ):

        self.x_max = x_max
        self.x_min = x_min
        self.y_max = y_max
        self.y_min = y_min
        self.screen_height_px = screen_height_px
        self.screen_width_px = screen_width_px
        self.color_bg = color_bg
        self.refresh_rate_hz = refresh_rate_hz

        self.draw_callback = draw_callback

        super().__init__(node_name)

        initialize_ros_callback(self)

        pygame.init()
        self.screen = pygame.display.set_mode((self.screen_width_px,
                                               self.screen_height_px))

        pygame.display.set_caption(window_title)

        self.screen.fill(self.color_bg)
        pygame.display.update()

        initialize_variables_callback(self)

        # created a timer instead of having a while loop in run, it will invoke the run every 0.1 seconds
        self.timer = self.create_timer(1 / self.refresh_rate_hz, self.run)



    def run(self):
        time_delta_s = time.time() - self.previous_iteration_timestamp_s
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.destroy_timer(self.timer)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.position_history = []

        self.screen.fill(self.color_bg)


        self.draw_callback(self, time_delta_s)

        self.previous_iteration_timestamp_s = time.time()

        pygame.display.update()

    def _pos_to_pixels(self, pos):
        """ Convert from meters to screen position. """
        x = int((pos[0] - self.x_min) * (self.screen_width_px)/(self.x_max - self.x_min))
        y = int((pos[1] - self.y_min) * (self.screen_height_px)/(self.y_max - self.y_min))
        return (x, y)

    def _line_end_coord(self, start_pos, len, angle_deg):
        x_start_px, y_start_px = self._pos_to_pixels(start_pos)
        x_end = x_start_px + len * math.cos(math.radians(angle_deg))
        y_end = y_start_px + len * math.sin(math.radians(angle_deg))
        return (x_end, y_end)

    def draw_point(self, pos, color):
        pygame.draw.circle(self.screen, color, self._pos_to_pixels(pos), 5)

    def draw_line_two_points(self, pos1, pos2, color):
        pygame.draw.line(self.screen, color, self._pos_to_pixels(pos1), self._pos_to_pixels(pos2))

    def draw_line_one_point(self, pos, length, angle_deg, color):
        pygame.draw.line(self.screen, color, self._pos_to_pixels(pos), self._line_end_coord(pos, length, angle_deg))

    def draw_heading_indicator(self, pos, angle_deg, color):
        self.draw_line_one_point(pos, HEADING_LINE_LENGTH_PX, angle_deg, color)
