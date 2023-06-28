#!/usr/bin/env python

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

X_MIN = -1
X_MAX = 1
Y_MIN = -1
Y_MAX = 1

SCREEN_WIDTH_PX = 480
SCREEN_HEIGHT_PX = 480


def pos_to_pixels(pos):
    """ Convert from meters to screen position. """
    x = int((pos[0] - X_MIN) * (SCREEN_WIDTH_PX)/(X_MAX - X_MIN))
    y = int((pos[1] - Y_MIN) * (SCREEN_HEIGHT_PX)/(Y_MAX - Y_MIN))
    return (x, y)

def line_end_coord(start_pos, len, angle_rad):
    x_end = start_pos[0] + len * math.cos(6.28/360 * angle_rad)
    y_end = start_pos[1] + len * math.sin(6.28/360 * angle_rad)
    return pos_to_pixels((x_end, y_end))

class VisualizerNode(Node):
    left_beacon_pos = (0, 0)
    right_beacon_pos = (0, 0)
    center_beacon_pos = (0, 0)
    center_beacon_pos_estimate = (0, 0)
    rover_heading_angle_estimate_deg = 0
    setpoint_pos = (0, 0)
    heading_angle_deg = 0

    def __init__(self):

        # Initializing the node and setting up subscriptions
        super().__init__('skibot_node')
        self.subscription = self.create_subscription(
            HedgePositionAngle, '/position_measurement/left', self.position_measurement_left_callback, 10
        )
        self.subscription = self.create_subscription(
            HedgePositionAngle, '/position_measurement/right', self.position_measurement_right_callback, 10
        )
        self.subscription = self.create_subscription(
            HedgePositionAngle, '/position_measurement', self.position_measurement_callback, 10
        )
        self.subscription = self.create_subscription(
            HedgePositionAngle, '/position_estimate', self.position_estimate_callback, 10
        )
        self.subscription = self.create_subscription(
            HedgePositionAngle, '/position_setpoint', self.position_setpoint_callback, 10
        )
        self.subscription = self.create_subscription(
            RoverHeading, '/relative_heading_setpoint', self.relative_heading_setpoint_callback, 10
        )
        # self.subscription = self.create_subscription(
        #     Command, '/relative_heading_setpoint', self.command_callback, 10
        # )

        # Start pygame...
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH_PX,
                                               SCREEN_HEIGHT_PX))
        self.refresh_rate = 100
        pygame.display.set_caption('FRIPS Simulator')

        self.screen.fill((0, 0, 255))
        pygame.display.update()

        # load and prep arrow image.
        # arrow_file = roslib.packages.resource_file('skibot', 'images',
        # 'arrow.png')

        # temporary solution... couldnt find any other way to load images
        # square = pygame.Surface((38, 38), flags=SRCALPHA)
        # square.fill((0, 0, 255, 0))



        # created a timer instead of having a while loop in run, it will invoke the run every 0.1 seconds
        self.timer = self.create_timer(1 / self.refresh_rate, self.run)

    def position_measurement_left_callback(self, pose_msg : HedgePositionAngle):
        self.left_beacon_pos = (pose_msg.x_m, pose_msg.y_m)

    def position_measurement_right_callback(self, pose_msg : HedgePositionAngle):
        self.right_beacon_pos = (pose_msg.x_m, pose_msg.y_m)

    def position_measurement_callback(self, pose_msg : HedgePositionAngle):
        self.center_beacon_pos = (pose_msg.x_m, pose_msg.y_m)

    def position_estimate_callback(self, pose_msg : HedgePositionAngle):
        # self._logger.info(f'Received position_estimate {pose_msg.x_m} {pose_msg.y_m}')
        self.center_beacon_pos_estimate = (pose_msg.x_m, pose_msg.y_m)
        self.rover_heading_angle_estimate_deg = pose_msg.angle

    def position_setpoint_callback(self, pose_msg : HedgePositionAngle):
        self.setpoint_pos = (pose_msg.x_m, pose_msg.y_m)

    def relative_heading_setpoint_callback(self, heading : RoverHeading):
        self.heading_angle_deg = heading.relative_direction_rad * 360/6.28

    # removed the while loop from here as it was not needed
    def run(self):
        last_pub = 0.0

        # destroy the timer when we get the quit from pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.destroy_timer(self.timer)

        self.screen.fill((0, 0, 255))

        # pixel_pos = pos_to_pixels((self.x,
        #                            self.y))

        # pygame.draw.circle(self.screen, (255, 0, 0), self.left_beacon_pos, 2)
        # pygame.draw.circle(self.screen, (255, 0, 0), self.right_beacon_pos, 2)
        # pygame.draw.circle(self.screen, (255, 0, 0), self.center_beacon_pos, 2)

        pygame.draw.circle(self.screen, (255, 0, 0), pos_to_pixels(self.center_beacon_pos_estimate), 5)
        pygame.draw.circle(self.screen, (0, 255, 0), pos_to_pixels(self.setpoint_pos), 5)

        HEADING_LINE_LENGTH_M = 0.1

        pygame.draw.line(self.screen, (255, 0, 0), pos_to_pixels(self.center_beacon_pos_estimate), line_end_coord(self.center_beacon_pos_estimate, HEADING_LINE_LENGTH_M, self.rover_heading_angle_estimate_deg))
        pygame.draw.line(self.screen, (0, 255, 0), pos_to_pixels(self.center_beacon_pos_estimate), line_end_coord(self.center_beacon_pos_estimate, HEADING_LINE_LENGTH_M, self.rover_heading_angle_estimate_deg + self.heading_angle_deg))

        pygame.display.update()

# Adjusted the init sequence as per other ros2 examples.


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.is_shutdown()


if __name__ == "__main__":
    main()