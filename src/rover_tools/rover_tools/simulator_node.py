#!/usr/bin/env python

import math
import random
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

class SimulatorNode(Node):
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

    def __init__(self):

        # Initializing the node and setting up subscriptions
        super().__init__('skibot_node')
        self.subscription_1 = self.create_subscription(
            HedgePositionAngle, '/position_measurement/left', self.position_measurement_left_callback, 10
        )
        self.subscription_2 = self.create_subscription(
            HedgePositionAngle, '/position_measurement/right', self.position_measurement_right_callback, 10
        )
        self.subscription_3 = self.create_subscription(
            HedgePositionAngle, '/position_measurement', self.position_measurement_callback, 10
        )
        # self.subscription_4 = self.create_subscription(
        #     HedgePositionAngle, '/position_estimate', self.position_estimate_callback, 10
        # )
        self.subscription_5 = self.create_subscription(
            HedgePositionAngle, '/position_setpoint', self.position_setpoint_callback, 10
        )
        self.subscription_6 = self.create_subscription(
            RoverHeading, '/relative_heading_setpoint', self.relative_heading_setpoint_callback, 10
        )
        self.position_estimate_publisher = self.create_publisher(
            HedgePositionAngle, '/position_estimate', 10
        )

        # Start pygame...
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH_PX,
                                               SCREEN_HEIGHT_PX))
        self.refresh_rate = 100
        pygame.display.set_caption('FRIPS Simulator')

        self.screen.fill((0, 0, 255))
        pygame.display.update()

        self.position_history = []



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
        self.speed_m_s = heading.speed_m_s
        self.relative_heading_angle_deg = math.degrees(heading.relative_direction_rad)
        self.command_duration_s = heading.durations_ms / 1000
        self.command_duration_timer_s = 0

    # removed the while loop from here as it was not needed
    def run(self):
        time_delta_s = time.time() - self.previous_iteration_timestamp_s
        # destroy the timer when we get the quit from pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.destroy_timer(self.timer)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.position_history = []

        self.screen.fill((0, 0, 255))

        heading_angle_deg = self.rover_heading_angle_estimate_deg + self.relative_heading_angle_deg

        # pixel_pos = pos_to_pixels((self.x,
        #                            self.y))

        # pygame.draw.circle(self.screen, (255, 0, 0), self.left_beacon_pos, 2)
        # pygame.draw.circle(self.screen, (255, 0, 0), self.right_beacon_pos, 2)
        # pygame.draw.circle(self.screen, (255, 0, 0), self.center_beacon_pos, 2)

        if self.command_duration_timer_s < self.command_duration_s:
            displacement_m = self.speed_m_s * time_delta_s
            displacement_x_m = displacement_m * math.cos(math.radians(heading_angle_deg))
            displacement_y_m = displacement_m * math.sin(math.radians(heading_angle_deg))

            old_x, old_y = self.center_beacon_pos_estimate
            new_x, new_y = (old_x + displacement_x_m), (old_y + displacement_y_m)
            self.center_beacon_pos_estimate = (new_x, new_y)

            self.command_duration_timer_s += time_delta_s



        pygame.draw.circle(self.screen, (255, 0, 0), pos_to_pixels(self.center_beacon_pos_estimate), 5)
        pygame.draw.circle(self.screen, (0, 255, 0), pos_to_pixels(self.setpoint_pos), 5)

        HEADING_LINE_LENGTH_M = 0.1

        pygame.draw.line(self.screen, (255, 0, 0), pos_to_pixels(self.center_beacon_pos_estimate), line_end_coord(self.center_beacon_pos_estimate, HEADING_LINE_LENGTH_M, self.rover_heading_angle_estimate_deg))
        pygame.draw.line(self.screen, (0, 255, 0), pos_to_pixels(self.center_beacon_pos_estimate), line_end_coord(self.center_beacon_pos_estimate, HEADING_LINE_LENGTH_M, heading_angle_deg))

        self.position_history.append(self.center_beacon_pos_estimate)

        for i, position in enumerate(self.position_history):
            # if i != len(self.position_history) - 1:
            #     pygame.draw.circle(self.screen, (255, 255, 255), pos_to_pixels(position), 5)
            if i != 0:
                previous_position = self.position_history[i - 1]
                pygame.draw.line(self.screen, (255, 255, 255), pos_to_pixels(previous_position), pos_to_pixels(position))

        msg = HedgePositionAngle()
        x_m, y_m = self.center_beacon_pos_estimate
        msg.x_m, msg.y_m = float(x_m), float(y_m)
        msg.angle = float(self.rover_heading_angle_estimate_deg)

        self.position_estimate_publisher.publish(msg)
        self.previous_iteration_timestamp_s = time.time()

        pygame.display.update()

# Adjusted the init sequence as per other ros2 examples.


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.is_shutdown()


if __name__ == "__main__":
    main()