#!/usr/bin/env python

from .tool_engine import ToolEngine, RGBColors

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


def engine_ros_init(engine : ToolEngine):
    def position_measurement_left_callback(pose_msg : HedgePositionAngle):
        engine.left_beacon_pos = (pose_msg.x_m, pose_msg.y_m)

    def position_measurement_right_callback(pose_msg : HedgePositionAngle):
        engine.right_beacon_pos = (pose_msg.x_m, pose_msg.y_m)

    def position_measurement_callback(pose_msg : HedgePositionAngle):
        engine.center_beacon_pos = (pose_msg.x_m, pose_msg.y_m)

    def position_estimate_callback(pose_msg : HedgePositionAngle):
        # engine._logger.info(f'Received position_estimate {pose_msg.x_m} {pose_msg.y_m}')
        engine._logger.fatal(f'angle = {pose_msg.angle:3.03f}')
        engine.center_beacon_pos_estimate = (pose_msg.x_m, pose_msg.y_m)
        engine.rover_heading_angle_estimate_deg = pose_msg.angle

    def position_setpoint_callback(pose_msg : HedgePositionAngle):
        engine.setpoint_pos = (pose_msg.x_m, pose_msg.y_m)

    def relative_heading_setpoint_callback(heading : RoverHeading):
        engine.relative_heading_angle_deg = math.degrees(heading.relative_direction_rad)

    engine.subscription = engine.create_subscription(
        HedgePositionAngle, '/position_measurement/left', position_measurement_left_callback, 10
    )
    engine.subscription = engine.create_subscription(
        HedgePositionAngle, '/position_measurement/right', position_measurement_right_callback, 10
    )
    engine.subscription = engine.create_subscription(
        HedgePositionAngle, '/position_measurement', position_measurement_callback, 10
    )
    engine.subscription = engine.create_subscription(
        HedgePositionAngle, '/position_estimate', position_estimate_callback, 10
    )
    engine.subscription = engine.create_subscription(
        HedgePositionAngle, '/position_setpoint', position_setpoint_callback, 10
    )
    engine.subscription = engine.create_subscription(
        RoverHeading, '/relative_heading_setpoint', relative_heading_setpoint_callback, 10
    )
    # engine.subscription = engine.create_subscription(
    #     Command, '/relative_heading_setpoint', engine.command_callback, 10
    # )

def engine_variable_init(engine : ToolEngine):
    engine.position_history = []

def engine_draw(engine : ToolEngine, time_delta_s : float):
    heading_angle_deg = engine.rover_heading_angle_estimate_deg + engine.relative_heading_angle_deg

    engine.position_history.append(engine.center_beacon_pos_estimate)
    
    for i, position in enumerate(engine.position_history):
        # if i != len(engine.position_history) - 1:
        #     pygame.draw.circle(self.screen, (255, 255, 255), pos_to_pixels(position), 5)
        if i != 0:
            previous_position = engine.position_history[i - 1]
            engine.draw_line_two_points(previous_position, position, RGBColors.COLOR_BLACK.value)

    engine.draw_point(engine.center_beacon_pos_estimate, RGBColors.COLOR_RED.value)
    engine.draw_point(engine.setpoint_pos, RGBColors.COLOR_GREEN.value)

    engine.draw_heading_indicator(engine.center_beacon_pos_estimate, engine.rover_heading_angle_estimate_deg, RGBColors.COLOR_RED.value)
    engine.draw_heading_indicator(engine.center_beacon_pos_estimate, heading_angle_deg, RGBColors.COLOR_GREEN.value)


def main(args=None):
    rclpy.init(args=args)

    engine = ToolEngine(
        initialize_ros_callback = engine_ros_init,
        draw_callback = engine_draw,
        initialize_variables_callback = engine_variable_init,
        window_title = 'FRIPS Visualizer',
        node_name = 'visualizer_node',
        x_min = -1, x_max = 1,
        y_min = -1, y_max = 1,
        screen_width_px = 480, screen_height_px = 480,
        color_bg = RGBColors.COLOR_LIGHT_GRAY.value,
        refresh_rate_hz = 100
    )

    rclpy.spin(engine)
    engine.destroy_node()
    rclpy.is_shutdown()


if __name__ == "__main__":
    main()