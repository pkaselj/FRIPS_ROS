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

POSITION_PUBLISHER_RATE_HZ = 10

def engine_ros_init(engine : ToolEngine):
    def position_measurement_left_callback(pose_msg : HedgePositionAngle):
        engine.left_beacon_pos = (pose_msg.x_m, pose_msg.y_m)

    def position_measurement_right_callback(pose_msg : HedgePositionAngle):
        engine.right_beacon_pos = (pose_msg.x_m, pose_msg.y_m)

    def position_measurement_callback(pose_msg : HedgePositionAngle):
        engine.center_beacon_pos = (pose_msg.x_m, pose_msg.y_m)

    def position_estimate_callback(pose_msg : HedgePositionAngle):
        # self._logger.info(f'Received position_estimate {pose_msg.x_m} {pose_msg.y_m}')
        engine.center_beacon_pos_estimate = (pose_msg.x_m, pose_msg.y_m)
        engine.rover_heading_angle_estimate_deg = pose_msg.angle

    def position_setpoint_callback(pose_msg : HedgePositionAngle):
        engine.setpoint_pos = (pose_msg.x_m, pose_msg.y_m)

    def relative_heading_setpoint_callback(heading : RoverHeading):
        engine.speed_m_s = heading.speed_m_s
        engine.relative_heading_angle_deg = math.degrees(heading.relative_direction_rad)
        engine.command_duration_s = heading.durations_ms / 1000
        engine.command_duration_timer_s = 0

    def publish_positional_data_timer_callback():
        engine.position_estimate_publisher.publish(engine.position_data_to_be_published)

    engine.subscription_1 = engine.create_subscription(
        HedgePositionAngle, '/position_measurement/left', position_measurement_left_callback, 10
    )
    engine.subscription_2 = engine.create_subscription(
        HedgePositionAngle, '/position_measurement/right', position_measurement_right_callback, 10
    )
    engine.subscription_3 = engine.create_subscription(
        HedgePositionAngle, '/position_measurement', position_measurement_callback, 10
    )
    # engine.subscription_4 = engine.create_subscription(
    #     HedgePositionAngle, '/position_estimate', position_estimate_callback, 10
    # )
    engine.subscription_5 = engine.create_subscription(
        HedgePositionAngle, '/position_setpoint', position_setpoint_callback, 10
    )
    engine.subscription_6 = engine.create_subscription(
        RoverHeading, '/relative_heading_setpoint', relative_heading_setpoint_callback, 10
    )
    engine.position_estimate_publisher = engine.create_publisher(
        HedgePositionAngle, '/position_estimate', 10
    )
    engine.position_estimate_publisher_timer = engine.create_timer(
        1.0 / POSITION_PUBLISHER_RATE_HZ, publish_positional_data_timer_callback
    )

def engine_variable_init(engine : ToolEngine):
    engine.position_history = []

    engine.position_data_to_be_published = HedgePositionAngle()
    engine.position_data_to_be_published.x_m = 0.0
    engine.position_data_to_be_published.y_m = 0.0
    engine.position_data_to_be_published.z_m = 0.0
    engine.position_data_to_be_published.angle = 0.0

def engine_draw(engine : ToolEngine, time_delta_s : float):
    heading_angle_deg = engine.rover_heading_angle_estimate_deg + engine.relative_heading_angle_deg


    if engine.command_duration_timer_s < engine.command_duration_s:
        displacement_m = engine.speed_m_s * time_delta_s
        displacement_x_m = displacement_m * math.cos(math.radians(heading_angle_deg))
        displacement_y_m = displacement_m * math.sin(math.radians(heading_angle_deg))

        old_x, old_y = engine.center_beacon_pos_estimate
        new_x, new_y = (old_x + displacement_x_m), (old_y + displacement_y_m)
        engine.center_beacon_pos_estimate = (new_x, new_y)

        engine.command_duration_timer_s += time_delta_s

    engine.draw_point(engine.center_beacon_pos_estimate, RGBColors.COLOR_RED.value)
    engine.draw_point(engine.setpoint_pos, RGBColors.COLOR_GREEN.value)

    engine.draw_heading_indicator(engine.center_beacon_pos_estimate, engine.rover_heading_angle_estimate_deg, RGBColors.COLOR_RED.value)
    engine.draw_heading_indicator(engine.center_beacon_pos_estimate, heading_angle_deg, RGBColors.COLOR_GREEN.value)

    engine.position_history.append(engine.center_beacon_pos_estimate)

    for i, position in enumerate(engine.position_history):
        # if i != len(self.position_history) - 1:
        #     pygame.draw.circle(self.screen, (255, 255, 255), pos_to_pixels(position), 5)
        if i != 0:
            previous_position = engine.position_history[i - 1]
            engine.draw_line_two_points(previous_position, position, RGBColors.COLOR_BLACK.value)

    x_m, y_m = engine.center_beacon_pos_estimate
    engine.position_data_to_be_published.x_m = float(x_m)
    engine.position_data_to_be_published.y_m = float(y_m)
    engine.position_data_to_be_published.angle = float(engine.rover_heading_angle_estimate_deg)


def main(args=None):
    rclpy.init(args=args)

    engine = ToolEngine(
        initialize_ros_callback = engine_ros_init,
        draw_callback = engine_draw,
        initialize_variables_callback = engine_variable_init,
        window_title = 'FRIPS Simulator',
        node_name = 'simulator_node',
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