#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This module implements a ROS2 node for keyboard teleoperation of robots.

It listens for keyboard input and publishes geometry_msgs/Twist messages

based on the input to control a robot.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import sys
import termios
import tty
import select
import os
import threading


class KeyTeleop(Node):
    """
    ROS2 Node for keyboard teleoperation of robots.

    This node listens for keyboard input and publishes

    geometry_msgs/Twist messages based on the input to control a robot.
    """

    def __init__(self):
        """Initialize the KeyTeleop node."""
        super().__init__('keyboard_teleop')

        if os.isatty(sys.stdin.fileno()):
            self.settings = termios.tcgetattr(sys.stdin)
        else:
            self.settings = None

        self.pub_twist = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cmd_bindings = {
            'q': np.array([1, 1]),
            'w': np.array([1, 0]),
            'e': np.array([1, -1]),
            'a': np.array([0, 1]),
            'd': np.array([0, -1]),
            'z': np.array([-1, -1]),
            'x': np.array([-1, 0]),
            'c': np.array([-1, 1])
        }

        self.set_bindings = {
            't': np.array([1, 1]),
            'b': np.array([-1, -1]),
            'y': np.array([1, 0]),
            'n': np.array([-1, 0]),
            'u': np.array([0, 1]),
            'm': np.array([0, -1])
        }

        self.speed = np.array([0.5, 1.0])
        self.inc_ratio = 0.1
        self.command = np.array([0, 0])

        self.print_usage()

        self.input_thread = threading.Thread(target=self.monitor_keyboard_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.timer = self.create_timer(0.1, self.publish_twist_message)
        self.get_logger().info('Node is running')

    def monitor_keyboard_input(self):
        """Monitor keyboard input and update commands."""
        while True:
            ch = self.get_key()
            if ch:
                self.process_key(ch)

    def publish_twist_message(self):
        """Publish Twist messages based on configured commands."""
        self.update()

    def fini(self):
        """Restore terminal settings when exiting the program."""
        if self.settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def print_usage(self):
        """Print usage instructions to the console."""
        msg = """
        Keyboard Teleop that Publish to /cmd_vel (geometry_msgs/Twist)
        Copyright (C) 2024
        Released under Apache License
        --------------------------------------------------
        H:       Print this menu
        Moving around:
          Q   W   E
          A   S   D
          Z   X   C
        T/B :   increase/decrease max speeds 10%
        Y/N :   increase/decrease only linear speed 10%
        U/M :   increase/decrease only angular speed 10%
        anything else : stop

        G :   Quit
        --------------------------------------------------
        """
        self.get_logger().info(msg)
        self.show_status()

    def show_status(self):
        """Display the current speed settings to the console."""
        msg = 'Status:\tlinear {:.2f}\tangular {:.2f}'.format(self.speed[0], self.speed[1])
        self.get_logger().info(msg)

    def process_key(self, ch):
        """Process a single key input."""
        if ch == 'h':
            self.print_usage()
        elif ch in self.cmd_bindings:
            self.command = self.cmd_bindings[ch]
        elif ch in self.set_bindings:
            self.speed = self.speed * (1 + self.set_bindings[ch] * self.inc_ratio)
            self.show_status()
        elif ch == 'g':
            self.get_logger().info('Quitting')
            self.fini()
            twist = Twist()
            self.pub_twist.publish(twist)
            rclpy.shutdown()
        else:
            self.command = np.array([0, 0])

    def update(self):
        """Update and publish the Twist message based on current command."""
        twist = Twist()
        cmd = self.speed * self.command
        twist.linear.x = cmd[0]
        twist.angular.z = cmd[1]
        self.pub_twist.publish(twist)

    def get_key(self):
        """Get a single keystroke without blocking."""
        if self.settings is not None:
            tty.setraw(sys.stdin.fileno())
            select.select([sys.stdin], [], [], 0)
            key = sys.stdin.read(1)
            return key.lower()
        else:
            return None


def main(args=None):
    rclpy.init(args=args)
    teleop = KeyTeleop()
    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        pass
    finally:
        teleop.fini()
        teleop.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
