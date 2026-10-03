#!/usr/bin/env python3

import os
import struct
import select
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


# ================================================================
# LINUX JOYSTICK EVENT
# ================================================================

EVENT_FORMAT = 'IhBB'
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

JS_DEVICE = '/dev/input/js0'


class EvoFoxTeleop(Node):

    def __init__(self):

        super().__init__('evofox_teleop')

        # ============================================================
        # EVOFOX ONE S MAPPING
        # ============================================================

        # ------------------------------------------------------------
        # LEFT JOYSTICK X
        #
        # -32767 = LEFT
        # +32767 = RIGHT
        # ------------------------------------------------------------

        self.TURN_AXIS = 0

        # ------------------------------------------------------------
        # RIGHT JOYSTICK Y
        #
        # -32767 = UP
        # +32767 = DOWN
        #
        # We will invert it:
        #
        # UP   = FORWARD
        # DOWN = BACKWARD
        # ------------------------------------------------------------

        self.MOVE_AXIS = 4

        # ------------------------------------------------------------
        # D-PAD
        #
        # CH6:
        # LEFT  = -32767
        # RIGHT = +32767
        #
        # CH7:
        # UP   = -32767
        # DOWN = +32767
        # ------------------------------------------------------------

        self.DPAD_X_AXIS = 6
        self.DPAD_Y_AXIS = 7

        # ------------------------------------------------------------
        # BUTTONS
        #
        # Button 6 = SELECT
        # Button 7 = START
        # Button 2 = X
        # ------------------------------------------------------------

        self.SELECT_BUTTON = 6
        self.START_BUTTON = 7
        self.X_BUTTON = 2

        # ============================================================
        # DEAD ZONE
        # ============================================================

        self.DEADZONE = 0.10

        # ============================================================
        # JOYSTICK TIMEOUT
        # ============================================================

        # If we stop receiving joystick events for this long,
        # stop and disarm the robot.
        #
        self.JOY_TIMEOUT = 50

        self.last_event_time = time.monotonic()

        # ============================================================
        # SPEED SETTINGS
        # ============================================================

        # Initial forward/backward ROS velocity
        self.linear_speed = 0.30

        # Initial turning ROS velocity
        self.angular_speed = 0.20

        # Maximum ROS velocities
        #
        # These are NOT DAC values.
        #
        # Arduino bridge will convert them to DAC.
        #
        self.MAX_LINEAR_SPEED = 2.70
        self.MAX_ANGULAR_SPEED = 0.70

        # Minimum speeds
        self.MIN_LINEAR_SPEED = 0.05
        self.MIN_ANGULAR_SPEED = 0.05

        # D-pad speed increments
        self.LINEAR_STEP = 0.05
        self.ANGULAR_STEP = 0.05

        # ============================================================
        # ROBOT STATE
        # ============================================================

        self.armed = False

        # ============================================================
        # CURRENT CONTROLLER STATE
        # ============================================================

        self.axes = {}
        self.buttons = {}

        # ============================================================
        # D-PAD PREVIOUS STATE
        # ============================================================

        self.prev_dpad_up = False
        self.prev_dpad_down = False
        self.prev_dpad_left = False
        self.prev_dpad_right = False

        # ============================================================
        # OPEN JOYSTICK
        # ============================================================

        try:

            self.js_fd = os.open(
                JS_DEVICE,
                os.O_RDONLY |
                os.O_NONBLOCK
            )

            self.get_logger().info(
                f'Connected to joystick: {JS_DEVICE}'
            )

        except FileNotFoundError:

            self.get_logger().error(
                f'{JS_DEVICE} not found.'
            )

            raise

        except PermissionError:

            self.get_logger().error(
                f'Permission denied for {JS_DEVICE}.'
            )

            self.get_logger().error(
                'Add your user to the input group or run '
                'with appropriate permissions.'
            )

            raise

        # ============================================================
        # CMD_VEL PUBLISHER
        # ============================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ============================================================
        # JOYSTICK READ TIMER
        # ============================================================

        # Read joystick approximately 100 times per second.
        self.read_timer = self.create_timer(
            0.01,
            self.read_joystick
        )

        # ============================================================
        # WATCHDOG
        # ============================================================

        self.watchdog_timer = self.create_timer(
            0.05,
            self.watchdog
        )

        # ============================================================
        # STARTUP MESSAGE
        # ============================================================

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            '       EVOFOX ONE S TELEOP'
        )

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            'Button 6 + Button 7 = ARM'
        )

        self.get_logger().info(
            'Button 6 + Button 2 = STOP / DISARM'
        )

        self.get_logger().info(
            'Axis 4 = Forward / Backward'
        )

        self.get_logger().info(
            'Axis 0 = Left / Right Turn'
        )

        self.get_logger().info(
            f'Dead zone = {self.DEADZONE * 100:.0f}%'
        )

        self.get_logger().info(
            'Initial state: DISARMED'
        )

        self.get_logger().info(
            '=========================================='
        )

    # ================================================================
    # READ JOYSTICK
    # ================================================================

    def read_joystick(self):

        # ------------------------------------------------------------
        # Read all currently available joystick events.
        # ------------------------------------------------------------

        while True:

            try:

                readable, _, _ = select.select(
                    [self.js_fd],
                    [],
                    [],
                    0
                )

                if not readable:
                    break

                event_data = os.read(
                    self.js_fd,
                    EVENT_SIZE
                )

                if len(event_data) != EVENT_SIZE:
                    break

                _, value, event_type, number = struct.unpack(
                    EVENT_FORMAT,
                    event_data
                )

                # ----------------------------------------------------
                # Remove initialization flag
                # ----------------------------------------------------

                clean_type = event_type & 0x7F

                # ====================================================
                # BUTTON EVENT
                # ====================================================

                if clean_type == 1:

                    self.buttons[number] = value

                    self.last_event_time = time.monotonic()

                    # Print button state
                    if value == 1:

                        self.get_logger().info(
                            f'Button {number}: ON'
                        )

                    else:

                        self.get_logger().info(
                            f'Button {number}: OFF'
                        )

                    # Process ARM / STOP
                    self.process_buttons()

                # ====================================================
                # AXIS EVENT
                # ====================================================

                elif clean_type == 2:

                    self.axes[number] = value

                    self.last_event_time = time.monotonic()

                    # Process D-pad
                    if number == self.DPAD_X_AXIS or \
                       number == self.DPAD_Y_AXIS:

                        self.process_dpad()

                    # Process movement
                    #
                    # We only need to update command when:
                    #
                    # Axis 0 = turning
                    # Axis 4 = movement
                    #
                    if number == self.TURN_AXIS or \
                       number == self.MOVE_AXIS:

                        self.publish_movement()

            except BlockingIOError:

                break

            except OSError as e:

                self.get_logger().error(
                    f'Joystick read error: {e}'
                )

                break

    # ================================================================
    # BUTTON PROCESSING
    # ================================================================

    def process_buttons(self):

        # ------------------------------------------------------------
        # Read current states.
        #
        # IMPORTANT:
        #
        # Button 6 ON means:
        #     Select pressed
        #
        # Button 7 ON means:
        #     Start pressed
        #
        # Button 2 ON means:
        #     X pressed
        # ------------------------------------------------------------

        select = (
            self.buttons.get(
                self.SELECT_BUTTON,
                0
            ) == 1
        )

        start = (
            self.buttons.get(
                self.START_BUTTON,
                0
            ) == 1
        )

        x_button = (
            self.buttons.get(
                self.X_BUTTON,
                0
            ) == 1
        )

        # ============================================================
        # ARM
        #
        # SELECT + START
        #
        # Button 6 ON
        # AND
        # Button 7 ON
        # ============================================================

        if select and start:

            if not self.armed:

                self.armed = True

                # Clear any old movement
                self.stop_robot()

                self.get_logger().warn(
                    '=========================================='
                )

                self.get_logger().warn(
                    '             ROBOT ARMED'
                )

                self.get_logger().warn(
                    '=========================================='
                )

        # ============================================================
        # STOP / DISARM
        #
        # SELECT + X
        #
        # Button 6 ON
        # AND
        # Button 2 ON
        # ============================================================

        if select and x_button:

            if self.armed:

                self.armed = False

                self.stop_robot()

                self.get_logger().error(
                    '=========================================='
                )

                self.get_logger().error(
                    '        ROBOT STOPPED / DISARMED'
                )

                self.get_logger().error(
                    '=========================================='
                )

    # ================================================================
    # DEAD ZONE
    # ================================================================

    def apply_deadzone(self, value):

        # ------------------------------------------------------------
        # Convert raw joystick value to -1 ... +1
        # ------------------------------------------------------------

        normalized = value / 32767.0

        # ------------------------------------------------------------
        # Dead zone
        # ------------------------------------------------------------

        if abs(normalized) < self.DEADZONE:

            return 0.0

        # ------------------------------------------------------------
        # Rescale remaining range.
        # ------------------------------------------------------------

        sign = 1.0 if normalized > 0 else -1.0

        magnitude = abs(normalized)

        magnitude = (
            (magnitude - self.DEADZONE)
            / (1.0 - self.DEADZONE)
        )

        return sign * magnitude

    # ================================================================
    # MOVEMENT
    # ================================================================

    def publish_movement(self):

        # ------------------------------------------------------------
        # If not armed, do nothing.
        # ------------------------------------------------------------

        if not self.armed:

            return

        # ------------------------------------------------------------
        # Get raw joystick values
        # ------------------------------------------------------------

        turn_raw = self.axes.get(
            self.TURN_AXIS,
            0
        )

        move_raw = self.axes.get(
            self.MOVE_AXIS,
            0
        )

        # ------------------------------------------------------------
        # Convert to -1 ... +1 and apply dead zone
        # ------------------------------------------------------------

        turn = self.apply_deadzone(
            turn_raw
        )

        move = self.apply_deadzone(
            move_raw
        )

        # ------------------------------------------------------------
        # RIGHT JOYSTICK
        #
        # Raw:
        #
        # UP   = -32767
        # DOWN = +32767
        #
        # We want:
        #
        # UP   = Forward
        # DOWN = Backward
        #
        # Therefore invert.
        # ------------------------------------------------------------

        move_command = -move

        # ============================================================
        # CREATE COMMAND
        # ============================================================

        cmd = Twist()

        # ============================================================
        # FORWARD / BACKWARD HAS PRIORITY
        # ============================================================

        if abs(move_command) > 0.0:

            cmd.linear.x = (
                move_command *
                self.linear_speed
            )

            cmd.angular.z = 0.0

        # ============================================================
        # TURN
        # ============================================================

        elif abs(turn) > 0.0:

            cmd.linear.x = 0.0

            cmd.angular.z = -(
                turn *
                self.angular_speed
            )

        # ============================================================
        # JOYSTICKS CENTERED
        # ============================================================

        else:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # ============================================================
        # PUBLISH
        # ============================================================

        self.cmd_pub.publish(cmd)

    # ================================================================
    # D-PAD
    # ================================================================

    def process_dpad(self):

        # ------------------------------------------------------------
        # D-pad raw values
        # ------------------------------------------------------------

        dpad_x = self.axes.get(
            self.DPAD_X_AXIS,
            0
        )

        dpad_y = self.axes.get(
            self.DPAD_Y_AXIS,
            0
        )

        # Normalize
        dpad_x = dpad_x / 32767.0
        dpad_y = dpad_y / 32767.0

        # ============================================================
        # UP
        #
        # CH7 = -32767
        #
        # Increase linear speed
        # ============================================================

        dpad_up = dpad_y < -0.5

        if dpad_up and not self.prev_dpad_up:

            self.linear_speed += self.LINEAR_STEP

            self.linear_speed = min(
                self.linear_speed,
                self.MAX_LINEAR_SPEED
            )

            self.get_logger().info(
                f'Linear speed = '
                f'{self.linear_speed:.2f} m/s'
            )

        # ============================================================
        # DOWN
        #
        # CH7 = +32767
        #
        # Decrease linear speed
        # ============================================================

        dpad_down = dpad_y > 0.5

        if dpad_down and not self.prev_dpad_down:

            self.linear_speed -= self.LINEAR_STEP

            self.linear_speed = max(
                self.linear_speed,
                self.MIN_LINEAR_SPEED
            )

            self.get_logger().info(
                f'Linear speed = '
                f'{self.linear_speed:.2f} m/s'
            )

        # ============================================================
        # RIGHT
        #
        # CH6 = +32767
        #
        # Increase turning speed
        # ============================================================

        dpad_right = dpad_x > 0.5

        if dpad_right and not self.prev_dpad_right:

            self.angular_speed += self.ANGULAR_STEP

            self.angular_speed = min(
                self.angular_speed,
                self.MAX_ANGULAR_SPEED
            )

            self.get_logger().info(
                f'Angular speed = '
                f'{self.angular_speed:.2f} rad/s'
            )

        # ============================================================
        # LEFT
        #
        # CH6 = -32767
        #
        # Decrease turning speed
        # ============================================================

        dpad_left = dpad_x < -0.5

        if dpad_left and not self.prev_dpad_left:

            self.angular_speed -= self.ANGULAR_STEP

            self.angular_speed = max(
                self.angular_speed,
                self.MIN_ANGULAR_SPEED
            )

            self.get_logger().info(
                f'Angular speed = '
                f'{self.angular_speed:.2f} rad/s'
            )

        # ------------------------------------------------------------
        # Save D-pad states
        # ------------------------------------------------------------

        self.prev_dpad_up = dpad_up
        self.prev_dpad_down = dpad_down
        self.prev_dpad_left = dpad_left
        self.prev_dpad_right = dpad_right

    # ================================================================
    # STOP
    # ================================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # ================================================================
    # WATCHDOG
    # ================================================================

    def watchdog(self):

        elapsed = (
            time.monotonic()
            - self.last_event_time
        )

        # ------------------------------------------------------------
        # No controller event for too long.
        #
        # Stop and disarm.
        # ------------------------------------------------------------

        if elapsed > self.JOY_TIMEOUT:

            if self.armed:

                self.armed = False

                self.stop_robot()

                self.get_logger().error(
                    '=========================================='
                )

                self.get_logger().error(
                    '       JOYSTICK TIMEOUT'
                )

                self.get_logger().error(
                    '       ROBOT STOPPED / DISARMED'
                )

                self.get_logger().error(
                    '=========================================='

                )

    # ================================================================
    # SHUTDOWN
    # ================================================================

    def destroy_node(self):

        # Always stop robot
        self.stop_robot()

        # Close joystick
        try:

            os.close(self.js_fd)

        except Exception:
            pass

        self.get_logger().warn(
            'EvoFox teleop stopped - robot stopped'
        )

        super().destroy_node()


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = EvoFoxTeleop()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
