#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
import serial
import time
import math

def quaternion_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class ArduinoBridgeNode(Node):

    def __init__(self):
        super().__init__('arduino_bridge_node')
        # --- Parameters ---
        self.declare_parameter('port', '/dev/arduino_motor')
        self.declare_parameter('telemetry_port', '/dev/arduino_telemetry')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('linear_threshold', 0.05)
        self.declare_parameter('angular_threshold', 0.02)
        self.declare_parameter('pulses_per_revolution', 1710.0)
        self.declare_parameter('wheel_radius', 0.165)
        self.declare_parameter('imu_topic', '/imu/data_raw')

        self.port_name = self.get_parameter('port').get_parameter_value().string_value
        self.telemetry_port_name = self.get_parameter('telemetry_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.lin_thresh = self.get_parameter('linear_threshold').get_parameter_value().double_value
        self.ang_thresh = self.get_parameter('angular_threshold').get_parameter_value().double_value
        self.ppr = self.get_parameter('pulses_per_revolution').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        # --- Motor Controller Serial Connection ---
        try:
            self.ser = serial.Serial()
            self.ser.port = self.port_name
            self.ser.baudrate = self.baudrate
            self.ser.timeout = 0.1
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
            
            # Flush old buffer data & stop motors on init
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.send_serial_cmd('S')
            self.get_logger().info(f"Connected to Motor Arduino on {self.port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open motor serial port {self.port_name}: {e}")
            raise e

        # --- Telemetry Arduino Serial Connection ---
        try:
            self.ser_telemetry = serial.Serial()
            self.ser_telemetry.port = self.telemetry_port_name
            self.ser_telemetry.baudrate = self.baudrate
            self.ser_telemetry.timeout = 0.1
            self.ser_telemetry.dtr = False
            self.ser_telemetry.rts = False
            self.ser_telemetry.open()
            
            # Flush old buffer data
            self.ser_telemetry.reset_input_buffer()
            self.ser_telemetry.reset_output_buffer()
            self.get_logger().info(f"Connected Telemetry Arduino on {self.telemetry_port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open telemetry serial port {self.telemetry_port_name}: {e}")
            raise e

        time.sleep(2.0)

        # --- ROS 2 Subscriptions & Publishers ---
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.wheel_odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)

        # --- Internal State Trackers ---
        self.last_cmd = 'S'
        self.arduino_state = 'S'
        self.last_dac = 0
        self.last_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0

        self.prev_left_rad = None
        self.prev_right_rad = None

        self.imu_yaw = 0.0
        self.imu_received = False

        # Timer to read telemetry asynchronously (20Hz)
        self.create_timer(0.05, self.read_telemetry_loop)
        
        # Timer to check Motor Arduino state & verify sync (20Hz)
        self.create_timer(0.05, self.check_motor_state)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.imu_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.imu_received = True

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        V_MIN = 0.20
        V_MAX = 0.40
        DAC_BASE = 500
        DAC_MAX = 2500

        TURN_SCALE = 1.0
        DAC_TURN_MAX = 800  
        DAC_TURN_BASE = 400

        req_speed = max(abs(linear_x), abs(angular_z))

        if linear_x > self.lin_thresh:
            target_cmd = 'F'
        elif linear_x < -self.lin_thresh:
            target_cmd = 'B'
        elif angular_z > self.ang_thresh:
            target_cmd = 'A'
        elif angular_z < -self.ang_thresh:
            target_cmd = 'C'
        else:
            target_cmd = 'S'

        if target_cmd != 'S':
            is_turning = target_cmd in ['A', 'C']
            if is_turning:
                req_speed = abs(angular_z)
                clamped_speed = max(V_MIN, min(req_speed, V_MAX))
                scale = (clamped_speed - V_MIN) / (V_MAX - V_MIN)
                turn_dac_range = (DAC_MAX - DAC_TURN_BASE) * TURN_SCALE
                target_dac = int(DAC_TURN_BASE + scale * turn_dac_range)
            else:
                req_speed = abs(linear_x)
                clamped_speed = max(V_MIN, min(req_speed, V_MAX))
                scale = (clamped_speed - V_MIN) / (V_MAX - V_MIN)
                target_dac = int(DAC_BASE + scale * (DAC_MAX - DAC_BASE))
        else:
            target_dac = 0

        if target_cmd != self.last_cmd:
            self.send_serial_cmd(target_cmd)
            self.last_cmd = target_cmd
            if target_cmd == 'S':
                self.last_dac = 0

        if target_cmd != 'S' and self.last_dac != target_dac:
            self.send_serial_cmd(f"{target_dac}")
            self.last_dac = target_dac

    def send_serial_cmd(self, cmd_str: str):
        if hasattr(self, 'ser') and self.ser.is_open:
            packet = f"{cmd_str}\n".encode('utf-8')
            self.ser.write(packet)
            self.get_logger().info(f"Sent to Motor Arduino: {cmd_str}")

    def check_motor_state(self):
        """Asynchronously reads reported state from Motor Arduino and re-transmits if mismatched."""
        if not hasattr(self, 'ser') or not self.ser.is_open:
            return

        try:
            # Read state changes sent from Arduino
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line in ['F', 'B', 'A', 'C', 'S', 'E']:
                    self.arduino_state = line
                    self.get_logger().info(f"Arduino reported state: {self.arduino_state}")

            # Verify Arduino state matches Jetson expected command
            if self.arduino_state == 'E':
                if self.last_cmd != 'S':
                    self.get_logger().warn("Arduino reported EMERGENCY STOP! Setting target state to 'S'.")
                    self.last_cmd = 'S'
                    self.last_dac = 0
            elif self.arduino_state != self.last_cmd:
                self.get_logger().warn(
                    f"State mismatch! Expected: '{self.last_cmd}', Arduino is: '{self.arduino_state}'. Re-sending command..."
                )
                self.send_serial_cmd(self.last_cmd)
                if self.last_cmd != 'S' and self.last_dac > 0:
                    self.send_serial_cmd(f"{self.last_dac}")

        except Exception as e:
            self.get_logger().warn(f"Motor state check error: {e}", throttle_duration_sec=2.0)

    def read_telemetry_loop(self):
        """Reads 'pulses_m1,pulses_m2' lines from Telemetry Arduino, discards stale lines, and publishes ROS topics."""
        if not hasattr(self, 'ser_telemetry') or not self.ser_telemetry.is_open:
            return

        try:
            latest_line = None

            # DRAIN BUFFER: Read all unread lines and keep ONLY the latest line
            while self.ser_telemetry.in_waiting > 0:
                raw_line = self.ser_telemetry.readline()
                if raw_line:
                    latest_line = raw_line

            if not latest_line:
                return

            line = latest_line.decode('utf-8', errors='ignore').strip()
            if not line or ',' not in line:
                return

            parts = line.split(',')
            if len(parts) != 2:
                return

            pulses_m1 = float(parts[0])
            pulses_m2 = float(parts[1])

            left_rotations = pulses_m2 / self.ppr
            right_rotations = pulses_m1 / self.ppr

            left_rad = left_rotations * (2.0 * math.pi)
            right_rad = right_rotations * (2.0 * math.pi)

            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9

            # First telemetry frame: establish baseline position
            if self.prev_left_rad is None:
                self.prev_left_rad = left_rad
                self.prev_right_rad = right_rad
                self.last_time = current_time
                
                # Publish initial zeroed state
                self.publish_topics(current_time, left_rad, right_rad, 0.0, 0.0, 0.0)
                return

            delta_left = left_rad - self.prev_left_rad
            delta_right = right_rad - self.prev_right_rad
            self.prev_left_rad = left_rad
            self.prev_right_rad = right_rad

            ds = self.wheel_radius * (delta_left + delta_right) / 2.0
            heading = self.imu_yaw + (math.pi/2)

            self.x += ds * math.cos(heading)
            self.y += ds * math.sin(heading)

            if dt > 0.001:
                rad_s_left = delta_left / dt
                rad_s_right = delta_right / dt
                v_x = ds / dt
            else:
                rad_s_left = 0.0
                rad_s_right = 0.0
                v_x = 0.0

            self.last_time = current_time

            # Publish data
            self.publish_topics(current_time, left_rad, right_rad, rad_s_left, rad_s_right, v_x)

        except Exception as e:
            self.get_logger().warn(f"Telemetry loop error: {e}", throttle_duration_sec=2.0)

    def publish_topics(self, current_time, left_rad, right_rad, rad_s_left, rad_s_right, v_x):
        """Helper function to publish /joint_states and /wheel/odom."""
        # 1. JointState
        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_msg.position = [left_rad, right_rad]
        joint_msg.velocity = [rad_s_left, rad_s_right]
        self.joint_state_pub.publish(joint_msg)

        # 2. Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        qx, qy, qz, qw = yaw_to_quaternion(self.imu_yaw)
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = v_x

        odom_msg.pose.covariance[0] = 0.02
        odom_msg.pose.covariance[7] = 0.02
        odom_msg.pose.covariance[35] = 0.05
        odom_msg.twist.covariance[0] = 0.02

        self.wheel_odom_pub.publish(odom_msg)

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.send_serial_cmd('S')
            self.ser.reset_output_buffer()
            self.ser.close()
        if hasattr(self, 'ser_telemetry') and self.ser_telemetry.is_open:
            self.ser_telemetry.reset_input_buffer()
            self.ser_telemetry.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()