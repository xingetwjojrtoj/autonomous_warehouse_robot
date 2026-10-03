#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import struct
import math

class Bno085RvcNode(Node):
    def __init__(self):
        super().__init__('imu_serial_bridge')
        
        self.declare_parameter('port', '/dev/imu')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        self.get_logger().info(f"Opening serial port: {port} at {baud} baud.")
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {str(e)}")
            return
            
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 15)
        self.timer = self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        # Read byte by byte until we find 0xAA 0xAA alignment frame
        if self.ser.in_waiting >= 19:
            b1 = self.ser.read(1)
            if b1 == b'\xaa':
                b2 = self.ser.read(1)
                if b2 == b'\xaa':
                    # Read the next 17 payload bytes completely
                    packet = self.ser.read(17)
                    if len(packet) == 17:           
                        self.parse_rvc_packet(packet)

    def parse_rvc_packet(self, packet):
        # --- DIAGNOSTIC: Calculate Checksum ---
        calc_sum = (0xAA + 0xAA + sum(packet[0:16])) & 0xFF
        given_sum = packet[16]

        # Unpack indices
        unpacked = struct.unpack('<Bhhhhhh', packet[0:13])

        # Raw values from sensor frame (+Y is forward, +X is right)
        sensor_yaw = -unpacked[1] * 0.01
        sensor_pitch = unpacked[2] * 0.01
        sensor_roll = unpacked[3] * 0.01

        

        sensor_accel_x = unpacked[4] * 0.00980665
        sensor_accel_y = unpacked[5] * 0.00980665
        sensor_accel_z = unpacked[6] * 0.00980665

        # --- REMAP AXES TO ROS REP-103 (+X Forward, +Y Left, +Z Up) ---
        # 1. Acceleration Remapping:
        ros_accel_x = sensor_accel_y  # Sensor forward (+Y) becomes ROS forward (+X)
        ros_accel_y = (
            -sensor_accel_x
        )  # Sensor right (+X) inverted becomes ROS left (+Y)
        ros_accel_z = sensor_accel_z  # Z remains Up

        # 2. Orientation Remapping (Applying a 90-degree Yaw offset in Euler angles):
        # Since sensor Y is forward, offset sensor yaw by +90 degrees (+pi/2 rads)
        yaw = math.radians(sensor_yaw) +(math.pi / 2.0)
        # yaw = math.radians(sensor_yaw) 
        pitch = math.radians(sensor_pitch)
        roll = math.radians(sensor_roll)


        
        # Convert Tait-Bryan angles to clean ROS Quaternions
        q = self.euler_to_quaternion(roll, pitch, yaw)

        # Build ROS2 message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Orientation
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        # Orientation Covariance
        imu_msg.orientation_covariance = [
            0.0025,
            0.0,
            0.0,
            0.0,
            0.0025,
            0.0,
            0.0,
            0.0,
            0.0025,
        ]

        # Linear Acceleration (Using remapped ROS values)
        imu_msg.linear_acceleration.x = ros_accel_x
        imu_msg.linear_acceleration.y = ros_accel_y
        imu_msg.linear_acceleration.z = ros_accel_z

        # Linear Acceleration Covariance
        imu_msg.linear_acceleration_covariance = [
            0.01,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.01,
        ]

        # Angular Velocity: NOT AVAILABLE IN UART-RVC MODE
        imu_msg.angular_velocity_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)



        # Force print raw reading metrics to see exact values
        self.get_logger().info(
            f'RAW Yaw: {sensor_yaw:.2f}° | ROS IMU -> AccelX: {ros_accel_x:.2f}, AccelY: {ros_accel_y:.2f} | ROS Yaw: {math.degrees(yaw):.2f}°',
            throttle_duration_sec=0.5,
            
        )

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        ]

def main(args=None):
    rclpy.init(args=args)
    node = Bno085RvcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
