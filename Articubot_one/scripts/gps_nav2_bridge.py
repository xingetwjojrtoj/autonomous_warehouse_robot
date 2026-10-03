#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint
from robot_localization.srv import FromLL

class GPSNav2Bridge(Node):
    def __init__(self):
        super().__init__('gps_nav2_bridge')

        # Create client for navsat_transform's built-in /fromLL service
        self.from_ll_client = self.create_client(FromLL, '/fromLL')
        
        while not self.from_ll_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /fromLL service from navsat_transform_node...')

        # Subscriber for Foxglove GPS targets
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/target_gps_waypoint',
            self.gps_callback,
            10
        )

        # Publisher to Nav2
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.get_logger().info('GPS-to-Nav2 Bridge is ready and connected to navsat_transform!')

    def gps_callback(self, msg: NavSatFix):
        req = FromLL.Request()
        req.ll_point = GeoPoint()
        req.ll_point.latitude = msg.latitude
        req.ll_point.longitude = msg.longitude
        req.ll_point.altitude = msg.altitude

        # Asynchronously call the /fromLL service
        future = self.from_ll_client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            
            # Create Nav2 PoseStamped Goal
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position = response.map_point
            goal_msg.pose.orientation.w = 1.0  # Default forward orientation

            self.goal_pub.publish(goal_msg)
            self.get_logger().info(
                f'Transformed GPS to MapPose: x={goal_msg.pose.position.x:.2f}m, y={goal_msg.pose.position.y:.2f}m -> Sent to /goal_pose'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to convert GPS point: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GPSNav2Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()