import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from ultralytics import YOLO
import serial

class YoloDepthArduinoNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_arduino_node')
        self.bridge = CvBridge()

        # --- Parameters ---
        self.declare_parameter('port', '/dev/arduino_motor')
        self.declare_parameter('baudrate', 115200)
        
        self.port_name = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # --- Serial Connection Setup ---
        self.ser = None
        try:
            self.ser = serial.Serial()
            self.ser.port = self.port_name
            self.ser.baudrate = self.baudrate
            self.ser.timeout = 0.1
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"Connected to Arduino on {self.port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.port_name}: {e}")

        # --- State Tracker for Commands ---
        self.last_person_cmd = None  # Tracks 'T' or 'Z' state

        # --- Load YOLOv8 on CUDA GPU ---
        self.model = YOLO('yolov8n.pt').to('cuda')

        # --- ROS 2 Subscribers ---
        self.color_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.image_callback)

        # --- ROS 2 Publishers ---
        self.raw_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)
        self.compressed_pub = self.create_publisher(CompressedImage, '/yolo/annotated_image/compressed', 10)

        self.get_logger().info("YOLO Depth Arduino Node initialized.")

    def send_serial_cmd(self, cmd_str: str):
        """Sends single-character command string over serial to Arduino."""
        if self.ser and self.ser.is_open:
            packet = f"{cmd_str}\n".encode('utf-8')
            self.ser.write(packet)
            self.get_logger().info(f"Sent to Arduino: {cmd_str}")

    def image_callback(self, color_msg, depth_msg):
        color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # Run inference filtering specifically for class 0 (person)
        results = self.model(color_image, device='cuda', classes=[0], verbose=False)

        person_near = False  # Flag to track if any person is closer than 5.0m

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]

                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2

                if 0 <= center_y < depth_image.shape[0] and 0 <= center_x < depth_image.shape[1]:
                    distance_m = depth_image[center_y, center_x] / 1000.0
                else:
                    distance_m = 0.0

                # Check 5-meter threshold condition
                if 0.0 <= distance_m < 2.0:
                    person_near = True

                # Draw bounding box and label
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(color_image, (center_x, center_y), 4, (0, 0, 255), -1)
                text = f"{label} {conf:.2f} | {distance_m:.2f}m"
                cv2.putText(color_image, text, (x1, max(y1 - 10, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # --- Command Trigger Logic ---
        target_cmd = 'T' if person_near else 'Z'

        # Send command ONLY when state changes to avoid repetitive serial writes
        if target_cmd != self.last_person_cmd:
            self.send_serial_cmd(target_cmd)
            self.last_person_cmd = target_cmd

        # --- Publish Video Streams ---
        raw_msg = Image()
        raw_msg.header = color_msg.header
        raw_msg.height, raw_msg.width, _ = color_image.shape
        raw_msg.encoding = 'bgr8'
        raw_msg.is_bigendian = 0
        raw_msg.step = color_image.strides[0]
        raw_msg.data = color_image.tobytes()
        self.raw_pub.publish(raw_msg)

        compressed_msg = CompressedImage()
        compressed_msg.header = color_msg.header
        compressed_msg.format = "jpeg"
        success, encoded = cv2.imencode('.jpg', color_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if success:
            compressed_msg.data = encoded.tobytes()
            self.compressed_pub.publish(compressed_msg)

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthArduinoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
