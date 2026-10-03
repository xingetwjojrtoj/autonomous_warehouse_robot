#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
import serial
import time

# --- CONFIGURATION ---
CONTROL_PORT = '/dev/ttyUSB4'  # 4  AT command port
DATA_PORT = '/dev/ttyUSB3'     # 3GPS data port
BAUD_RATE = 115200
# ---------------------

class GpsGnssDriverNode(Node):
    def __init__(self):
        super().__init__('gps_gnss_driver_node')
        
        # Create a ROS2 Publisher for the GPS Fix data
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # 1. Initialize the GPS hardware via AT command
        self.get_logger().info("Initializing GNSS hardware within the modem...")
        self.send_at_command(CONTROL_PORT, "AT+CGPS=1")
        time.sleep(2)
        
        # 2. Open the serial port connection for streaming data
        try:
            self.serial_conn = serial.Serial(DATA_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f"Connected to data port {DATA_PORT}. Processing NMEA stream...")
        except Exception as e:
            self.get_logger().error(f"Failed to open data port {DATA_PORT}: {e}")
            raise e

        # 3. Create a timer loop that triggers every 0.1s (~10Hz) to read incoming strings
        self.timer = self.create_timer(0.1, self.read_gps_data)

    def send_at_command(self, port, command, delay=1):
        try:
            with serial.Serial(port, BAUD_RATE, timeout=2) as ser:
                ser.write((command + '\r\n').encode('utf-8'))
                time.sleep(delay)
                resp = ser.read_all().decode('utf-8', errors='ignore').strip()
                self.get_logger().info(f"Modem Response: {resp}")
        except Exception as e:
            self.get_logger().warn(f"Failed to send AT sequence: {e}")

    def convert_to_decimal(self, nmea_val, direction):
        """Converts NMEA DDMM.MMMM format into Standard Decimal Degrees (DD) for ROS2."""
        if not nmea_val or direction not in ['N', 'S', 'E', 'W']:
            return 0.0
        try:
            if direction in ['N', 'S']:
                degrees = int(nmea_val[:2])
                minutes = float(nmea_val[2:])
            else:
                degrees = int(nmea_val[:3])
                minutes = float(nmea_val[3:])
            
            decimal = degrees + (minutes / 60.0)
            if direction in ['S', 'W']:
                decimal = -decimal
            return decimal
        except ValueError:
            return 0.0

    def convert_to_dms(self, nmea_val, direction):
        """Converts NMEA DDMM.MMMM format into visual Degrees Minutes Seconds string."""
        if not nmea_val or direction not in ['N', 'S', 'E', 'W']:
            return ""
        try:
            if direction in ['N', 'S']:
                degrees = int(nmea_val[:2])
                minutes_raw = float(nmea_val[2:])
            else:
                degrees = int(nmea_val[:3])
                minutes_raw = float(nmea_val[3:])
            minutes = int(minutes_raw)
            seconds = (minutes_raw - minutes) * 60
            return f"{degrees}°{minutes}'{seconds:.2f}\"{direction}"
        except ValueError:
            return ""

    def read_gps_data(self):
        if not self.serial_conn.is_open:
            return

        # Read waiting lines from serial buffer
        while self.serial_conn.in_waiting > 0:
            try:
                line = self.serial_conn.readline().decode('ascii', errors='ignore').strip()
                parts = line.split(',')
                
                # Check for active multi-constellation network string
                if line.startswith('$GNGNS') and len(parts) > 7:
                    raw_lat, lat_dir, raw_lon, lon_dir = parts[2], parts[3], parts[4], parts[5]
                    sat_count = parts[7]
                    
                    # Log visual DMS locally in terminal
                    lat_dms = self.convert_to_dms(raw_lat, lat_dir)
                    lon_dms = self.convert_to_dms(raw_lon, lon_dir)
                    if lat_dms and lon_dms:
                        self.get_logger().info(f"Sats: {sat_count} | Local Display: {lat_dms}, {lon_dms}")

                    # Generate ROS2 Decimal Coordinates
                    dec_lat = self.convert_to_decimal(raw_lat, lat_dir)
                    dec_lon = self.convert_to_decimal(raw_lon, lon_dir)

                    if dec_lat != 0.0 and dec_lon != 0.0:
                        # Construct standard ROS2 Message
                        fix_msg = NavSatFix()
                        fix_msg.header = Header()
                        fix_msg.header.stamp = self.get_clock().now().to_msg()
                        fix_msg.header.frame_id = "gps_link"
                        
                        fix_msg.latitude = dec_lat
                        fix_msg.longitude = dec_lon
                        # If altitude info exists in your NMEA stream, it falls into index 9
                        fix_msg.altitude = float(parts[9]) if (len(parts) > 9 and parts[9]) else 0.0
                        
                        # Publish out to ROS2 ecosystem
                        self.publisher_.publish(fix_msg)

            except Exception as e:
                self.get_logger().error(f"Error parsing line: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = GpsGnssDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
