import asyncio
import json
import math
import os
import signal
import subprocess
import threading
import time

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, FileResponse, StreamingResponse

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, Imu, CompressedImage
from nav_msgs.msg import Odometry


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_NAME = "articubot_one"

# Nav2 Action Imports
try:
    from nav2_msgs.action import NavigateToPose, FollowWaypoints
    NAV2_ACTION_AVAILABLE = True
except ImportError:
    NAV2_ACTION_AVAILABLE = False

# Global Process Tracker for script tasks (1 to 6)
script_processes = {1: None, 2: None, 3: None, 4: None, 5: None, 6: None}

# Global Centralized State Store
system_state = {
    "script_status": {
        "1": "STOPPED",
        "2": "STOPPED",
        "3": "STOPPED",
        "4": "STOPPED",
        "5": "STOPPED",
        "6": "STOPPED"
    },
    "linear_speed": 0.20,
    "angular_speed": 0.20,
    "mission_status": "IDLE",
    "bluetooth_connected": False,
    "selected_map": "",
    "available_maps": []
}

# Reference Origin for Mini-Map GPS Projection
REF_LAT = 22.5280
REF_LNG = 75.9230

# Predefined Waypoints using Local Map Frame Coordinates
PREDEFINED_POINTS = {
    "Point A": {"x": -0.795903, "y": 4.65571, "z": 0.703635, "w": 0.710562},
    "Point B": {"x": 2.78942, "y": 10.76100, "z": -0.0152111, "w": 0.999884},
    "Point C": {"x": 6.25337, "y": 5.71582, "z": -0.998535, "w": 0.0541161},
}


def update_available_maps():
    """Scans user's home directory for available .yaml map files."""
    home_dir = os.path.expanduser("~")
    try:
        yaml_files = [f for f in os.listdir(home_dir) if f.endswith('.yaml')]
        yaml_files.sort()
        system_state["available_maps"] = yaml_files
        if yaml_files and (not system_state["selected_map"] or system_state["selected_map"] not in yaml_files):
            system_state["selected_map"] = yaml_files[0]
    except Exception as e:
        print(f"Error scanning available maps: {e}")


class ROS2GCSBridge(Node):
    def __init__(self, ws_loop, active_connections):
        super().__init__('anveshan_gcs_bridge')
        self.ws_loop = ws_loop
        self.active_connections = active_connections
        self.latest_image_bytes = None

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        # Nav2 Action Clients
        self.waypoint_goal_handle = None
        if NAV2_ACTION_AVAILABLE:
            self.nav_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
            self.follow_waypoints_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        else:
            self.nav_action_client = None
            self.follow_waypoints_client = None

        self.FRESH_TIMEOUT = 1.5
        self.position_last_seen = {}
        self.heading_last_seen = {}

        self.POSITION_PRIORITY = {
            'gps_filtered': 0,
            'gps_fix': 1,
            'odometry_global': 2,
            'odom': 3,
            'odometry_local': 4,
        }
        self.HEADING_PRIORITY = {
            'heading_topic': 0,
            'imu_data': 1,
            'odometry_global': 2,
            'odom': 3,
            'odometry_local': 4,
            'imu_data_raw': 5,
        }

        # Subscriptions
        self.create_subscription(
            CompressedImage,
            '/yolo/annotated_image/compressed',
            self.image_callback,
            qos_profile_sensor_data
        )

        # GPS Subscriptions
        self.create_subscription(
            NavSatFix, '/gps/filtered',
            lambda msg: self.gps_callback(msg, 'gps_filtered'), qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, '/gps/fix',
            lambda msg: self.gps_callback(msg, 'gps_fix'), qos_profile_sensor_data)

        # Odometry Subscriptions
        self.create_subscription(
            Odometry, '/odometry/global',
            lambda msg: self.odom_callback(msg, 'odometry_global'), qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/odom',
            lambda msg: self.odom_callback(msg, 'odom'), qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/odometry_local',
            lambda msg: self.odom_callback(msg, 'odometry_local'), qos_profile_sensor_data)

        # IMU Subscriptions
        self.create_subscription(
            Imu, '/imu/data',
            lambda msg: self.imu_callback(msg, 'imu_data'), qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/imu/data_raw',
            lambda msg: self.imu_callback(msg, 'imu_data_raw'), qos_profile_sensor_data)
        self.create_subscription(
            Float32, '/heading',
            lambda msg: self.heading_callback(msg, 'heading_topic'), qos_profile_sensor_data)

        self.get_logger().info("ROS 2 GCS Bridge initialized.")

    def image_callback(self, msg: CompressedImage):
        self.latest_image_bytes = bytes(msg.data)

    def _is_best_position_source(self, source: str) -> bool:
        now = time.monotonic()
        self.position_last_seen[source] = now
        my_priority = self.POSITION_PRIORITY.get(source, 99)
        for other_source, other_priority in self.POSITION_PRIORITY.items():
            if other_priority < my_priority:
                last_seen = self.position_last_seen.get(other_source)
                if last_seen is not None and (now - last_seen) < self.FRESH_TIMEOUT:
                    return False
        return True

    def _is_best_heading_source(self, source: str) -> bool:
        now = time.monotonic()
        self.heading_last_seen[source] = now
        my_priority = self.HEADING_PRIORITY.get(source, 99)
        for other_source, other_priority in self.HEADING_PRIORITY.items():
            if other_priority < my_priority:
                last_seen = self.heading_last_seen.get(other_source)
                if last_seen is not None and (now - last_seen) < self.FRESH_TIMEOUT:
                    return False
        return True

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def send_waypoint_mission(self, waypoints):
        if not waypoints or not (NAV2_ACTION_AVAILABLE and self.follow_waypoints_client):
            update_mission_status("FAILED")
            return

        if not self.follow_waypoints_client.wait_for_server(timeout_sec=2.0):
            update_mission_status("FAILED")
            return

        poses = []
        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(wp["x"])
            pose.pose.position.y = float(wp["y"])
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = float(wp.get("z", 0.0))
            pose.pose.orientation.w = float(wp.get("w", 1.0))
            poses.append(pose)

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        update_mission_status("NAVIGATING...")
        send_future = self.follow_waypoints_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._waypoint_goal_response_cb)

    def _waypoint_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            update_mission_status("REJECTED")
            return
        self.waypoint_goal_handle = goal_handle
        update_mission_status("ACCEPTED")
        goal_handle.get_result_async().add_done_callback(self._waypoint_result_cb)

    def _waypoint_result_cb(self, future):
        self.waypoint_goal_handle = None
        update_mission_status("FINISHED")

    def cancel_waypoint_mission(self):
        if self.waypoint_goal_handle is not None:
            self.waypoint_goal_handle.cancel_goal_async()
            self.waypoint_goal_handle = None
        else:
            subprocess.Popen("ros2 action cancel_goal /follow_waypoints", shell=True, executable="/bin/bash")
        self.publish_cmd_vel(0.0, 0.0)

    def cancel_nav2_mission(self):
        subprocess.Popen("ros2 action cancel_goal /navigate_to_pose", shell=True, executable="/bin/bash")
        self.cancel_waypoint_mission()
        self.publish_cmd_vel(0.0, 0.0)
        update_mission_status("STOPPED")

    def gps_callback(self, msg: NavSatFix, source: str):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude) or (msg.latitude == 0.0 and msg.longitude == 0.0):
            return
        if not self._is_best_position_source(source):
            return
        self.broadcast_websocket(json.dumps({"type": "gps_update", "lat": msg.latitude, "lng": msg.longitude}))

    def odom_callback(self, msg: Odometry, source: str):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._is_best_position_source(source):
            calc_lat = REF_LAT + (y / 111111.0)
            calc_lng = REF_LNG + (x / (111111.0 * math.cos(math.radians(REF_LAT))))
            self.broadcast_websocket(json.dumps({"type": "gps_update", "lat": calc_lat, "lng": calc_lng}))
        self.process_quaternion_heading(msg.pose.pose.orientation, source)

    def imu_callback(self, msg: Imu, source: str):
        self.process_quaternion_heading(msg.orientation, source)

    def process_quaternion_heading(self, q, source: str):
        if not self._is_best_heading_source(source):
            return
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        compass_heading = (180 + 180 - yaw_deg) % 360.0
        self.broadcast_websocket(json.dumps({"type": "heading_update", "heading": round(compass_heading, 1)}))

    def heading_callback(self, msg: Float32, source: str):
        if not self._is_best_heading_source(source):
            return
        self.broadcast_websocket(json.dumps({"type": "heading_update", "heading": round(float(msg.data), 1)}))

    def broadcast_websocket(self, payload):
        for ws in list(self.active_connections):
            asyncio.run_coroutine_threadsafe(ws.send_text(payload), self.ws_loop)


app = FastAPI()
active_websockets = set()
ros_node = None
ros_thread = None


def broadcast_system_state():
    if ros_node:
        payload = json.dumps({"type": "state_sync", "state": system_state})
        ros_node.broadcast_websocket(payload)


def update_mission_status(status_str):
    system_state["mission_status"] = status_str
    broadcast_system_state()


def check_bluetooth_controller():
    try:
        input_dir = "/dev/input"
        if os.path.exists(input_dir):
            devices = os.listdir(input_dir)
            if any(d.startswith("js") or d.startswith("event") for d in devices):
                return True
    except Exception:
        pass
    return False


def monitor_background_tasks():
    while True:
        state_changed = False
        for sid, proc in list(script_processes.items()):
            str_sid = str(sid)
            if proc is not None:
                poll_result = proc.poll()
                if poll_result is not None:
                    if poll_result in (0, -2, -15):
                        new_status = "STOPPED"
                    else:
                        new_status = "FAILED"

                    if system_state["script_status"][str_sid] != new_status:
                        system_state["script_status"][str_sid] = new_status
                        state_changed = True
                    script_processes[sid] = None

        bt_status = check_bluetooth_controller()
        if system_state["bluetooth_connected"] != bt_status:
            system_state["bluetooth_connected"] = bt_status
            state_changed = True

        if state_changed:
            broadcast_system_state()

        time.sleep(0.5)


@app.on_event("startup")
async def on_startup():
    global ros_thread
    update_available_maps()
    running_loop = asyncio.get_running_loop()
    ros_thread = threading.Thread(
        target=start_ros2_node,
        args=(running_loop, active_websockets),
        daemon=True
    )
    ros_thread.start()

    monitor_thread = threading.Thread(target=monitor_background_tasks, daemon=True)
    monitor_thread.start()


@app.get("/map_view", response_class=HTMLResponse)
async def get_map_view():
    map_path = os.path.join(BASE_DIR, "map_viewer.html")
    with open(map_path, "r", encoding="utf-8") as f:
        return f.read()


@app.get("/", response_class=HTMLResponse)
async def get_index():
    index_path = os.path.join(BASE_DIR, "templates", "index.html")
    with open(index_path, "r", encoding="utf-8") as f:
        return f.read()


@app.get("/logo.png")
async def get_logo():
    logo_path = os.path.join(BASE_DIR, "logo.png")
    if os.path.exists(logo_path):
        
        return FileResponse(logo_path)
    return HTMLResponse(status_code=404, content="Logo not found")


@app.get("/video_feed")
async def video_feed():
    async def frame_generator():
        while True:
            if ros_node and ros_node.latest_image_bytes:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + ros_node.latest_image_bytes + b'\r\n')
            await asyncio.sleep(0.033)

    return StreamingResponse(frame_generator(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_websockets.add(websocket)

    update_available_maps()
    await websocket.send_text(json.dumps({"type": "state_sync", "state": system_state}))

    try:
        while True:
            data_str = await websocket.receive_text()
            data = json.loads(data_str)
            msg_type = data.get("type")

            if msg_type == "teleop_cmd" and ros_node:
                ros_node.publish_cmd_vel(data.get("linear_x", 0.0), data.get("angular_z", 0.0))

            elif msg_type == "speed_update":
                system_state["linear_speed"] = float(data.get("linear_speed", system_state["linear_speed"]))
                system_state["angular_speed"] = float(data.get("angular_speed", system_state["angular_speed"]))
                broadcast_system_state()

            elif msg_type == "stop_mission" and ros_node:
                ros_node.cancel_nav2_mission()

            elif msg_type == "stop_all_scripts":
                for sid in list(script_processes.keys()):
                    stop_script(sid)
                broadcast_system_state()

            elif msg_type == "estop":
                execute_emergency_stop()

            elif msg_type == "script_control":
                script_id = data.get("script_id")
                action = data.get("action")
                if action == "start":
                    start_script(script_id)
                elif action == "stop":
                    stop_script(script_id)

            elif msg_type == "select_map":
                map_name = data.get("map_name")
                if map_name:
                    system_state["selected_map"] = map_name
                    broadcast_system_state()

            elif msg_type == "save_map":
                filename = data.get("filename", "").strip()
                if filename:
                    home_dir = os.path.expanduser("~")
                    map_save_path = os.path.join(home_dir, filename)
                    cmd = f"source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null; ros2 run nav2_map_server map_saver_cli -f {map_save_path}"
                    
                    def run_map_saver():
                        proc = subprocess.Popen(["/bin/bash", "-c", cmd])
                        proc.wait()
                        time.sleep(1)
                        update_available_maps()
                        broadcast_system_state()

                    threading.Thread(target=run_map_saver, daemon=True).start()

            elif msg_type == "navigate_to_point" and ros_node:
                point_key = data.get("point")
                target = PREDEFINED_POINTS.get(point_key)
                if target:
                    ros_node.send_waypoint_mission([target])

    except WebSocketDisconnect:
        active_websockets.remove(websocket)


def start_script(script_id):
    global script_processes
    stop_script(script_id)
    try:
        pkg_share = get_package_share_directory(PKG_NAME)
    except Exception as e:
        print(f"Error finding ROS package {PKG_NAME}: {e}")
        pkg_share = ""

    # Source ROS environment dynamically
    ros_env = "source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null; [ -f install/setup.bash ] && source install/setup.bash;"

    # home_dir = os.path.expanduser("~")
    # robot_dir = os.path.join(home_dir, "2_wheel_robot")
    # script_dir = os.path.join(home_dir, "2_wheel_robot", "src", "articubot_one", "scripts")

    if script_id == 1:
        cmd = f"cd {ros_env} ros2 launch {PKG_NAME} 2wheel_launch_sim.launch.py"
    elif script_id == 2:
        selected_map = system_state.get("selected_map", "")
        if selected_map:
            map_path = os.path.join(os.path.expanduser("~"), selected_map)
        else:
            map_path = os.path.join(pkg_share, "maps", "my_map.yaml") if pkg_share else "my_map.yaml"
        
        cmd = f"{ros_env} ros2 launch {PKG_NAME} nav2.launch.py map:={map_path}"
    elif script_id == 3:
        cmd = f"{ros_env} (ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true depth_module.profile:=428x240x15 rgb_camera.profile:=428x240x15 &) && sleep 3 && ros2 run {PKG_NAME} yolo_depth_node.py"

    elif script_id == 4:
        cmd = f"{ros_env} ros2 run {PKG_NAME} evofox_teleop.py"

    elif script_id == 5:
        cmd = f"{ros_env} ros2 launch rosbridge_server rosbridge_websocket_launch.xml"

    elif script_id == 6:
        map_param = os.path.join(pkg_share, "config", "mapper_params_online_async.yaml") if pkg_share else "./config/mapper_params_online_async.yaml"
        cmd = f"{ros_env} ros2 launch slam_toolbox online_async_launch.py params_file:={map_param}"

    else:
        return

    try:
        script_processes[script_id] = subprocess.Popen(["/bin/bash", "-c", cmd], preexec_fn=os.setsid)
        system_state["script_status"][str(script_id)] = "RUNNING"
    except Exception as e:
        print(f"Error launching script {script_id}: {e}")
        system_state["script_status"][str(script_id)] = "FAILED"

    broadcast_system_state()


def stop_script(script_id):
    global script_processes
    proc = script_processes.get(script_id)
    if proc and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception as e:
            print(f"Error terminating process {script_id}: {e}")
    script_processes[script_id] = None
    system_state["script_status"][str(script_id)] = "STOPPED"
    broadcast_system_state()


def zero_velocity_burst():
    msg = Twist()
    for _ in range(20):
        if ros_node:
            ros_node.cmd_vel_pub.publish(msg)
            ros_node.cmd_vel_nav_pub.publish(msg)
        time.sleep(0.05)


def execute_emergency_stop():
    if ros_node:
        ros_node.cancel_nav2_mission()
    for sid in list(script_processes.keys()):
        stop_script(sid)
    system_state["mission_status"] = "E-STOPPED"
    broadcast_system_state()
    threading.Thread(target=zero_velocity_burst, daemon=True).start()


def start_ros2_node(loop, active_connections):
    global ros_node
    rclpy.init()
    ros_node = ROS2GCSBridge(loop, active_connections)
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=5000)
