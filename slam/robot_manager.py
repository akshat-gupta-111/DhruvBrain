#!/usr/bin/env python3
import os
import sys
import json
import time
import threading
from io import BytesIO
from http.server import BaseHTTPRequestHandler, HTTPServer
from PIL import Image, ImageDraw, ImageFont

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Global memory shared between the ROS node and the web server
latest_map_img_bytes = b""   # empty = no map received yet
map_lock = threading.Lock()
map_received_count = 0        # how many /map messages we have processed
scan_received = False         # set True on first /scan echo (via map arrival)


def _make_placeholder_png(width: int = 480, height: int = 240) -> bytes:
    """Generate a grey 'Waiting for map...' placeholder image."""
    img = Image.new('RGB', (width, height), color=(40, 40, 40))
    draw = ImageDraw.Draw(img)
    msg = "Waiting for /map data..."
    sub = "Drive the robot to start SLAM mapping."
    # Draw centred text (no truetype needed)
    draw.text((width // 2, height // 2 - 20), msg, fill=(180, 180, 180), anchor='mm')
    draw.text((width // 2, height // 2 + 10), sub, fill=(120, 120, 120), anchor='mm')
    buf = BytesIO()
    img.save(buf, format='PNG')
    return buf.getvalue()


PLACEHOLDER_PNG = _make_placeholder_png()

class MapWebServer(BaseHTTPRequestHandler):
    """Serve the live map page and PNG on port 8000."""

    # Silence the per-request log lines so they don't flood the terminal
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        global latest_map_img_bytes, map_received_count

        # ── / ── main page ────────────────────────────────────────────────────
        if self.path == '/':
            with map_lock:
                has_map = bool(latest_map_img_bytes)
                count   = map_received_count

            if has_map:
                status_html = f'<p style="color:#4caf50;">&#10004; Live map active &mdash; {count} frames received</p>'
            else:
                status_html = '<p style="color:#ff9800;">&#9899; Waiting for /map data &mdash; drive the robot to start mapping</p>'

            html = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta http-equiv="refresh" content="2">
  <title>DHRUV SLAM &mdash; Live Map</title>
  <style>
    body {{ background:#1a1a2e; color:#eee; text-align:center;
            font-family:'Segoe UI',sans-serif; margin:0; padding:20px; }}
    h1   {{ color:#00b4d8; margin-bottom:4px; }}
    img  {{ border:2px solid #444; border-radius:6px;
            max-width:95%; max-height:78vh; margin-top:12px; }}
    a    {{ color:#90e0ef; text-decoration:none; }}
  </style>
</head>
<body>
  <h1>DHRUV SLAM &mdash; Live Map</h1>
  {status_html}
  <div><img src="/map.png" alt="SLAM map"></div>
  <p style="font-size:0.8em;color:#777;">Auto-refreshing every 2 s &nbsp;|&nbsp;
     <a href="/status">/status (diagnostics)</a></p>
</body>
</html>"""
            self._send(200, 'text/html; charset=utf-8', html.encode())

        # ── /map.png ── actual map image (or placeholder) ─────────────────────
        elif self.path == '/map.png':
            with map_lock:
                img_data = latest_map_img_bytes
            # Always return 200 with either the real map or the placeholder
            self._send(200, 'image/png', img_data if img_data else PLACEHOLDER_PNG)

        # ── /status ── quick JSON diagnostic ──────────────────────────────────
        elif self.path == '/status':
            with map_lock:
                count = map_received_count
            payload = json.dumps({
                'map_frames_received': count,
                'map_live': count > 0,
            }, indent=2)
            self._send(200, 'application/json', payload.encode())

        else:
            self._send(404, 'text/plain', b'Not found')

    def _send(self, code: int, ctype: str, body: bytes):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def run_web_server():
    server = HTTPServer(('0.0.0.0', 8000), MapWebServer)
    server.serve_forever()

class RobotManagerNode(Node):
    def __init__(self):
        super().__init__('robot_manager_node')
        # Map subscriber for generating the live web view image
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # TF Listeners to safely read the robot's current position for checkpoints
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.checkpoints = {}

    def map_callback(self, msg: OccupancyGrid):
        """Converts raw OccupancyGrid data array into an image stream for the Web Server."""
        global latest_map_img_bytes, map_received_count
        width = msg.info.width
        height = msg.info.height
        if width == 0 or height == 0:
            return
        map_received_count += 1

        # Map grid values: -1 = Unknown (Gray), 0 = Free (White), 100 = Occupied (Black)
        img = Image.new('RGB', (width, height))
        pixels = img.load()
        
        for y in range(height):
            for x in range(width):
                val = msg.data[x + y * width]
                if val == 0:
                    pixels[x, height - 1 - y] = (255, 255, 255) # Free
                elif val == 100:
                    pixels[x, height - 1 - y] = (0, 0, 0)       # Wall
                else:
                    pixels[x, height - 1 - y] = (127, 127, 127) # Unknown

        # Prepare to draw on the image
        draw = ImageDraw.Draw(img)
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Draw Checkpoints (Red Dots)
        for name, pt in self.checkpoints.items():
            px = (pt['x'] - origin_x) / resolution
            py = height - 1 - ((pt['y'] - origin_y) / resolution)
            r = 3
            draw.ellipse((px - r, py - r, px + r, py + r), fill=(255, 0, 0))

        # Draw Robot Position (Blue Dot)
        try:
            # Look up current robot position on the map
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
            px = (rx - origin_x) / resolution
            py = height - 1 - ((ry - origin_y) / resolution)
            r = 5
            draw.ellipse((px - r, py - r, px + r, py + r), fill=(0, 100, 255))
        except Exception as e:
            if not hasattr(self, 'tf_error_printed'):
                print(f"[WARNING] Cannot draw blue dot. TF Error: {e}")
                self.tf_error_printed = True

        # Buffer output stream
        buf = BytesIO()
        img.save(buf, format='PNG')
        with map_lock:
            latest_map_img_bytes = buf.getvalue()

    def record_checkpoint(self, checkpoint_name):
        """Look up the transform from map coordinate origin to tracking base_link."""
        try:
            now = self.get_clock().now()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now, rclpy.duration.Duration(seconds=1.5))
            
            position = trans.transform.translation
            orientation = trans.transform.rotation
            
            self.checkpoints[checkpoint_name] = {
                "x": position.x,
                "y": position.y,
                "z": position.z,
                "qx": orientation.x,
                "qy": orientation.y,
                "qz": orientation.z,
                "qw": orientation.w
            }
            self.get_logger().info(f"Successfully pinned checkpoint '{checkpoint_name}'")
            print(f"[SUCCESS] Checkpoint '{checkpoint_name}' recorded.")
        except Exception as e:
            print(f"[ERROR] Could not pinpoint position via TF frames: {e}")

    def save_map_and_metadata(self, room_name):
        print(f"Executing system map preservation for: {room_name}...")
        # Hardcoded to /root/ to perfectly sync with host mounts
        import subprocess
        # Explicitly inherit the environment (so ROS_LOCALHOST_ONLY is passed) and use transient_local QoS
        env = os.environ.copy()
        env['ROS_LOCALHOST_ONLY'] = '1'
        try:
            result = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", f"/root/{room_name}_map", 
                 "--ros-args", "-p", "map_subscribe_transient_local:=true"],
                check=True,
                env=env,
                timeout=10
            )
            print(f"[SUCCESS] Map saved to /root/{room_name}_map")
        except subprocess.CalledProcessError as e:
            print(f"\n[ERROR] Map saver failed to connect to ROS network (Exit Code {e.returncode}).")
            print("Please run the save command manually in Terminal 5 instead!")
        except subprocess.TimeoutExpired:
            print(f"\n[ERROR] Map saver timed out waiting for the /map topic.")
            print("Please run the save command manually in Terminal 5 instead!")
        
        metadata_path = f"/root/{room_name}_checkpoints.json"
        with open(metadata_path, 'w') as f:
            json.dump(self.checkpoints, f, indent=4)
        print(f"[SUCCESS] Saved to /root/{room_name}_map and points saved to {metadata_path}")


def interactive_menu():
    rclpy.init()
    node = RobotManagerNode()
    
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()
    
    web_thread = threading.Thread(target=run_web_server, daemon=True)
    web_thread.start()
    
    print("\n==========================================")
    print("  ROS 2 Autonomous Mapping & Checkpointing  ")
    print("  Live Browser view running at: http://localhost:8000  ")
    print("==========================================\n")
    
    while True:
        print("\nAvailable Commands: \n 1: start_mapping \n 2: set_checkpoint \n 3: save \n 4: test_mapping \n 5: exit")
        choice = input("Select an option (1-5): ").strip()
        
        if choice == '1':
            print("\n[START MAPPING] Ready. Please run your LiDAR and slam_toolbox nodes in extra terminals.")
            
        elif choice == '2':
            name = input("Enter checkpoint name (e.g., desk_area): ").strip()
            if name:
                node.record_checkpoint(name)
                
        elif choice == '3':
            room = input("Enter map layout prefix (e.g., apartment): ").strip()
            if room:
                node.save_map_and_metadata(room)
                
        elif choice == '4':
            room = input("Enter the saved room map name (e.g., apartment): ").strip()
            checkpoint_file = f"/root/{room}_checkpoints.json"
            
            if not os.path.exists(checkpoint_file):
                print(f"[ERROR] Missing profile for '{room}' at {checkpoint_file}")
                continue
                
            with open(checkpoint_file, 'r') as f:
                saved_points = json.load(f)
                
            print(f"Available points: {list(saved_points.keys())}")
            target_pt = input("Which checkpoint should the robot target? ").strip()
            
            if target_pt in saved_points:
                pt = saved_points[target_pt]
                print(f"Navigating to {target_pt}...")
                
                navigator = BasicNavigator()
                
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = pt['x']
                goal_pose.pose.position.y = pt['y']
                goal_pose.pose.position.z = pt['z']
                goal_pose.pose.orientation.x = pt['qx']
                goal_pose.pose.orientation.y = pt['qy']
                goal_pose.pose.orientation.z = pt['qz']
                goal_pose.pose.orientation.w = pt['qw']
                
                navigator.goToPose(goal_pose)
                
                while not navigator.isTaskComplete():
                    feedback = navigator.getFeedback()
                    if feedback:
                        eta = feedback.estimated_time_remaining
                        eta_sec = eta.sec + eta.nanosec * 1e-9
                        print(f"Moving... ETA: {eta_sec:.1f}s", end="\r")
                    time.sleep(1.0)
                    
                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    print(f"\n[SUCCESS] Safely reached destination: {target_pt}!")
                else:
                    print(f"\n[FAILED] Could not complete navigation trajectory.")
            else:
                print(f"Target '{target_pt}' not found.")
                
        elif choice == '5':
            print("Shutting down application node framework.")
            rclpy.shutdown()
            sys.exit(0)

if __name__ == '__main__':
    interactive_menu()
