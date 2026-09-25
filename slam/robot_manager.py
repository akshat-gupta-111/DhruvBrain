#!/usr/bin/env python3
import os
import sys
import json
import time
import threading
from io import BytesIO
from http.server import BaseHTTPRequestHandler, HTTPServer
from PIL import Image

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Global memory to share map graphics with the port 8000 Web Server
latest_map_img_bytes = b""
map_lock = threading.Lock()

class MapWebServer(BaseHTTPRequestHandler):
    """Serve a basic HTML page and stream live map updates over port 8000."""
    def do_GET(self):
        global latest_map_img_bytes
        if self.path == '/':
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            html = """
            <html>
            <head><title>ROS 2 Live Map Stream</title><meta http-equiv="refresh" content="2"></head>
            <body style="background:#222; color:#fff; text-align:center; font-family:sans-serif;">
                <h2>Live Map Simulation Viewer</h2>
                <div><img src="/map.png" style="border:2px solid #555; max-width:90%; max-height:80vh;" /></div>
                <p>Auto-refreshing every 2 seconds...</p>
            </body>
            </html>
            """
            self.wfile.write(html.encode('utf-8'))
        elif self.path == '/map.png':
            with map_lock:
                img_data = latest_map_img_bytes
            if img_data:
                self.send_response(200)
                self.send_header("Content-type", "image/png")
                self.end_headers()
                self.wfile.write(img_data)
            else:
                self.send_response(404)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

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
        global latest_map_img_bytes
        width = msg.info.width
        height = msg.info.height
        if width == 0 or height == 0:
            return

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
        result = subprocess.run(
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", f"/root/{room_name}_map", 
             "--ros-args", "-p", "map_subscribe_transient_local:=true"],
            check=True,
            env=env
        )
        print(f"[SUCCESS] Map saved to /root/{room_name}_map")
        
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
