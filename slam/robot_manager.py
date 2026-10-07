#!/usr/bin/env python3
import os
import sys
import json
import time
import threading
from io import BytesIO
import yaml
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


# MIME types for downloadable files
_MIME = {
    '.pgm':  'image/x-portable-graymap',
    '.yaml': 'text/yaml',
    '.json': 'application/json',
    '.png':  'image/png',
}

SAVE_DIR = '/root'   # where robot_manager saves map files


def _list_saved_maps():
    """
    Scan SAVE_DIR for saved map sets and return a dict:
      { 'apartment': {'pgm': True, 'yaml': True, 'json': True}, ... }
    """
    sets = {}
    try:
        for fname in os.listdir(SAVE_DIR):
            if fname.endswith('_map.pgm'):
                name = fname[:-len('_map.pgm')]
                if name not in sets:
                    sets[name] = {}
                sets[name]['pgm']  = os.path.isfile(os.path.join(SAVE_DIR, f'{name}_map.pgm'))
                sets[name]['yaml'] = os.path.isfile(os.path.join(SAVE_DIR, f'{name}_map.yaml'))
                sets[name]['json'] = os.path.isfile(os.path.join(SAVE_DIR, f'{name}_checkpoints.json'))
    except Exception:
        pass
    return sets

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

            # Build the downloads section
            saved = _list_saved_maps()
            if saved:
                dl_rows = ''
                for map_name, files in sorted(saved.items()):
                    btn_pgm  = f'<a class="btn" href="/download?file={map_name}_map.pgm">&#11123; .pgm</a>'  if files.get('pgm')  else '<span class="btn disabled">.pgm</span>'
                    btn_yaml = f'<a class="btn" href="/download?file={map_name}_map.yaml">&#11123; .yaml</a>' if files.get('yaml') else '<span class="btn disabled">.yaml</span>'
                    btn_json = f'<a class="btn" href="/download?file={map_name}_checkpoints.json">&#11123; .json</a>' if files.get('json') else '<span class="btn disabled">.json</span>'
                    btn_anno = f'<a class="btn" href="/annotated.png?map={map_name}">&#128444; Annotated .png</a>' if files.get('pgm') and files.get('yaml') and files.get('json') else '<span class="btn disabled">Annotated .png</span>'
                    dl_rows += f'<tr><td class="mapname">{map_name}</td><td>{btn_pgm}</td><td>{btn_yaml}</td><td>{btn_json}</td><td>{btn_anno}</td></tr>'
                downloads_html = f"""
  <div class="card">
    <h2>&#128190; Saved Maps</h2>
    <table>
      <thead><tr><th>Map Name</th><th>Grid (.pgm)</th><th>Metadata (.yaml)</th><th>Checkpoints (.json)</th><th>Visual (.png)</th></tr></thead>
      <tbody>{dl_rows}</tbody>
    </table>
  </div>"""
            else:
                downloads_html = '<div class="card"><p style="color:#888">No saved maps yet. Use option 3 (save) in the terminal menu.</p></div>'

            with map_lock:
                snap_available = bool(latest_map_img_bytes)
            snap_btn = '<a class="btn" href="/snapshot.png">&#11123; Download live map (.png)</a>' if snap_available else '<span class="btn disabled">Live PNG (no map yet)</span>'

            html = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta http-equiv="refresh" content="3">
  <title>DHRUV SLAM &mdash; Live Map</title>
  <style>
    * {{ box-sizing:border-box; margin:0; padding:0; }}
    body {{ background:#0f0f1a; color:#e0e0e0;
            font-family:'Segoe UI',system-ui,sans-serif; padding:24px; }}
    h1   {{ color:#00b4d8; font-size:1.6rem; margin-bottom:4px; text-align:center; }}
    .sub {{ text-align:center; color:#666; font-size:.85rem; margin-bottom:20px; }}
    .status-ok  {{ color:#4caf50; text-align:center; margin:8px 0; }}
    .status-wait{{ color:#ff9800; text-align:center; margin:8px 0; }}
    .mapwrap {{ text-align:center; margin-bottom:24px; }}
    .mapwrap img {{ border:2px solid #2a2a4a; border-radius:8px;
                    max-width:100%; max-height:70vh; }}
    .card {{ background:#16213e; border:1px solid #2a2a4a; border-radius:10px;
             padding:20px; margin-bottom:20px; }}
    .card h2 {{ color:#90e0ef; font-size:1.1rem; margin-bottom:14px; }}
    table  {{ width:100%; border-collapse:collapse; font-size:.9rem; }}
    th     {{ color:#90e0ef; border-bottom:1px solid #2a2a4a;
             padding:8px 12px; text-align:left; }}
    td     {{ padding:8px 12px; border-bottom:1px solid #1a1a30; }}
    td.mapname {{ font-weight:600; color:#cce; }}
    .btn {{ display:inline-block; padding:5px 14px; border-radius:6px;
            background:#0077b6; color:#fff; text-decoration:none;
            font-size:.82rem; margin:2px; transition:background .2s; }}
    .btn:hover {{ background:#0096c7; }}
    .btn.disabled {{ background:#2a2a4a; color:#555; cursor:default; }}
    .snap {{ text-align:center; margin-top:4px; }}
  </style>
</head>
<body>
  <h1>DHRUV SLAM &mdash; Live Map</h1>
  <p class="sub">Auto-refreshing every 3 s &nbsp;|&nbsp; <a href="/status" style="color:#90e0ef">diagnostics</a></p>
  <p class="{'status-ok' if has_map else 'status-wait'}">
    {'&#10004; Live &mdash; ' + str(count) + ' frames received' if has_map else '&#9899; Waiting for /map &mdash; drive the robot to start mapping'}
  </p>

  <div class="mapwrap"><img src="/map.png" alt="SLAM map"></div>
  <div class="snap">{snap_btn}</div>

  {downloads_html}
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

        # ── /download?file=<name> ── serve a saved map file ───────────────────
        elif self.path.startswith('/download?file='):
            filename = self.path[len('/download?file='):]
            # Security: only allow basenames with known extensions, no path traversal
            basename = os.path.basename(filename)
            ext = os.path.splitext(basename)[1].lower()
            if ext not in _MIME or '..' in filename:
                self._send(400, 'text/plain', b'Invalid file')
                return
            filepath = os.path.join(SAVE_DIR, basename)
            if not os.path.isfile(filepath):
                self._send(404, 'text/plain', b'File not found')
                return
            with open(filepath, 'rb') as f:
                data = f.read()
            self.send_response(200)
            self.send_header('Content-Type', _MIME[ext])
            self.send_header('Content-Disposition', f'attachment; filename="{basename}"')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        # ── /snapshot.png ── download the current live map as PNG ─────────────
        elif self.path == '/snapshot.png':
            with map_lock:
                img_data = latest_map_img_bytes
            if not img_data:
                self._send(404, 'text/plain', b'No map data yet')
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.send_header('Content-Disposition', 'attachment; filename="live_map_snapshot.png"')
            self.send_header('Content-Length', str(len(img_data)))
            self.end_headers()
            self.wfile.write(img_data)

        # ── /annotated.png?map=<name> ── render annotated map ─────────────────
        elif self.path.startswith('/annotated.png?map='):
            map_name = self.path[len('/annotated.png?map='):]
            map_name = os.path.basename(map_name)  # Security
            
            pgm_path = os.path.join(SAVE_DIR, f"{map_name}_map.pgm")
            yaml_path = os.path.join(SAVE_DIR, f"{map_name}_map.yaml")
            json_path = os.path.join(SAVE_DIR, f"{map_name}_checkpoints.json")
            
            if not all(os.path.exists(p) for p in [pgm_path, yaml_path, json_path]):
                self._send(404, 'text/plain', b'Missing map files (pgm, yaml, or json)')
                return
                
            try:
                # 1. Load YAML to get resolution and origin
                with open(yaml_path, 'r') as f:
                    map_meta = yaml.safe_load(f)
                
                resolution = map_meta['resolution']
                origin_x = map_meta['origin'][0]
                origin_y = map_meta['origin'][1]
                
                # 2. Load the PGM Image
                img = Image.open(pgm_path).convert("RGBA")
                draw = ImageDraw.Draw(img)
                width, height = img.size
                
                # Try to load a default font
                font = ImageFont.load_default()
                
                # 3. Load the Checkpoints
                with open(json_path, 'r') as f:
                    checkpoints = json.load(f)
                    
                # 4. Draw each checkpoint
                for name, coords in checkpoints.items():
                    world_x = coords['x']
                    world_y = coords['y']
                    
                    px = (world_x - origin_x) / resolution
                    py = height - ((world_y - origin_y) / resolution)
                    
                    r = 5
                    draw.ellipse((px - r, py - r, px + r, py + r), fill="red", outline="black")
                    draw.text((px + 10, py - 10), name, fill="red", font=font)
                    
                buf = BytesIO()
                img.save(buf, format='PNG')
                img_data = buf.getvalue()
                
                self.send_response(200)
                self.send_header('Content-Type', 'image/png')
                self.send_header('Content-Disposition', f'attachment; filename="{map_name}_annotated.png"')
                self.send_header('Content-Length', str(len(img_data)))
                self.end_headers()
                self.wfile.write(img_data)
            except Exception as e:
                self._send(500, 'text/plain', f'Error generating image: {e}'.encode())

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
            # Added use_sim_time:=false and increased timeout to 30s to fix "failed to spin map subscription"
            result = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", f"/root/{room_name}_map", 
                 "--ros-args", "-p", "map_subscribe_transient_local:=true", "-p", "use_sim_time:=false"],
                check=True,
                env=env,
                timeout=30
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
            with map_lock:
                count = map_received_count
            print("\n[MAPPING] All SLAM nodes are already running in the background.")
            print(f"          /map frames received so far: {count}")
            if count == 0:
                print("          Map not received yet — drive the robot to start building it.")
                print("          Live view: http://localhost:8000")
            else:
                print(f"          Mapping is active. Live view: http://localhost:8000")


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
