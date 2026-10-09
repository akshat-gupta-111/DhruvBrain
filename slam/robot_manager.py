#!/usr/bin/env python3
"""
DHRUV SLAM – All-in-one ROS 2 mapping manager.
Single-terminal operation: manages SLAM subprocesses and exposes
a full web control panel on port 8000.
"""
import os
import sys
import json
import time
import urllib.parse
import threading
import subprocess
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

# ─────────────────────────────────────────────────────────────────
#  Globals shared between ROS node, web server and control panel
# ─────────────────────────────────────────────────────────────────
latest_map_img_bytes: bytes = b""
map_lock = threading.Lock()
map_received_count = 0

# Live feedback line shown in the web Control Panel
cmd_log: list[str] = []
cmd_log_lock = threading.Lock()

SAVE_DIR = "/root"

# Subprocesses started by "Launch SLAM" button
_slam_procs: list[subprocess.Popen] = []
_slam_lock = threading.Lock()

# Reference to the global node (set in main)
_node: "RobotManagerNode | None" = None


def _log(msg: str):
    """Append a timestamped line to the web control-panel log."""
    ts = time.strftime("%H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line)
    with cmd_log_lock:
        cmd_log.append(line)
        if len(cmd_log) > 60:
            cmd_log.pop(0)


# ─────────────────────────────────────────────────────────────────
#  Saved-map helpers
# ─────────────────────────────────────────────────────────────────
def _list_saved_maps() -> dict:
    sets: dict = {}
    try:
        for fname in os.listdir(SAVE_DIR):
            if fname.endswith("_map.pgm"):
                name = fname[: -len("_map.pgm")]
                sets.setdefault(name, {})
                sets[name]["pgm"]  = os.path.isfile(os.path.join(SAVE_DIR, f"{name}_map.pgm"))
                sets[name]["yaml"] = os.path.isfile(os.path.join(SAVE_DIR, f"{name}_map.yaml"))
                sets[name]["json"] = os.path.isfile(os.path.join(SAVE_DIR, f"{name}_checkpoints.json"))
    except Exception:
        pass
    return sets


def _make_placeholder_png(width: int = 480, height: int = 240) -> bytes:
    img = Image.new("RGB", (width, height), color=(40, 40, 40))
    draw = ImageDraw.Draw(img)
    draw.text((width // 2, height // 2 - 20), "Waiting for /map data...", fill=(180, 180, 180), anchor="mm")
    draw.text((width // 2, height // 2 + 10), "Drive the robot to start SLAM mapping.", fill=(120, 120, 120), anchor="mm")
    buf = BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


PLACEHOLDER_PNG = _make_placeholder_png()

_MIME = {
    ".pgm":  "image/x-portable-graymap",
    ".yaml": "text/yaml",
    ".json": "application/json",
    ".png":  "image/png",
}


# ─────────────────────────────────────────────────────────────────
#  Web server
# ─────────────────────────────────────────────────────────────────
class MapWebServer(BaseHTTPRequestHandler):
    """Full SLAM control panel + live map on port 8000."""

    def log_message(self, format, *args):  # noqa: A002
        pass  # Silence per-request noise

    # ── helpers ────────────────────────────────────────────────────
    def _send(self, code: int, ctype: str, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, filepath: str, ext: str, dl_name: str):
        with open(filepath, "rb") as f:
            data = f.read()
        self.send_response(200)
        self.send_header("Content-Type", _MIME.get(ext, "application/octet-stream"))
        self.send_header("Content-Disposition", f'attachment; filename="{dl_name}"')
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    # ── GET ────────────────────────────────────────────────────────
    def do_GET(self):
        global latest_map_img_bytes, map_received_count

        p = self.path.split("?", 1)[0]  # path without query string
        qs = urllib.parse.parse_qs(self.path[len(p) + 1:] if "?" in self.path else "")

        if p == "/":
            self._serve_main_page()

        elif p == "/map.png":
            with map_lock:
                data = latest_map_img_bytes
            self._send(200, "image/png", data if data else PLACEHOLDER_PNG)

        elif p == "/snapshot.png":
            with map_lock:
                data = latest_map_img_bytes
            if not data:
                self._send(404, "text/plain", b"No map data yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Disposition", 'attachment; filename="live_map_snapshot.png"')
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        elif p == "/download":
            filename = qs.get("file", [""])[0]
            basename = os.path.basename(filename)
            ext = os.path.splitext(basename)[1].lower()
            if ext not in _MIME or ".." in filename:
                self._send(400, "text/plain", b"Invalid file")
                return
            filepath = os.path.join(SAVE_DIR, basename)
            if not os.path.isfile(filepath):
                self._send(404, "text/plain", b"File not found")
                return
            self._send_file(filepath, ext, basename)

        elif p == "/annotated.png":
            map_name = os.path.basename(qs.get("map", [""])[0])
            self._serve_annotated_png(map_name)

        elif p == "/status":
            with map_lock:
                count = map_received_count
            with cmd_log_lock:
                logs = list(cmd_log)
            payload = json.dumps({"map_frames_received": count, "map_live": count > 0, "log": logs}, indent=2)
            self._send(200, "application/json", payload.encode())

        elif p == "/checkpoints":
            # Return current in-memory checkpoints as JSON
            if _node:
                self._send(200, "application/json", json.dumps(_node.checkpoints, indent=2).encode())
            else:
                self._send(503, "application/json", b"{}")

        else:
            self._send(404, "text/plain", b"Not found")

    # ── POST ───────────────────────────────────────────────────────
    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length)
        try:
            data = json.loads(body) if body else {}
        except Exception:
            data = {}

        p = self.path

        if p == "/api/launch_slam":
            threading.Thread(target=_launch_slam_stack, daemon=True).start()
            self._send(200, "application/json", b'{"ok":true}')

        elif p == "/api/stop_slam":
            threading.Thread(target=_stop_slam_stack, daemon=True).start()
            self._send(200, "application/json", b'{"ok":true}')

        elif p == "/api/set_checkpoint":
            name = data.get("name", "").strip()
            if not name:
                self._send(400, "application/json", b'{"error":"name required"}')
                return
            if _node:
                threading.Thread(target=_node.record_checkpoint, args=(name,), daemon=True).start()
                self._send(200, "application/json", b'{"ok":true}')
            else:
                self._send(503, "application/json", b'{"error":"node not ready"}')

        elif p == "/api/save_map":
            room = data.get("name", "").strip()
            if not room:
                self._send(400, "application/json", b'{"error":"name required"}')
                return
            if _node:
                threading.Thread(target=_node.save_map_and_metadata, args=(room,), daemon=True).start()
                self._send(200, "application/json", b'{"ok":true}')
            else:
                self._send(503, "application/json", b'{"error":"node not ready"}')

        elif p == "/api/navigate":
            map_name = data.get("map_name", "").strip()
            target = data.get("target", "").strip()
            if not map_name or not target:
                self._send(400, "application/json", b'{"error":"map_name and target required"}')
                return
            if _node:
                threading.Thread(target=_node.navigate_to, args=(map_name, target), daemon=True).start()
                self._send(200, "application/json", b'{"ok":true}')
            else:
                self._send(503, "application/json", b'{"error":"node not ready"}')

        else:
            self._send(404, "text/plain", b"Unknown API endpoint")

    # ── annotated PNG ──────────────────────────────────────────────
    def _serve_annotated_png(self, map_name: str):
        pgm_path  = os.path.join(SAVE_DIR, f"{map_name}_map.pgm")
        yaml_path = os.path.join(SAVE_DIR, f"{map_name}_map.yaml")
        json_path = os.path.join(SAVE_DIR, f"{map_name}_checkpoints.json")
        if not all(os.path.exists(x) for x in [pgm_path, yaml_path, json_path]):
            self._send(404, "text/plain", b"Missing map files")
            return
        try:
            with open(yaml_path) as f:
                meta = yaml.safe_load(f)
            resolution = meta["resolution"]
            ox, oy = meta["origin"][0], meta["origin"][1]

            img = Image.open(pgm_path).convert("RGBA")
            draw = ImageDraw.Draw(img)
            _, height = img.size
            font = ImageFont.load_default()

            with open(json_path) as f:
                cps = json.load(f)
            for name, coords in cps.items():
                px = (coords["x"] - ox) / resolution
                py = height - ((coords["y"] - oy) / resolution)
                r = 5
                draw.ellipse((px - r, py - r, px + r, py + r), fill="red", outline="black")
                draw.text((px + 10, py - 10), name, fill="red", font=font)

            buf = BytesIO()
            img.save(buf, format="PNG")
            img_data = buf.getvalue()

            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Disposition", f'attachment; filename="{map_name}_annotated.png"')
            self.send_header("Content-Length", str(len(img_data)))
            self.end_headers()
            self.wfile.write(img_data)
        except Exception as e:
            self._send(500, "text/plain", f"Error: {e}".encode())

    # ── main page HTML ─────────────────────────────────────────────
    def _serve_main_page(self):
        with map_lock:
            has_map = bool(latest_map_img_bytes)
            count   = map_received_count

        saved = _list_saved_maps()

        # Build saved-maps table
        if saved:
            dl_rows = ""
            for mname, files in sorted(saved.items()):
                def btn(label, href, ok):
                    return f'<a class="btn" href="{href}">{label}</a>' if ok else f'<span class="btn disabled">{label}</span>'
                dl_rows += f"""
                <tr>
                  <td class="mapname">{mname}</td>
                  <td>{btn("⬋ .pgm",  f"/download?file={mname}_map.pgm",  files.get("pgm"))}</td>
                  <td>{btn("⬋ .yaml", f"/download?file={mname}_map.yaml", files.get("yaml"))}</td>
                  <td>{btn("⬋ .json", f"/download?file={mname}_checkpoints.json", files.get("json"))}</td>
                  <td>{btn("🖼 Annotated", f"/annotated.png?map={mname}", files.get("pgm") and files.get("yaml") and files.get("json"))}</td>
                </tr>"""
            downloads_html = f"""
            <div class="card">
              <h2>💾 Saved Maps</h2>
              <table><thead><tr>
                <th>Name</th><th>.pgm</th><th>.yaml</th><th>.json</th><th>Visual</th>
              </tr></thead><tbody>{dl_rows}</tbody></table>
            </div>"""
        else:
            downloads_html = '<div class="card"><p class="muted">No saved maps yet — use the Save Map button.</p></div>'

        snap_btn = ('<a class="btn" href="/snapshot.png">⬋ Download live map (.png)</a>'
                    if has_map else '<span class="btn disabled">Live PNG (no map yet)</span>')

        with cmd_log_lock:
            log_html = "\n".join(f"<div>{l}</div>" for l in reversed(cmd_log[-20:])) or "<div class='muted'>No activity yet.</div>"

        status_cls = "status-ok" if has_map else "status-wait"
        status_txt = (f"✔ Live — {count} frames received" if has_map
                      else "⚫ Waiting for /map — drive the robot to start mapping")

        html = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>DHRUV SLAM — Live Map</title>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;600;700&display=swap');
    * {{ box-sizing:border-box; margin:0; padding:0; }}
    body {{ background:#0a0a14; color:#dde; font-family:'Inter',system-ui,sans-serif; padding:20px; max-width:1100px; margin:auto; }}
    h1   {{ color:#00c8f0; font-size:1.7rem; margin-bottom:2px; text-align:center; letter-spacing:1px; }}
    .sub {{ text-align:center; color:#555; font-size:.82rem; margin-bottom:18px; }}
    .{status_cls} {{ text-align:center; font-size:.9rem; margin:8px 0 18px; padding:6px 14px; border-radius:20px; display:inline-block; width:100%; }}
    .status-ok   {{ background:#0d3320; color:#4caf50; border:1px solid #1e6640; }}
    .status-wait {{ background:#332800; color:#ff9800; border:1px solid #664e00; }}
    /* Layout */
    .grid {{ display:grid; grid-template-columns:1fr 1fr; gap:18px; margin-bottom:18px; }}
    @media(max-width:700px){{ .grid{{grid-template-columns:1fr;}} }}
    /* Map */
    .mapwrap {{ text-align:center; margin-bottom:6px; }}
    .mapwrap img {{ border:2px solid #1a1a38; border-radius:10px; max-width:100%; max-height:65vh; image-rendering:pixelated; }}
    /* Cards */
    .card {{ background:#10102a; border:1px solid #1e1e3e; border-radius:12px; padding:18px; margin-bottom:18px; }}
    .card h2 {{ color:#90dcef; font-size:1rem; font-weight:700; margin-bottom:14px; }}
    /* Buttons */
    .btn {{ display:inline-block; padding:7px 16px; border-radius:8px; background:#0077b6;
            color:#fff; text-decoration:none; font-size:.82rem; margin:3px; border:none;
            cursor:pointer; transition:background .2s,transform .1s; font-family:inherit; }}
    .btn:hover {{ background:#0096c7; transform:translateY(-1px); }}
    .btn.green  {{ background:#1b7a34; }} .btn.green:hover {{ background:#27a248; }}
    .btn.red    {{ background:#7a1b1b; }} .btn.red:hover {{ background:#a22727; }}
    .btn.orange {{ background:#7a5000; }} .btn.orange:hover {{ background:#b07000; }}
    .btn.disabled {{ background:#1c1c3a; color:#444; cursor:default; transform:none; }}
    /* Inputs */
    input[type=text] {{ background:#1a1a30; border:1px solid #2a2a50; border-radius:6px;
                        color:#eee; padding:7px 12px; font-size:.85rem; font-family:inherit;
                        margin-right:6px; width:calc(100% - 100px); }}
    .row {{ display:flex; align-items:center; margin-bottom:10px; flex-wrap:wrap; gap:6px; }}
    /* Log */
    .logbox {{ background:#070710; border:1px solid #1a1a30; border-radius:8px;
               padding:12px; font-family:monospace; font-size:.78rem; color:#8af;
               max-height:200px; overflow-y:auto; }}
    /* Table */
    table {{ width:100%; border-collapse:collapse; font-size:.85rem; }}
    th {{ color:#90dcef; border-bottom:1px solid #1e1e3e; padding:7px 10px; text-align:left; }}
    td {{ padding:7px 10px; border-bottom:1px solid #12122a; }}
    td.mapname {{ font-weight:600; color:#aac; }}
    .muted {{ color:#444; font-size:.85rem; }}
    .snap {{ text-align:center; margin:6px 0 14px; }}
  </style>
</head>
<body>
  <h1>🤖 DHRUV SLAM — Live Map</h1>
  <p class="sub"><a href="/status" style="color:#90dcef">diagnostics JSON</a></p>
  <div class="{status_cls}">{status_txt}</div>

  <div class="mapwrap">
    <img src="/map.png?t={int(time.time())}" alt="SLAM map" id="mapImg">
  </div>
  <div class="snap">{snap_btn}</div>

  <!-- Control Panel -->
  <div class="grid">

    <!-- SLAM Control -->
    <div class="card">
      <h2>⚙️ SLAM Control</h2>
      <p class="muted" style="margin-bottom:12px;font-size:.8rem;">
        Launch all required ROS nodes in one click (LiDAR → RF2O → SLAM Toolbox).
      </p>
      <button class="btn green" onclick="api('/api/launch_slam')">▶ Launch SLAM Stack</button>
      <button class="btn red"   onclick="api('/api/stop_slam')">■ Stop SLAM Stack</button>
    </div>

    <!-- Checkpoint -->
    <div class="card">
      <h2>📍 Set Checkpoint</h2>
      <p class="muted" style="margin-bottom:10px;font-size:.8rem;">
        Pin the robot's current position with a name.
      </p>
      <div class="row">
        <input type="text" id="cpName" placeholder="e.g. desk_area">
        <button class="btn orange" onclick="setCheckpoint()">Pin 📌</button>
      </div>
    </div>

    <!-- Save Map -->
    <div class="card">
      <h2>💾 Save Map</h2>
      <p class="muted" style="margin-bottom:10px;font-size:.8rem;">
        Saves map + checkpoints to disk.  The map files will appear in the table below.
      </p>
      <div class="row">
        <input type="text" id="mapName" placeholder="e.g. incubation">
        <button class="btn" onclick="saveMap()">Save 💾</button>
      </div>
    </div>

    <!-- Navigate -->
    <div class="card">
      <h2>🚀 Navigate to Checkpoint</h2>
      <p class="muted" style="margin-bottom:10px;font-size:.8rem;">
        Autonomously drive to a saved checkpoint (requires Nav2 running).
      </p>
      <div class="row">
        <input type="text" id="navMap"    placeholder="Map name (e.g. incubation)" style="width:48%">
        <input type="text" id="navTarget" placeholder="Checkpoint name"            style="width:48%; margin-right:0;">
      </div>
      <button class="btn green" onclick="navigate()">🚀 Go!</button>
    </div>

  </div>

  <!-- Activity Log -->
  <div class="card">
    <h2>📋 Activity Log</h2>
    <div class="logbox" id="logBox">{log_html}</div>
  </div>

  {downloads_html}

<script>
  // Auto-refresh the map image every 2s without reloading the full page
  setInterval(() => {{
    const img = document.getElementById('mapImg');
    if (img) img.src = '/map.png?t=' + Date.now();
  }}, 2000);

  // Auto-refresh the log every 3s
  setInterval(refreshLog, 3000);

  function refreshLog() {{
    fetch('/status').then(r => r.json()).then(d => {{
      const box = document.getElementById('logBox');
      if (!box || !d.log) return;
      box.innerHTML = d.log.slice(-20).reverse().map(l => '<div>' + l + '</div>').join('');
    }}).catch(() => {{}});
  }}

  function api(endpoint, body) {{
    return fetch(endpoint, {{
      method: 'POST',
      headers: {{'Content-Type': 'application/json'}},
      body: body ? JSON.stringify(body) : '{{}}'
    }}).then(r => r.json()).then(d => {{
      refreshLog();
      return d;
    }});
  }}

  function setCheckpoint() {{
    const name = document.getElementById('cpName').value.trim();
    if (!name) {{ alert('Please enter a checkpoint name!'); return; }}
    api('/api/set_checkpoint', {{name}}).then(() => document.getElementById('cpName').value = '');
  }}

  function saveMap() {{
    const name = document.getElementById('mapName').value.trim();
    if (!name) {{ alert('Please enter a map name!'); return; }}
    api('/api/save_map', {{name}});
  }}

  function navigate() {{
    const map_name = document.getElementById('navMap').value.trim();
    const target   = document.getElementById('navTarget').value.trim();
    if (!map_name || !target) {{ alert('Please fill in both map name and checkpoint name!'); return; }}
    if (!confirm('Send robot to: ' + target + '?')) return;
    api('/api/navigate', {{map_name, target}});
  }}
</script>
</body>
</html>"""
        self._send(200, "text/html; charset=utf-8", html.encode())


# ─────────────────────────────────────────────────────────────────
#  SLAM subprocess management (single-terminal operation)
# ─────────────────────────────────────────────────────────────────
def _ros_env() -> dict:
    env = os.environ.copy()
    env["ROS_LOCALHOST_ONLY"] = "1"
    return env


def _launch_slam_stack():
    global _slam_procs
    with _slam_lock:
        if _slam_procs:
            _log("SLAM stack is already running. Stop it first.")
            return

    _log("Starting SLAM stack (LiDAR → RF2O → SLAM Toolbox)...")
    env = _ros_env()
    cmds = [
        # Static TF: base_link → base_footprint
        ["ros2", "run", "tf2_ros", "static_transform_publisher",
         "--x", "0", "--y", "0", "--z", "0",
         "--roll", "0", "--pitch", "0", "--yaw", "0",
         "--frame-id", "base_link", "--child-frame-id", "base_footprint"],
        # Static TF: base_link → laser
        ["ros2", "run", "tf2_ros", "static_transform_publisher",
         "--x", "0", "--y", "0", "--z", "0.16",
         "--roll", "0", "--pitch", "0", "--yaw", "0",
         "--frame-id", "base_link", "--child-frame-id", "laser"],
        # RPLidar
        ["ros2", "launch", "rplidar_ros", "rplidar_a1_launch.py"],
        # RF2O laser odometry
        ["ros2", "launch", "rf2o_laser_odometry", "rf2o_laser_odometry.launch.py",
         "laser_scan_topic:=/scan", "odom_topic:=/odom",
         "publish_tf:=true", "base_frame_id:=base_link", "odom_frame_id:=odom"],
        # SLAM Toolbox
        ["ros2", "launch", "slam_toolbox", "online_async_launch.py", "use_sim_time:=false"],
    ]

    procs = []
    for cmd in cmds:
        _log(f"Starting: {' '.join(cmd[:3])}...")
        try:
            p = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            procs.append(p)
            time.sleep(1.5)  # Stagger each launch so TFs are ready before next node
        except Exception as e:
            _log(f"ERROR launching {cmd[0]}: {e}")

    with _slam_lock:
        _slam_procs.extend(procs)
    _log(f"SLAM stack launched ({len(procs)} processes). Watch the map update!")


def _stop_slam_stack():
    global _slam_procs
    with _slam_lock:
        procs = list(_slam_procs)
        _slam_procs.clear()
    if not procs:
        _log("No SLAM processes running.")
        return
    _log(f"Stopping {len(procs)} SLAM processes...")
    for p in reversed(procs):
        try:
            p.terminate()
        except Exception:
            pass
    time.sleep(2)
    for p in procs:
        try:
            p.kill()
        except Exception:
            pass
    _log("SLAM stack stopped.")


# ─────────────────────────────────────────────────────────────────
#  ROS node
# ─────────────────────────────────────────────────────────────────
class RobotManagerNode(Node):
    def __init__(self):
        super().__init__("robot_manager_node")
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.checkpoints: dict = {}
        self._tf_warn_done = False

    # ── map → image ────────────────────────────────────────────────
    def map_callback(self, msg: OccupancyGrid):
        global latest_map_img_bytes, map_received_count
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        map_received_count += 1

        img = Image.new("RGB", (w, h))
        px = img.load()
        for y in range(h):
            for x in range(w):
                v = msg.data[x + y * w]
                if   v == 0:   px[x, h - 1 - y] = (255, 255, 255)
                elif v == 100: px[x, h - 1 - y] = (0, 0, 0)
                else:          px[x, h - 1 - y] = (127, 127, 127)

        draw = ImageDraw.Draw(img)
        res = msg.info.resolution
        ox  = msg.info.origin.position.x
        oy  = msg.info.origin.position.y

        # Red checkpoint dots
        for name, pt in self.checkpoints.items():
            cpx = (pt["x"] - ox) / res
            cpy = h - 1 - ((pt["y"] - oy) / res)
            r = 4
            draw.ellipse((cpx - r, cpy - r, cpx + r, cpy + r), fill=(220, 30, 30))

        # Blue robot dot
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            rx = (tf.transform.translation.x - ox) / res
            ry = h - 1 - ((tf.transform.translation.y - oy) / res)
            r  = 6
            draw.ellipse((rx - r, ry - r, rx + r, ry + r), fill=(30, 120, 255), outline=(200, 230, 255))
        except Exception as e:
            if not self._tf_warn_done:
                _log(f"[WARNING] Blue dot unavailable: {e}")
                self._tf_warn_done = True

        buf = BytesIO()
        img.save(buf, format="PNG")
        with map_lock:
            latest_map_img_bytes = buf.getvalue()

    # ── checkpoint ────────────────────────────────────────────────
    def record_checkpoint(self, name: str):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link",
                                                  rclpy.time.Time(),
                                                  rclpy.duration.Duration(seconds=2))
            t = tf.transform.translation
            q = tf.transform.rotation
            self.checkpoints[name] = {"x": t.x, "y": t.y, "z": t.z,
                                       "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w}
            _log(f"📍 Checkpoint '{name}' saved at ({t.x:.2f}, {t.y:.2f})")
        except Exception as e:
            _log(f"[ERROR] Cannot record checkpoint: {e}")

    # ── save map ──────────────────────────────────────────────────
    def save_map_and_metadata(self, room: str):
        _log(f"Saving map '{room}'...")
        env = _ros_env()
        try:
            subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli",
                 "-f", f"/root/{room}_map",
                 "--ros-args", "-p", "map_subscribe_transient_local:=true",
                 "-p", "use_sim_time:=false"],
                check=True, env=env, timeout=30
            )
            _log(f"✅ Map grid saved to /root/{room}_map.pgm + .yaml")
        except subprocess.CalledProcessError as e:
            _log(f"[ERROR] map_saver_cli failed (code {e.returncode}). "
                 "Run manually: ros2 run nav2_map_server map_saver_cli -f /root/{room}_map "
                 "--ros-args -p map_subscribe_transient_local:=true")
        except subprocess.TimeoutExpired:
            _log("[ERROR] map_saver_cli timed out. Try the manual Terminal 5 command.")

        # Always save the checkpoints JSON regardless of map_saver result
        json_path = f"/root/{room}_checkpoints.json"
        with open(json_path, "w") as f:
            json.dump(self.checkpoints, f, indent=4)
        _log(f"✅ Checkpoints saved to {json_path}")

    # ── navigate ──────────────────────────────────────────────────
    def navigate_to(self, map_name: str, target: str):
        json_path = f"/root/{map_name}_checkpoints.json"
        if not os.path.exists(json_path):
            _log(f"[ERROR] No checkpoint file: {json_path}")
            return
        with open(json_path) as f:
            pts = json.load(f)
        if target not in pts:
            _log(f"[ERROR] Checkpoint '{target}' not found in {map_name}. "
                 f"Available: {list(pts.keys())}")
            return

        pt = pts[target]
        _log(f"🚀 Navigating to '{target}' at ({pt['x']:.2f}, {pt['y']:.2f})...")

        navigator = BasicNavigator()
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = navigator.get_clock().now().to_msg()
        goal.pose.position.x = pt["x"]
        goal.pose.position.y = pt["y"]
        goal.pose.position.z = pt.get("z", 0.0)
        goal.pose.orientation.x = pt.get("qx", 0.0)
        goal.pose.orientation.y = pt.get("qy", 0.0)
        goal.pose.orientation.z = pt.get("qz", 0.0)
        goal.pose.orientation.w = pt.get("qw", 1.0)

        navigator.goToPose(goal)
        while not navigator.isTaskComplete():
            fb = navigator.getFeedback()
            if fb:
                eta = fb.estimated_time_remaining
                _log(f"Moving... ETA {eta.sec + eta.nanosec*1e-9:.1f}s")
            time.sleep(2.0)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            _log(f"✅ Reached '{target}'!")
        else:
            _log(f"[FAILED] Could not reach '{target}'.")


# ─────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────
def main():
    global _node
    rclpy.init()
    _node = RobotManagerNode()

    ros_thread = threading.Thread(target=lambda: rclpy.spin(_node), daemon=True)
    ros_thread.start()

    web_thread = threading.Thread(target=lambda: HTTPServer(("0.0.0.0", 8000), MapWebServer).serve_forever(), daemon=True)
    web_thread.start()

    _log("DHRUV SLAM started. Web panel at http://localhost:8000")
    _log("Use the browser buttons to launch SLAM, set checkpoints, save maps, and navigate.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        _log("Shutting down...")
        _stop_slam_stack()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
