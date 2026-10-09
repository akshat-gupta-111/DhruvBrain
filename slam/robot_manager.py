#!/usr/bin/env python3
import os
import sys
import json
import time
import threading
import queue
from io import BytesIO
import yaml
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
from PIL import Image, ImageDraw, ImageFont

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.task import Future

# ── Globals ──────────────────────────────────────────────────────────────────
latest_map_img_bytes = b""
map_lock = threading.Lock()
map_received_count = 0

# Command queue: web UI and terminal both push here; main loop drains it
cmd_queue = queue.Queue()

# Live log ring-buffer for the web UI
_log_lock = threading.Lock()
_log_lines = []
MAX_LOG = 100

# Shared node reference for the web handler
_node_ref = None

SAVE_DIR = '/root'

_MIME = {
    '.pgm':  'image/x-portable-graymap',
    '.yaml': 'text/yaml',
    '.json': 'application/json',
    '.png':  'image/png',
}


def _log(msg: str):
    """Print to stdout AND append to the web live-log panel."""
    print(msg)
    ts = time.strftime('%H:%M:%S')
    with _log_lock:
        _log_lines.append(f"[{ts}] {msg}")
        if len(_log_lines) > MAX_LOG:
            _log_lines.pop(0)


def _make_placeholder_png(width: int = 480, height: int = 240) -> bytes:
    img = Image.new('RGB', (width, height), color=(40, 40, 40))
    draw = ImageDraw.Draw(img)
    draw.text((width // 2, height // 2 - 20), "Waiting for /map data...", fill=(180, 180, 180), anchor='mm')
    draw.text((width // 2, height // 2 + 10), "Drive the robot to start SLAM mapping.", fill=(120, 120, 120), anchor='mm')
    buf = BytesIO()
    img.save(buf, format='PNG')
    return buf.getvalue()


PLACEHOLDER_PNG = _make_placeholder_png()


def _list_saved_maps():
    sets = {}
    try:
        for fname in os.listdir(SAVE_DIR):
            if fname.endswith('_map.pgm'):
                name = fname[:-len('_map.pgm')]
                sets[name] = {
                    'pgm':  os.path.isfile(os.path.join(SAVE_DIR, f'{name}_map.pgm')),
                    'yaml': os.path.isfile(os.path.join(SAVE_DIR, f'{name}_map.yaml')),
                    'json': os.path.isfile(os.path.join(SAVE_DIR, f'{name}_checkpoints.json')),
                }
    except Exception:
        pass
    return sets


# ── Web Server ────────────────────────────────────────────────────────────────

class MapWebServer(BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        pass  # silence per-request logs

    def do_GET(self):
        global map_received_count
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)

        # ── / ── main control panel ───────────────────────────────────────────
        if path == '/':
            with map_lock:
                has_map = bool(latest_map_img_bytes)
                count = map_received_count

            saved = _list_saved_maps()
            if saved:
                dl_rows = ''
                for map_name, files in sorted(saved.items()):
                    btn_pgm  = f'<a class="btn" href="/download?file={map_name}_map.pgm">&#11123; .pgm</a>'  if files.get('pgm')  else '<span class="btn disabled">.pgm</span>'
                    btn_yaml = f'<a class="btn" href="/download?file={map_name}_map.yaml">&#11123; .yaml</a>' if files.get('yaml') else '<span class="btn disabled">.yaml</span>'
                    btn_json = f'<a class="btn" href="/download?file={map_name}_checkpoints.json">&#11123; .json</a>' if files.get('json') else '<span class="btn disabled">.json</span>'
                    has_all  = files.get('pgm') and files.get('yaml') and files.get('json')
                    btn_anno = f'<a class="btn" href="/annotated.png?map={map_name}">&#128444; Annotated</a>' if has_all else '<span class="btn disabled">Annotated</span>'
                    dl_rows += f'<tr><td class="mapname">{map_name}</td><td>{btn_pgm}</td><td>{btn_yaml}</td><td>{btn_json}</td><td>{btn_anno}</td></tr>'
                downloads_html = f"""
  <div class="card">
    <h2>&#128190; Saved Maps</h2>
    <table>
      <thead><tr><th>Map Name</th><th>.pgm</th><th>.yaml</th><th>.json</th><th>Annotated</th></tr></thead>
      <tbody>{dl_rows}</tbody>
    </table>
  </div>"""
            else:
                downloads_html = '<div class="card"><p style="color:#555">No saved maps yet. Use the Save Map button.</p></div>'

            map_opts = '<option value="LIVE">Live Map (Current Session)</option>'
            for m in sorted(saved.keys()):
                map_opts += f'<option value="{m}">{m}</option>'

            snap_btn = '<a class="btn" href="/snapshot.png">&#11123; Live map (.png)</a>' if has_map else '<span class="btn disabled">Live PNG (no map yet)</span>'

            with _log_lock:
                log_html = '\n'.join(f'<div class="ll">{l}</div>' for l in reversed(_log_lines)) or '<div class="ll" style="color:#444">No activity yet...</div>'

            status_cls = 'badge-ok' if has_map else 'badge-wait'
            status_txt = f'&#10004; {count} frames received' if has_map else '&#9899; Waiting for /map &mdash; drive the robot'

            html = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>DHRUV SLAM &mdash; Control Panel</title>
  <style>
    *{{box-sizing:border-box;margin:0;padding:0;}}
    body{{background:#080910;color:#ccd;font-family:'Segoe UI',system-ui,sans-serif;padding:18px;}}
    h1{{color:#00b4d8;font-size:1.45rem;text-align:center;margin-bottom:3px;}}
    .sub{{text-align:center;color:#445;font-size:.8rem;margin-bottom:16px;}}
    .layout{{display:grid;grid-template-columns:1fr 1fr;gap:14px;max-width:1140px;margin:0 auto;}}
    .card{{background:#0f1624;border:1px solid #1a2640;border-radius:12px;padding:16px;margin-bottom:14px;}}
    .card h2{{color:#7ecfef;font-size:.95rem;margin-bottom:10px;}}
    /* Map */
    .mapwrap{{text-align:center;}}
    .mapwrap img{{max-width:100%;max-height:52vh;border-radius:8px;border:1px solid #1a2640;}}
    /* Status badge */
    .badge-ok{{display:inline-block;background:#0f2e10;color:#4caf50;border-radius:20px;padding:3px 12px;font-size:.76rem;margin-bottom:8px;}}
    .badge-wait{{display:inline-block;background:#2e1e08;color:#ff9800;border-radius:20px;padding:3px 12px;font-size:.76rem;margin-bottom:8px;}}
    /* Command buttons */
    .cmd-grid{{display:grid;grid-template-columns:1fr 1fr;gap:9px;}}
    .cbtn{{padding:13px 8px;border:none;border-radius:9px;font-size:.88rem;font-weight:700;cursor:pointer;
            display:flex;flex-direction:column;align-items:center;gap:3px;transition:filter .15s;}}
    .cbtn:hover{{filter:brightness(1.25);}}
    .cbtn .ic{{font-size:1.5rem;}}
    .cbtn .lb{{font-size:.7rem;font-weight:400;opacity:.75;}}
    .c1{{background:#0c4272;color:#fff;}}
    .c2{{background:#0d4a22;color:#fff;}}
    .c3{{background:#4a2c00;color:#fff;}}
    .c4{{background:#3a0858;color:#fff;}}
    /* Download/snap buttons */
    .btn{{display:inline-block;padding:5px 11px;border-radius:6px;background:#0077b6;color:#fff;
          text-decoration:none;font-size:.78rem;margin:2px;transition:background .2s;}}
    .btn:hover{{background:#0096c7;}}
    .btn.disabled{{background:#1a2640;color:#334;cursor:default;pointer-events:none;}}
    /* Log */
    .logbox{{background:#040508;border-radius:8px;padding:10px;height:230px;overflow-y:auto;
              font-family:monospace;font-size:.75rem;line-height:1.65;}}
    .ll{{color:#5eadc8;border-bottom:1px solid #0a0d14;padding:1px 0;}}
    /* Table */
    table{{width:100%;border-collapse:collapse;font-size:.8rem;}}
    th{{color:#7ecfef;border-bottom:1px solid #1a2640;padding:6px 10px;text-align:left;}}
    td{{padding:6px 10px;border-bottom:1px solid #0f1624;}}
    td.mapname{{font-weight:700;color:#99b;}}
    .snap{{text-align:center;margin-top:8px;}}
    /* Modal */
    .mb{{display:none;position:fixed;inset:0;background:rgba(0,0,0,.75);z-index:100;
          align-items:center;justify-content:center;}}
    .mb.open{{display:flex;}}
    .md{{background:#0f1624;border:1px solid #1a2640;border-radius:14px;
          padding:26px;min-width:300px;max-width:420px;width:90%;}}
    .md h3{{color:#7ecfef;margin-bottom:13px;font-size:.95rem;}}
    .md input,.md select{{width:100%;padding:9px 12px;border-radius:7px;
          border:1px solid #1a2640;background:#080910;color:#ccd;font-size:.88rem;margin-bottom:11px;}}
    .mbtns{{display:flex;gap:9px;justify-content:flex-end;}}
    .mbtns button{{padding:8px 18px;border:none;border-radius:7px;cursor:pointer;font-size:.85rem;font-weight:600;}}
    .bcancel{{background:#1a2640;color:#778;}}.bok{{background:#0096c7;color:#fff;}}
    @media(max-width:680px){{.layout{{grid-template-columns:1fr;}}.cmd-grid{{grid-template-columns:1fr 1fr;}}}}
  </style>
</head>
<body>
  <h1>&#129302; DHRUV SLAM &mdash; Control Panel</h1>
  <p class="sub">Web UI &nbsp;|&nbsp; <a href="/status" style="color:#7ecfef">diagnostics</a></p>

  <div class="layout">
    <!-- LEFT: map + commands -->
    <div>
      <div class="card">
        <h2>&#128247; Live Map</h2>
        <span class="{status_cls}">{status_txt}</span>
        <div class="mapwrap"><img id="mapimg" src="/map.png" alt="SLAM map"></div>
        <div class="snap">{snap_btn}</div>
      </div>

      <div class="card">
        <h2>&#127918; Commands</h2>
        <div class="cmd-grid">
          <button class="cbtn c1" onclick="runCmd(1)">
            <span class="ic">&#128205;</span>1 &mdash; Status<span class="lb">Check /map frames</span></button>
          <button class="cbtn c2" onclick="showModal('m-cp')">
            <span class="ic">&#128204;</span>2 &mdash; Set Checkpoint<span class="lb">Drop a named pin</span></button>
          <button class="cbtn c3" onclick="showModal('m-save')">
            <span class="ic">&#128190;</span>3 &mdash; Save Map<span class="lb">Write .pgm + checkpoints</span></button>
          <button class="cbtn c4" onclick="showModal('m-nav')">
            <span class="ic">&#128663;</span>4 &mdash; Navigate To<span class="lb">Drive to checkpoint</span></button>
        </div>
      </div>
    </div>

    <!-- RIGHT: log + downloads -->
    <div>
      <div class="card">
        <h2>&#128220; Live Log</h2>
        <div class="logbox" id="logbox">{log_html}</div>
      </div>
      {downloads_html}
    </div>
  </div>

  <!-- Modal: Set Checkpoint -->
  <div class="mb" id="m-cp">
    <div class="md">
      <h3>&#128204; Set Checkpoint</h3>
      <input id="cp-name" type="text" placeholder="e.g. desk_area" autofocus>
      <div class="mbtns">
        <button class="bcancel" onclick="hideModal('m-cp')">Cancel</button>
        <button class="bok" onclick="doCheckpoint()">Save Pin</button>
      </div>
    </div>
  </div>

  <!-- Modal: Save Map -->
  <div class="mb" id="m-save">
    <div class="md">
      <h3>&#128190; Save Map</h3>
      <input id="save-name" type="text" placeholder="e.g. incubation">
      <div class="mbtns">
        <button class="bcancel" onclick="hideModal('m-save')">Cancel</button>
        <button class="bok" onclick="doSave()">Save</button>
      </div>
    </div>
  </div>

  <!-- Modal: Navigate -->
  <div class="mb" id="m-nav">
    <div class="md">
      <h3>&#128663; Navigate To Checkpoint</h3>
      <select id="nav-map" onchange="loadCPs()">
        <option value="" disabled selected>-- Select Map --</option>
        {map_opts}
      </select>
      <select id="nav-cp"><option value="">-- select map first --</option></select>
      <div class="mbtns">
        <button class="bcancel" onclick="hideModal('m-nav')">Cancel</button>
        <button class="bok" onclick="doNav()">Go!</button>
      </div>
    </div>
  </div>

  <script>
    // Refresh map image every 3 s without page reload
    setInterval(() => {{
      const img = document.getElementById('mapimg');
      if(img) img.src = '/map.png?t=' + Date.now();
    }}, 3000);

    // Refresh log every 4 s
    setInterval(() => {{
      fetch('/api/log').then(r => r.text()).then(h => {{
        const b = document.getElementById('logbox');
        if(b) b.innerHTML = h;
      }});
    }}, 4000);

    function showModal(id) {{ document.getElementById(id).classList.add('open'); }}
    function hideModal(id) {{ document.getElementById(id).classList.remove('open'); }}

    function post(body) {{
      return fetch('/api/command', {{
        method: 'POST',
        headers: {{'Content-Type': 'application/json'}},
        body: JSON.stringify(body)
      }}).then(r => r.json());
    }}

    function runCmd(n) {{ post({{cmd: n}}); }}

    function doCheckpoint() {{
      const name = document.getElementById('cp-name').value.trim();
      if(!name) return;
      post({{cmd: 2, checkpoint_name: name}});
      hideModal('m-cp');
      document.getElementById('cp-name').value = '';
    }}

    function doSave() {{
      const name = document.getElementById('save-name').value.trim();
      if(!name) return;
      post({{cmd: 3, map_name: name}});
      hideModal('m-save');
      document.getElementById('save-name').value = '';
    }}

    function loadCPs() {{
      const m = document.getElementById('nav-map').value.trim();
      if(!m) return;
      fetch('/api/checkpoints?map=' + encodeURIComponent(m))
        .then(r => r.json())
        .then(pts => {{
          const sel = document.getElementById('nav-cp');
          sel.innerHTML = pts.length
            ? pts.map(p => `<option value="${{p}}">${{p}}</option>`).join('')
            : '<option value="">No checkpoints found</option>';
        }});
    }}

    function doNav() {{
      const m  = document.getElementById('nav-map').value.trim();
      const cp = document.getElementById('nav-cp').value;
      if(!m || !cp) return;
      post({{cmd: 4, map_name: m, checkpoint: cp}});
      hideModal('m-nav');
    }}
  </script>
</body>
</html>"""
            self._send(200, 'text/html; charset=utf-8', html.encode())

        # ── /map.png ─────────────────────────────────────────────────────────
        elif path == '/map.png':
            with map_lock:
                img_data = latest_map_img_bytes
            self._send(200, 'image/png', img_data if img_data else PLACEHOLDER_PNG)

        # ── /status ──────────────────────────────────────────────────────────
        elif path == '/status':
            with map_lock:
                count = map_received_count
            self._send(200, 'application/json', json.dumps({
                'map_frames_received': count, 'map_live': count > 0
            }, indent=2).encode())

        # ── /download ────────────────────────────────────────────────────────
        elif path == '/download':
            filename = qs.get('file', [''])[0]
            basename = os.path.basename(filename)
            ext = os.path.splitext(basename)[1].lower()
            if ext not in _MIME or '..' in filename:
                self._send(400, 'text/plain', b'Invalid file'); return
            filepath = os.path.join(SAVE_DIR, basename)
            if not os.path.isfile(filepath):
                self._send(404, 'text/plain', b'File not found'); return
            with open(filepath, 'rb') as f:
                data = f.read()
            self.send_response(200)
            self.send_header('Content-Type', _MIME[ext])
            self.send_header('Content-Disposition', f'attachment; filename="{basename}"')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        # ── /snapshot.png ─────────────────────────────────────────────────────
        elif path == '/snapshot.png':
            with map_lock:
                img_data = latest_map_img_bytes
            if not img_data:
                self._send(404, 'text/plain', b'No map data yet'); return
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.send_header('Content-Disposition', 'attachment; filename="live_map_snapshot.png"')
            self.send_header('Content-Length', str(len(img_data)))
            self.end_headers()
            self.wfile.write(img_data)

        # ── /api/log ──────────────────────────────────────────────────────────
        elif path == '/api/log':
            with _log_lock:
                html = '\n'.join(f'<div class="ll">{l}</div>' for l in reversed(_log_lines)) \
                       or '<div class="ll" style="color:#444">No activity yet...</div>'
            self._send(200, 'text/html; charset=utf-8', html.encode())

        # ── /api/checkpoints ─────────────────────────────────────────────────
        elif path == '/api/checkpoints':
            map_name = qs.get('map', [''])[0]
            json_path = os.path.join(SAVE_DIR, f'{map_name}_checkpoints.json')
            pts = []
            if os.path.isfile(json_path):
                try:
                    with open(json_path) as f:
                        pts = list(json.load(f).keys())
                except Exception:
                    pass
            if _node_ref is not None:
                for k in _node_ref.checkpoints:
                    if k not in pts:
                        pts.append(k)
            self._send(200, 'application/json', json.dumps(pts).encode())

        # ── /annotated.png ────────────────────────────────────────────────────
        elif path == '/annotated.png':
            map_name  = os.path.basename(qs.get('map', [''])[0])
            pgm_path  = os.path.join(SAVE_DIR, f"{map_name}_map.pgm")
            yaml_path = os.path.join(SAVE_DIR, f"{map_name}_map.yaml")
            json_path = os.path.join(SAVE_DIR, f"{map_name}_checkpoints.json")
            if not all(os.path.exists(p) for p in [pgm_path, yaml_path, json_path]):
                self._send(404, 'text/plain', b'Missing map files'); return
            try:
                with open(yaml_path) as f:
                    meta = yaml.safe_load(f)
                res = meta['resolution']
                ox, oy = meta['origin'][0], meta['origin'][1]
                img = Image.open(pgm_path).convert("RGBA")
                draw = ImageDraw.Draw(img)
                w, h = img.size
                font = ImageFont.load_default()
                with open(json_path) as f:
                    cps = json.load(f)
                for name, c in cps.items():
                    px = (c['x'] - ox) / res
                    py = h - ((c['y'] - oy) / res)
                    r = 5
                    draw.ellipse((px-r, py-r, px+r, py+r), fill="red", outline="black")
                    draw.text((px+10, py-10), name, fill="red", font=font)
                buf = BytesIO()
                img.save(buf, format='PNG')
                data = buf.getvalue()
                self.send_response(200)
                self.send_header('Content-Type', 'image/png')
                self.send_header('Content-Disposition', f'attachment; filename="{map_name}_annotated.png"')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            except Exception as e:
                self._send(500, 'text/plain', f'Error: {e}'.encode())

        else:
            self._send(404, 'text/plain', b'Not found')

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path != '/api/command':
            self._send(404, 'text/plain', b'Not found'); return
        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length)
        try:
            payload = json.loads(body)
        except Exception:
            self._send(400, 'application/json', b'{"error":"bad json"}'); return
        cmd_queue.put(payload)
        self._send(200, 'application/json', json.dumps({'queued': True}).encode())

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def run_web_server():
    server = HTTPServer(('0.0.0.0', 8000), MapWebServer)
    server.serve_forever()


# ── ROS Node ──────────────────────────────────────────────────────────────────

class RobotManagerNode(Node):
    def __init__(self):
        super().__init__('robot_manager_node')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.checkpoints = {}
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_nav_goal(self, pt, checkpoint_name):
        _log(f"[NAV] Waiting for NavigateToPose action server...")
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            _log("[ERROR] Nav2 Action Server not available. Is Nav2 running?")
            return
            
        _log(f"[NAV] Sending goal for '{checkpoint_name}'...")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pt['x']
        goal.pose.pose.position.y = pt['y']
        goal.pose.pose.position.z = pt['z']
        goal.pose.pose.orientation.x = pt['qx']
        goal.pose.pose.orientation.y = pt['qy']
        goal.pose.pose.orientation.z = pt['qz']
        goal.pose.pose.orientation.w = pt['qw']
        
        send_goal_future = self.nav_client.send_goal_async(goal, feedback_callback=self._nav_feedback_cb)
        send_goal_future.add_done_callback(self._nav_goal_response_cb)

    def _nav_feedback_cb(self, feedback_msg):
        eta = feedback_msg.feedback.estimated_time_remaining
        _log(f"[NAV] ETA: {eta.sec + eta.nanosec*1e-9:.1f}s")

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            _log("[NAV] Goal rejected by Nav2 server!")
            return
        _log("[NAV] Goal accepted, calculating path...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        result = future.result().status
        _log(f"[NAV] Navigation completed with status code: {result} (4=SUCCEEDED)")

    def map_callback(self, msg: OccupancyGrid):
        global latest_map_img_bytes, map_received_count
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        map_received_count += 1

        img = Image.new('RGB', (w, h))
        pix = img.load()
        for y in range(h):
            for x in range(w):
                v = msg.data[x + y * w]
                pix[x, h - 1 - y] = (255, 255, 255) if v == 0 else (0, 0, 0) if v == 100 else (127, 127, 127)

        draw = ImageDraw.Draw(img)
        res = msg.info.resolution
        ox  = msg.info.origin.position.x
        oy  = msg.info.origin.position.y

        # Red dots — checkpoints
        for _, pt in self.checkpoints.items():
            px = (pt['x'] - ox) / res
            py = h - 1 - ((pt['y'] - oy) / res)
            r = 3
            draw.ellipse((px-r, py-r, px+r, py+r), fill=(255, 0, 0))

        # Blue dot — robot position
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            px = (t.transform.translation.x - ox) / res
            py = h - 1 - ((t.transform.translation.y - oy) / res)
            r = 5
            draw.ellipse((px-r, py-r, px+r, py+r), fill=(0, 100, 255))
        except Exception as e:
            if not hasattr(self, '_tf_warned'):
                _log(f"[WARNING] Blue dot: TF Error: {e}")
                self._tf_warned = True

        buf = BytesIO()
        img.save(buf, format='PNG')
        with map_lock:
            latest_map_img_bytes = buf.getvalue()

    def record_checkpoint(self, name):
        try:
            now = self.get_clock().now()
            t = self.tf_buffer.lookup_transform('map', 'base_link', now, rclpy.duration.Duration(seconds=1.5))
            p = t.transform.translation
            q = t.transform.rotation
            self.checkpoints[name] = {'x': p.x, 'y': p.y, 'z': p.z,
                                       'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w}
            _log(f"[SUCCESS] Checkpoint '{name}' pinned at ({p.x:.2f}, {p.y:.2f})")
        except Exception as e:
            _log(f"[ERROR] Could not record checkpoint: {e}")

    def save_map_and_metadata(self, room_name):
        _log(f"[SAVE] Saving map '{room_name}'...")
        import subprocess
        env = os.environ.copy()
        env['ROS_LOCALHOST_ONLY'] = '1'
        try:
            subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli",
                 "-f", f"/root/{room_name}_map",
                 "--ros-args", "-p", "map_subscribe_transient_local:=true",
                 "-p", "use_sim_time:=false"],
                check=True, env=env, timeout=30
            )
            _log(f"[SUCCESS] Map grid saved to /root/{room_name}_map.pgm")
        except subprocess.CalledProcessError as e:
            _log(f"[ERROR] map_saver_cli failed (code {e.returncode}). Run manually in Terminal 5!")
        except subprocess.TimeoutExpired:
            _log("[ERROR] map_saver_cli timed out. Run manually in Terminal 5!")
        meta = f"/root/{room_name}_checkpoints.json"
        with open(meta, 'w') as f:
            json.dump(self.checkpoints, f, indent=4)
        _log(f"[SUCCESS] {len(self.checkpoints)} checkpoints → {meta}")


# ── Command Dispatcher ────────────────────────────────────────────────────────

def _handle_command(node, payload):
    cmd = payload.get('cmd')

    if cmd == 1:
        with map_lock:
            count = map_received_count
        _log(f"[STATUS] /map frames: {count}" + (" — active!" if count > 0 else " — drive robot to start."))

    elif cmd == 2:
        name = payload.get('checkpoint_name', '').strip()
        if name:
            node.record_checkpoint(name)
        else:
            _log("[ERROR] No checkpoint name provided.")

    elif cmd == 3:
        map_name = payload.get('map_name', '').strip()
        if map_name:
            threading.Thread(target=node.save_map_and_metadata, args=(map_name,), daemon=True).start()
        else:
            _log("[ERROR] No map name provided.")

    elif cmd == 4:
        map_name   = payload.get('map_name', '').strip()
        checkpoint = payload.get('checkpoint', '').strip()
        
        if map_name == 'LIVE':
            saved = node.checkpoints
            if not saved:
                _log("[ERROR] No checkpoints set in current Live session yet."); return
        else:
            cp_file = f"/root/{map_name}_checkpoints.json"
            if not os.path.exists(cp_file):
                _log(f"[ERROR] Map '{map_name}' not found at {cp_file}"); return
            with open(cp_file) as f:
                saved = json.load(f)
                
        if checkpoint not in saved:
            _log(f"[ERROR] Checkpoint '{checkpoint}' not in map '{map_name}'"); return
        
        pt = saved[checkpoint]
        node.send_nav_goal(pt, checkpoint)

    elif cmd == 5:
        _log("[EXIT] Shutting down...")
        rclpy.shutdown()
        sys.exit(0)


# ── Entry Point ───────────────────────────────────────────────────────────────

def interactive_menu():
    global _node_ref
    rclpy.init()
    node = RobotManagerNode()
    _node_ref = node

    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    threading.Thread(target=run_web_server, daemon=True).start()

    _log("==========================================")
    _log("  DHRUV SLAM — Control Panel")
    _log("  Web UI: http://localhost:8000")
    _log("  Terminal: 1 | 2:name | 3:mapname | 4:map,checkpoint | 5=exit")
    _log("==========================================")

    # Background thread for terminal input
    def _terminal():
        while True:
            try:
                line = input("").strip()
                if not line:
                    continue
                if ':' in line:
                    c, arg = line.split(':', 1)
                    c = int(c.strip())
                    arg = arg.strip()
                    if c == 2:
                        cmd_queue.put({'cmd': 2, 'checkpoint_name': arg})
                    elif c == 3:
                        cmd_queue.put({'cmd': 3, 'map_name': arg})
                    elif c == 4:
                        m, cp = (arg.split(',', 1) + [''])[:2]
                        cmd_queue.put({'cmd': 4, 'map_name': m.strip(), 'checkpoint': cp.strip()})
                else:
                    cmd_queue.put({'cmd': int(line)})
            except (ValueError, EOFError):
                pass

    threading.Thread(target=_terminal, daemon=True).start()

    while True:
        try:
            payload = cmd_queue.get(timeout=1.0)
            _handle_command(node, payload)
        except queue.Empty:
            pass


if __name__ == '__main__':
    interactive_menu()
