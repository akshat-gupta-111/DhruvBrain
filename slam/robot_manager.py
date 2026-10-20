#!/usr/bin/env python3
import os
import re
import sys
import json
import time
import html
import threading
import queue
from io import BytesIO
import yaml
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
from PIL import Image, ImageDraw, ImageFont

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.task import Future

# ── Globals ──────────────────────────────────────────────────────────────────
latest_map_img_bytes = b""
map_lock = threading.Lock()
map_received_count = 0

# Diagnostics freshness tracking
latest_scan_time = 0.0
latest_odom_time = 0.0
latest_map_time = 0.0

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

def _sanitize_name(name: str) -> str:
    """Sanitize name to allow only alphanumeric, underscores and hyphens (prevents directory traversal)."""
    clean = re.sub(r'[^a-zA-Z0-9_\-]', '', name)
    return clean.strip()

def _make_placeholder_png(width: int = 480, height: int = 240) -> bytes:
    img = Image.new('RGB', (width, height), color=(40, 40, 40))
    draw = ImageDraw.Draw(img)
    draw.text((width // 2, height // 2 - 20), "Waiting for /map data...", fill=(180, 180, 180), anchor='mm')
    draw.text((width // 2, height // 2 + 10), "Move the robot slightly so laser odometry starts.", fill=(120, 120, 120), anchor='mm')
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
        global map_received_count, latest_scan_time, latest_odom_time, latest_map_time
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)

        # ── / ── main control panel ───────────────────────────────────────────
        if path == '/':
            with map_lock:
                has_map = bool(latest_map_img_bytes)
                count = map_received_count

            saved = _list_saved_maps()
            # Read current active config if set
            current_active_mode = "mapping"
            current_active_map = ""
            if os.path.isfile("/root/default_mode.txt"):
                try:
                    with open("/root/default_mode.txt") as cf:
                        for cl in cf:
                            if "NAV_MODE=" in cl:
                                current_active_mode = cl.split("=")[1].replace('"', '').strip()
                            if "ACTIVE_MAP=" in cl:
                                current_active_map = cl.split("=")[1].replace('"', '').strip()
                except Exception:
                    pass

            if saved:
                dl_rows = ''
                for map_name, files in sorted(saved.items()):
                    esc_name = html.escape(map_name)
                    btn_pgm  = f'<a class="btn" href="/download?file={esc_name}_map.pgm">&#11123; .pgm</a>'  if files.get('pgm')  else '<span class="btn disabled">.pgm</span>'
                    btn_yaml = f'<a class="btn" href="/download?file={esc_name}_map.yaml">&#11123; .yaml</a>' if files.get('yaml') else '<span class="btn disabled">.yaml</span>'
                    btn_json = f'<a class="btn" href="/download?file={esc_name}_checkpoints.json">&#11123; .json</a>' if files.get('json') else '<span class="btn disabled">.json</span>'
                    has_all  = files.get('pgm') and files.get('yaml') and files.get('json')
                    btn_anno = f'<a class="btn" href="/annotated.png?map={esc_name}">&#128444; Annotated</a>' if has_all else '<span class="btn disabled">Annotated</span>'
                    
                    is_active = (current_active_mode == "localization" and current_active_map == map_name)
                    btn_load = f'<span class="badge-ok" style="margin:0">Active Map</span>' if is_active else f'<button class="btn" style="background:#7b2cbf;cursor:pointer;border:none" onclick="loadMapMode(\'{esc_name}\')">&#128640; Use Map</button>'
                    
                    dl_rows += f'<tr><td class="mapname">{esc_name}</td><td>{btn_load}</td><td>{btn_pgm}</td><td>{btn_yaml}</td><td>{btn_json}</td><td>{btn_anno}</td></tr>'
                
                mode_bar = f'<div style="margin-bottom:8px;font-size:0.82rem;color:#7ecfef">Current Stack: <b>{current_active_mode.upper()}</b> {("(" + current_active_map + ")") if current_active_map else ""}</div>'
                reset_btn = '<button class="btn" style="background:#d90429;border:none;cursor:pointer;margin-top:6px" onclick="resetMappingMode()">&#128260; Reset to New Mapping Mode</button>'
                downloads_html = f"""
  <div class="card">
    <h2>&#128190; Saved Maps &amp; Localization</h2>
    {mode_bar}
    <table>
      <thead><tr><th>Map Name</th><th>Action</th><th>.pgm</th><th>.yaml</th><th>.json</th><th>Annotated</th></tr></thead>
      <tbody>{dl_rows}</tbody>
    </table>
    {reset_btn}
  </div>"""
            else:
                downloads_html = '<div class="card"><p style="color:#555">No saved maps yet. Use the Save Map button.</p></div>'


            map_opts = '<option value="LIVE">Live Map (Current Session)</option>'
            for m in sorted(saved.keys()):
                esc_m = html.escape(m)
                map_opts += f'<option value="{esc_m}">{esc_m}</option>'

            snap_btn = '<a class="btn" href="/snapshot.png">&#11123; Live map (.png)</a>' if has_map else '<span class="btn disabled">Live PNG (no map yet)</span>'

            with _log_lock:
                log_html = '\n'.join(f'<div class="ll">{html.escape(l)}</div>' for l in reversed(_log_lines)) or '<div class="ll" style="color:#444">No activity yet...</div>'

            status_cls = 'badge-ok' if has_map else 'badge-wait'
            status_txt = f'&#10004; {count} frames received' if has_map else '&#9899; Waiting for /map &mdash; drive or rotate robot'

            html_page = f"""<!DOCTYPE html>
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
    .mapwrap{{text-align:center;}}
    .mapwrap img{{max-width:100%;max-height:52vh;border-radius:8px;border:1px solid #1a2640;}}
    .badge-ok{{display:inline-block;background:#0f2e10;color:#4caf50;border-radius:20px;padding:3px 12px;font-size:.76rem;margin-bottom:8px;}}
    .badge-wait{{display:inline-block;background:#2e1e08;color:#ff9800;border-radius:20px;padding:3px 12px;font-size:.76rem;margin-bottom:8px;}}
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
    .btn{{display:inline-block;padding:5px 11px;border-radius:6px;background:#0077b6;color:#fff;
          text-decoration:none;font-size:.78rem;margin:2px;transition:background .2s;}}
    .btn:hover{{background:#0096c7;}}
    .btn.disabled{{background:#1a2640;color:#334;cursor:default;pointer-events:none;}}
    .logbox{{background:#040508;border-radius:8px;padding:10px;height:230px;overflow-y:auto;
              font-family:monospace;font-size:.75rem;line-height:1.65;}}
    .ll{{color:#5eadc8;border-bottom:1px solid #0a0d14;padding:1px 0;}}
    table{{width:100%;border-collapse:collapse;font-size:.8rem;}}
    th{{color:#7ecfef;border-bottom:1px solid #1a2640;padding:6px 10px;text-align:left;}}
    td{{padding:6px 10px;border-bottom:1px solid #0f1624;}}
    td.mapname{{font-weight:700;color:#99b;}}
    .snap{{text-align:center;margin-top:8px;}}
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

    <div>
      <div class="card">
        <h2>&#128220; Live Log</h2>
        <div class="logbox" id="logbox">{log_html}</div>
      </div>
      {downloads_html}
    </div>
  </div>

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
    setInterval(() => {{
      const img = document.getElementById('mapimg');
      if(img) img.src = '/map.png?t=' + Date.now();
    }}, 3000);

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

    function loadMapMode(mapName) {{
      if(!confirm('Switch system to Localization Mode on saved map \"' + mapName + '\"?\\n\\nThis will configure ROS to localize against ' + mapName + ' and Nav2 to navigate to its saved checkpoints.')) return;
      post({{cmd: 6, map_name: mapName, mode: 'localization'}}).then(() => {{
        alert('Configuration saved for map \"' + mapName + '\".\\n\\nPlease run on Jetson terminal:\\nsudo docker compose restart\\n\\nAfter restart, the saved map will load permanently with full AMCL localization!');
        setTimeout(() => location.reload(), 1500);
      }});
    }}

    function resetMappingMode() {{
      if(!confirm('Switch back to SLAM Mapping Mode?\\n\\nThis resets the robot to explore and map new environments.')) return;
      post({{cmd: 6, map_name: '', mode: 'mapping'}}).then(() => {{
        alert('Set to SLAM Mapping mode.\\n\\nPlease run on Jetson terminal:\\nsudo docker compose restart');
        setTimeout(() => location.reload(), 1500);
      }});
    }}
  </script>
</body>
</html>"""
            self._send(200, 'text/html; charset=utf-8', html_page.encode())

        # ── /map.png ─────────────────────────────────────────────────────────
        elif path == '/map.png':
            with map_lock:
                img_data = latest_map_img_bytes
            self._send(200, 'image/png', img_data if img_data else PLACEHOLDER_PNG)

        # ── /status ──────────────────────────────────────────────────────────
        elif path == '/status':
            now = time.time()
            with map_lock:
                count = map_received_count
            scan_fresh = (now - latest_scan_time < 3.0) and (latest_scan_time > 0)
            odom_fresh = (now - latest_odom_time < 3.0) and (latest_odom_time > 0)
            map_fresh  = (now - latest_map_time < 10.0) and (latest_map_time > 0)

            # Check TF transform status
            tf_available = False
            if _node_ref is not None:
                try:
                    _node_ref.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                    tf_available = True
                except Exception:
                    pass

            status_payload = {
                'map_frames_received': count,
                'map_live': map_fresh,
                'scan_live': scan_fresh,
                'odom_live': odom_fresh,
                'tf_map_to_base_live': tf_available,
                'seconds_since_scan': round(now - latest_scan_time, 1) if latest_scan_time > 0 else None,
                'seconds_since_odom': round(now - latest_odom_time, 1) if latest_odom_time > 0 else None,
                'seconds_since_map': round(now - latest_map_time, 1) if latest_map_time > 0 else None,
            }
            self._send(200, 'application/json', json.dumps(status_payload, indent=2).encode())

        # ── /download ────────────────────────────────────────────────────────
        elif path == '/download':
            filename = qs.get('file', [''])[0]
            basename = os.path.basename(filename)
            # Prevent directory traversal
            if '..' in filename or '/' in filename or '\\' in filename:
                self._send(400, 'text/plain', b'Invalid path'); return

            ext = os.path.splitext(basename)[1].lower()
            if ext not in _MIME:
                self._send(400, 'text/plain', b'Invalid file extension'); return

            # Enforce that only SLAM artifacts (_map.pgm, _map.yaml, _checkpoints.json) can be fetched
            if not any(basename.endswith(suffix) for suffix in ['_map.pgm', '_map.yaml', '_checkpoints.json']):
                self._send(403, 'text/plain', b'Access denied: Only map files are accessible'); return

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
                html_log = '\n'.join(f'<div class="ll">{html.escape(l)}</div>' for l in reversed(_log_lines)) \
                           or '<div class="ll" style="color:#444">No activity yet...</div>'
            self._send(200, 'text/html; charset=utf-8', html_log.encode())

        # ── /api/checkpoints ─────────────────────────────────────────────────
        elif path == '/api/checkpoints':
            map_name = _sanitize_name(qs.get('map', [''])[0])
            json_path = os.path.join(SAVE_DIR, f'{map_name}_checkpoints.json')
            pts = []
            if os.path.isfile(json_path):
                try:
                    with open(json_path) as f:
                        pts = list(json.load(f).keys())
                except Exception:
                    pass
            if _node_ref is not None and map_name == 'LIVE':
                for k in _node_ref.checkpoints:
                    if k not in pts:
                        pts.append(k)
            self._send(200, 'application/json', json.dumps(pts).encode())

        # ── /annotated.png ────────────────────────────────────────────────────
        elif path == '/annotated.png':
            map_name  = _sanitize_name(qs.get('map', [''])[0])
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
            if not isinstance(payload, dict):
                self._send(400, 'application/json', b'{"error":"payload must be a JSON object"}'); return
            if 'cmd' not in payload or not isinstance(payload['cmd'], int):
                self._send(400, 'application/json', b'{"error":"missing or invalid cmd field"}'); return
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
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.checkpoints = {}
        # If in localization mode with an active map, auto-load its saved checkpoints
        if os.path.isfile('/root/default_mode.txt'):
            try:
                active_m = ''
                with open('/root/default_mode.txt') as f:
                    for line in f:
                        if 'ACTIVE_MAP=' in line:
                            active_m = line.split('=')[1].replace('"', '').strip()
                if active_m:
                    cp_path = f'/root/{active_m}_checkpoints.json'
                    if os.path.isfile(cp_path):
                        with open(cp_path) as jf:
                            self.checkpoints = json.load(jf)
                        _log(f"[INIT] Loaded {len(self.checkpoints)} checkpoints from saved map '{active_m}'")
            except Exception as e:
                _log(f"[WARN] Failed to auto-load checkpoints: {e}")

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._last_eta_log = 0.0

    def set_initial_pose(self, x=0.0, y=0.0, yaw=0.0):
        import math
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        # Convert yaw to quaternion
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Covariance diagonal: 0.25m position variance, ~0.06 rad yaw variance
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.initial_pose_pub.publish(msg)
        _log(f"[POSE] Initialized robot pose at ({x:.2f}, {y:.2f}) on map.")


    def scan_callback(self, msg):
        global latest_scan_time
        latest_scan_time = time.time()

    def odom_callback(self, msg):
        global latest_odom_time
        latest_odom_time = time.time()

    def send_nav_goal(self, pt, checkpoint_name):
        _log(f"[NAV] Waiting for NavigateToPose action server (up to 15s)...")
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            _log("[ERROR] Nav2 Action Server not available after 15s. Is Nav2 still starting up?")
            _log("[DEBUG] Check: sudo docker compose logs slam | grep -i nav2")
            return
            
        _log(f"[NAV] Sending goal to checkpoint '{checkpoint_name}' at ({pt['x']:.2f}, {pt['y']:.2f})")
        self._current_goal_pt = pt
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pt['x'])
        goal.pose.pose.position.y = float(pt['y'])
        goal.pose.pose.position.z = float(pt['z'])
        goal.pose.pose.orientation.x = float(pt['qx'])
        goal.pose.pose.orientation.y = float(pt['qy'])
        goal.pose.pose.orientation.z = float(pt['qz'])
        goal.pose.pose.orientation.w = float(pt['qw'])
        
        self.send_goal_future = self.nav_client.send_goal_async(goal, feedback_callback=self._nav_feedback_cb)
        self.send_goal_future.add_done_callback(self._nav_goal_response_cb)

    def _nav_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        eta = fb.estimated_time_remaining
        eta_sec = eta.sec + eta.nanosec * 1e-9
        path_dist = getattr(fb, 'distance_remaining', None)
        
        # Calculate real-time physical Euclidean distance to goal using active TF
        euclidean_dist = None
        if hasattr(self, '_current_goal_pt') and self._current_goal_pt:
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                import math
                dx = self._current_goal_pt['x'] - t.transform.translation.x
                dy = self._current_goal_pt['y'] - t.transform.translation.y
                euclidean_dist = math.hypot(dx, dy)
            except Exception:
                pass

        now = time.time()
        if now - self._last_eta_log > 2.0:
            if euclidean_dist is not None:
                dist_str = f" | Dist to goal: {euclidean_dist:.2f}m"
            elif path_dist is not None:
                dist_str = f" | Dist left: {path_dist:.2f}m"
            else:
                dist_str = ""

            if eta_sec > 0:
                _log(f"[NAV] ETA: {eta_sec:.1f}s{dist_str}")
            else:
                _log(f"[NAV] In progress{dist_str}")
            self._last_eta_log = now

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            _log("[NAV] Goal REJECTED by Nav2! Likely causes:")
            _log("[NAV]   1) No map->odom TF (SLAM Toolbox / RF2O not publishing TF?)")
            _log("[NAV]   2) Robot pose unknown")
            _log("[NAV]   3) Nav2 costmap not initialised yet — wait 10s and retry")
            return
        _log("[NAV] Goal ACCEPTED — Nav2 is planning path and moving robot...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        result = future.result().status
        status_str = {4: 'SUCCEEDED ✅', 5: 'CANCELED', 6: 'ABORTED ❌'}.get(result, f'UNKNOWN ({result})')
        _log(f"[NAV] Navigation finished — status: {status_str}")

    def map_callback(self, msg: OccupancyGrid):
        global latest_map_img_bytes, map_received_count, latest_map_time
        latest_map_time = time.time()
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
        clean_name = _sanitize_name(name)
        if not clean_name:
            _log("[ERROR] Invalid checkpoint name.")
            return
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            p = t.transform.translation
            q = t.transform.rotation
            self.checkpoints[clean_name] = {'x': p.x, 'y': p.y, 'z': p.z,
                                            'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w}
            _log(f"[SUCCESS] Checkpoint '{clean_name}' pinned at ({p.x:.2f}, {p.y:.2f})")
        except Exception as e:
            _log(f"[ERROR] Could not record checkpoint: {e}")
            _log("[HINT] TF error? Drive or rotate the robot a little so RF2O builds odom->base_link first.")

    def save_map_and_metadata(self, room_name):
        clean_room = _sanitize_name(room_name)
        if not clean_room:
            _log("[ERROR] Invalid map name for saving.")
            return

        _log(f"[SAVE] Saving map '{clean_room}'...")
        import subprocess
        env = os.environ.copy()
        env['ROS_LOCALHOST_ONLY'] = '1'
        save_success = False

        # Attempt 1: Native SLAM Toolbox save service (fastest, most reliable directly from SLAM memory)
        try:
            res = subprocess.run(
                ["ros2", "service", "call", "/slam_toolbox/save_map",
                 "slam_toolbox/srv/SaveMap", f"{{name: {{data: '/root/{clean_room}_map'}}}}"],
                capture_output=True, text=True, env=env, timeout=15
            )
            if res.returncode == 0 and ("result=0" in res.stdout or "success" in res.stdout.lower() or os.path.exists(f"/root/{clean_room}_map.yaml")):
                _log(f"[SUCCESS] SLAM Toolbox saved map grid to /root/{clean_room}_map.pgm")
                save_success = True
        except Exception as e:
            _log(f"[DEBUG] SLAM Toolbox service save skipped: {e}")

        # Attempt 2: Fallback to nav2_map_server map_saver_cli with timeout allowance
        if not save_success:
            try:
                subprocess.run(
                    ["ros2", "run", "nav2_map_server", "map_saver_cli",
                     "-f", f"/root/{clean_room}_map",
                     "--ros-args", "-p", "map_subscribe_transient_local:=true",
                     "-p", "use_sim_time:=false"],
                    check=True, env=env, timeout=15
                )
                _log(f"[SUCCESS] Map grid saved to /root/{clean_room}_map.pgm")
                save_success = True
            except subprocess.CalledProcessError as e:
                _log(f"[ERROR] map_saver_cli failed (code {e.returncode}).")
            except subprocess.TimeoutExpired:
                _log("[ERROR] map_saver_cli timed out.")

        # ONLY save checkpoints metadata if map saver succeeded to avoid orphaned or mismatched files
        if save_success:
            meta = f"/root/{clean_room}_checkpoints.json"
            with open(meta, 'w') as f:
                json.dump(self.checkpoints, f, indent=4)
            _log(f"[SUCCESS] {len(self.checkpoints)} checkpoints -> {meta}")
        else:
            _log("[WARN] Skipping checkpoints save because map saving failed.")

# ── Command Dispatcher ────────────────────────────────────────────────────────

def _handle_command(node, payload):
    cmd = payload.get('cmd')

    if cmd == 1:
        with map_lock:
            count = map_received_count
        _log(f"[STATUS] /map frames: {count}" + (" — active!" if count > 0 else " — move/turn robot to start SLAM."))

    elif cmd == 2:
        name = str(payload.get('checkpoint_name', '')).strip()
        if name:
            node.record_checkpoint(name)
        else:
            _log("[ERROR] No checkpoint name provided.")

    elif cmd == 3:
        map_name = str(payload.get('map_name', '')).strip()
        if map_name:
            threading.Thread(target=node.save_map_and_metadata, args=(map_name,), daemon=True).start()
        else:
            _log("[ERROR] No map name provided.")

    elif cmd == 4:
        map_name   = _sanitize_name(str(payload.get('map_name', '')).strip())
        checkpoint = _sanitize_name(str(payload.get('checkpoint', '')).strip())
        
        if map_name == 'LIVE':
            saved = node.checkpoints
            if not saved:
                _log("[ERROR] No checkpoints set in current Live session yet."); return
        else:
            # Check if this saved map was created in a previous session
            _log(f"[NAV NOTICE] Navigating using saved map '{map_name}'. Ensure current live mapping origin matches saved map origin, or navigate in the active LIVE session.")
            cp_file = f"/root/{map_name}_checkpoints.json"
            if not os.path.exists(cp_file):
                _log(f"[ERROR] Map '{map_name}' not found at {cp_file}"); return
            with open(cp_file) as f:
                saved = json.load(f)
                
        if checkpoint not in saved:
            _log(f"[ERROR] Checkpoint '{checkpoint}' not found in map '{map_name}'"); return
        
        pt = saved[checkpoint]
        node.send_nav_goal(pt, checkpoint)

    elif cmd == 5:
        _log("[EXIT] Shutting down...")
        rclpy.shutdown()
        sys.exit(0)

    elif cmd == 6:
        # Switch mode / Set active map
        map_name = _sanitize_name(str(payload.get('map_name', '')).strip())
        mode = payload.get('mode', 'localization')
        if not map_name and mode == 'localization':
            _log("[ERROR] No map name specified.")
            return
        
        cfg_path = "/root/default_mode.txt"
        try:
            with open(cfg_path, 'w') as f:
                f.write(f'export NAV_MODE="{mode}"\n')
                if map_name:
                    f.write(f'export ACTIVE_MAP="{map_name}"\n')
            _log(f"[CONFIG] Mode set to: {mode} (Map: {map_name or 'None'})")
            _log("[RESTART] To apply this map across all ROS nodes, run on Jetson terminal: sudo docker compose restart")
        except Exception as e:
            _log(f"[ERROR] Failed to save mode config: {e}")

    elif cmd == 7:
        # Set Initial Pose for AMCL
        x = float(payload.get('x', 0.0))
        y = float(payload.get('y', 0.0))
        yaw = float(payload.get('yaw', 0.0))
        node.set_initial_pose(x, y, yaw)

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
    _log("  Terminal: 1 | 2:name | 3:mapname | 4:map,checkpoint | 6:mapname (load) | 7:x,y | 5=exit")
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
                    elif c == 6:
                        cmd_queue.put({'cmd': 6, 'map_name': arg.strip(), 'mode': 'localization'})
                    elif c == 7:
                        coords = arg.split(',')
                        x = float(coords[0].strip()) if len(coords) > 0 else 0.0
                        y = float(coords[1].strip()) if len(coords) > 1 else 0.0
                        yaw = float(coords[2].strip()) if len(coords) > 2 else 0.0
                        cmd_queue.put({'cmd': 7, 'x': x, 'y': y, 'yaw': yaw})
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
