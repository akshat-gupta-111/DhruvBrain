#!/usr/bin/env python3
# =============================================================================
#  DHRUV NAVIGATION & SLAM — ROBOT MANAGER & TELEMETRY DASHBOARD
#  Web UI Port: 8000 | Multi-Sensor Monitoring | Visual Odometry & LiDAR Tracking
# =============================================================================

import os
import re
import sys
import json
import time
import html
import threading
import queue
import math
from io import BytesIO
import yaml
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import cv2

import rclpy
import rclpy.duration
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, Imu, Image as RosImage
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

# ── Globals ──────────────────────────────────────────────────────────────────
latest_map_img_bytes = b""
map_lock = threading.Lock()
map_received_count = 0

latest_vo_img_bytes = b""
vo_img_lock = threading.Lock()
latest_vo_vx = 0.0
latest_vo_wz = 0.0

# Diagnostics freshness tracking
latest_scan_time = 0.0
latest_odom_time = 0.0
latest_map_time = 0.0
latest_imu_time = 0.0
latest_vo_time = 0.0

cmd_queue = queue.Queue()

_log_lock = threading.Lock()
_log_lines = []
MAX_LOG = 120

_node_ref = None
SAVE_DIR = '/root'

_MIME = {
    '.pgm':  'image/x-portable-graymap',
    '.yaml': 'text/yaml',
    '.json': 'application/json',
    '.png':  'image/png',
    '.jpg':  'image/jpeg',
}

def _log(msg: str):
    """Print to stdout AND append to the web live-log panel."""
    print(msg, flush=True)
    ts = time.strftime('%H:%M:%S')
    with _log_lock:
        _log_lines.append(f"[{ts}] {msg}")
        if len(_log_lines) > MAX_LOG:
            _log_lines.pop(0)

def _sanitize_name(name: str) -> str:
    clean = re.sub(r'[^a-zA-Z0-9_\-]', '', name)
    return clean.strip()

def _make_placeholder_png(width: int = 480, height: int = 240, text: str = "Waiting for /map data...") -> bytes:
    img = Image.new('RGB', (width, height), color=(20, 24, 34))
    draw = ImageDraw.Draw(img)
    draw.text((width // 2, height // 2 - 15), text, fill=(180, 190, 210), anchor='mm')
    draw.text((width // 2, height // 2 + 15), "Rotate or drive robot slightly to build initial scan.", fill=(110, 120, 140), anchor='mm')
    buf = BytesIO()
    img.save(buf, format='PNG')
    return buf.getvalue()

PLACEHOLDER_PNG = _make_placeholder_png(480, 240, "Waiting for /map data...")
PLACEHOLDER_VO_PNG = _make_placeholder_png(400, 240, "Waiting for Camera / VO Stream...")

def _list_saved_maps():
    sets = {}
    try:
        if os.path.exists(SAVE_DIR):
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
        pass

    def do_GET(self):
        global map_received_count, latest_scan_time, latest_odom_time, latest_map_time, latest_imu_time, latest_vo_time
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)

        # ── / ── main dashboard ───────────────────────────────────────────────
        if path == '/':
            with map_lock:
                has_map = bool(latest_map_img_bytes)
                count = map_received_count

            saved = _list_saved_maps()
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

            dl_rows = ''
            if saved:
                for map_name, files in sorted(saved.items()):
                    esc_name = html.escape(map_name)
                    btn_pgm  = f'<a class="btn" href="/download?file={esc_name}_map.pgm">&#11123; .pgm</a>'  if files.get('pgm')  else '<span class="btn disabled">.pgm</span>'
                    btn_yaml = f'<a class="btn" href="/download?file={esc_name}_map.yaml">&#11123; .yaml</a>' if files.get('yaml') else '<span class="btn disabled">.yaml</span>'
                    btn_json = f'<a class="btn" href="/download?file={esc_name}_checkpoints.json">&#11123; .json</a>' if files.get('json') else '<span class="btn disabled">.json</span>'
                    has_all  = files.get('pgm') and files.get('yaml') and files.get('json')
                    btn_anno = f'<a class="btn" href="/annotated.png?map={esc_name}">&#128444; Annotated</a>' if has_all else '<span class="btn disabled">Annotated</span>'
                    
                    is_active = (current_active_map == map_name)
                    btn_load = f'<span class="badge-ok" style="margin:0">Active</span> <button class="btn" style="background:#555;cursor:pointer;border:none;margin-left:4px" onclick="deselectMap()">&#10005; Deselect</button>' if is_active else f'<button class="btn" style="background:#7b2cbf;cursor:pointer;border:none" onclick="loadMapMode(\'{esc_name}\')">&#128640; Load Map</button>'
                    btn_del  = f'<button class="btn" style="background:#b7094c;cursor:pointer;border:none" onclick="deleteMap(\'{esc_name}\')">&#128465; Delete</button>'
                    
                    dl_rows += f'<tr><td class="mapname">{esc_name}</td><td>{btn_load}</td><td>{btn_pgm}</td><td>{btn_yaml}</td><td>{btn_json}</td><td>{btn_anno}</td><td>{btn_del}</td></tr>'

            downloads_html = f"""
  <div class="card">
    <h2>&#128190; Saved Maps</h2>
    <table>
      <thead><tr><th>Map Name</th><th>Action</th><th>.pgm</th><th>.yaml</th><th>.json</th><th>Annotated</th><th>Delete</th></tr></thead>
      <tbody>{dl_rows}</tbody>
    </table>
  </div>""" if saved else '<div class="card"><p style="color:#667">No saved maps yet. Use the Save Map button to preserve current session.</p></div>'

            map_opts = '<option value="LIVE">Live Map (Current Session)</option>'
            for m in sorted(saved.keys()):
                esc_m = html.escape(m)
                map_opts += f'<option value="{esc_m}">{esc_m}</option>'

            status_cls = 'badge-ok' if has_map else 'badge-wait'
            status_txt = f'&#10004; {count} map frames received' if has_map else '&#9899; Waiting for SLAM map &mdash; rotate/nudge robot'

            with _log_lock:
                log_html = '\n'.join(f'<div class="ll">{html.escape(l)}</div>' for l in reversed(_log_lines)) or '<div class="ll" style="color:#555">Starting navigation telemetry...</div>'

            html_page = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>DHRUV SLAM &mdash; Autonomous Navigation Dashboard</title>
  <style>
    *{{box-sizing:border-box;margin:0;padding:0;}}
    body{{background:#080a12;color:#d5d9e5;font-family:'Segoe UI',system-ui,sans-serif;padding:16px;}}
    h1{{color:#00b4d8;font-size:1.4rem;text-align:center;margin-bottom:2px;}}
    .sub{{text-align:center;color:#606a85;font-size:.8rem;margin-bottom:14px;}}
    .tele-bar{{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:10px;margin:0 auto 14px;max-width:1200px;}}
    .tele-item{{background:#101726;border:1px solid #1c2842;border-radius:9px;padding:10px 14px;text-align:center;}}
    .tele-val{{font-size:1.15rem;font-weight:700;color:#00d2ff;margin-top:3px;}}
    .tele-lbl{{font-size:.72rem;color:#7e8ba8;text-transform:uppercase;letter-spacing:.5px;}}
    .layout{{display:grid;grid-template-columns:1.1fr 0.9fr;gap:14px;max-width:1200px;margin:0 auto;}}
    .card{{background:#101726;border:1px solid #1c2842;border-radius:12px;padding:14px;margin-bottom:14px;}}
    .card h2{{color:#7ecfef;font-size:.92rem;margin-bottom:10px;display:flex;align-items:center;justify-content:space-between;}}
    .mapwrap{{text-align:center;background:#05070d;border-radius:8px;padding:4px;border:1px solid #1a253d;}}
    .mapwrap img{{max-width:100%;max-height:48vh;border-radius:6px;display:block;margin:0 auto;}}
    .badge-ok{{display:inline-block;background:#0f2e10;color:#4caf50;border-radius:20px;padding:3px 12px;font-size:.74rem;margin-bottom:8px;}}
    .badge-wait{{display:inline-block;background:#2e1e08;color:#ff9800;border-radius:20px;padding:3px 12px;font-size:.74rem;margin-bottom:8px;}}
    .cmd-grid{{display:grid;grid-template-columns:1fr 1fr;gap:8px;}}
    .cbtn{{padding:11px 8px;border:none;border-radius:8px;font-size:.85rem;font-weight:700;cursor:pointer;
            display:flex;flex-direction:column;align-items:center;gap:2px;transition:filter .15s;}}
    .cbtn:hover{{filter:brightness(1.25);}}
    .cbtn .ic{{font-size:1.3rem;}}
    .cbtn .lb{{font-size:.68rem;font-weight:400;opacity:.75;}}
    .c1{{background:#0c4272;color:#fff;}}
    .c2{{background:#0d4a22;color:#fff;}}
    .c3{{background:#4a2c00;color:#fff;}}
    .c4{{background:#3a0858;color:#fff;}}
    .btn{{display:inline-block;padding:5px 11px;border-radius:6px;background:#0077b6;color:#fff;
          text-decoration:none;font-size:.76rem;margin:2px;transition:background .2s;}}
    .btn:hover{{background:#0096c7;}}
    .btn.disabled{{background:#1a2640;color:#556;cursor:default;pointer-events:none;}}
    .logbox{{background:#04050a;border-radius:8px;padding:10px;height:210px;overflow-y:auto;
              font-family:monospace;font-size:.75rem;line-height:1.6;border:1px solid #161e30;}}
    .ll{{color:#5eadc8;border-bottom:1px solid #0c101c;padding:1px 0;}}
    table{{width:100%;border-collapse:collapse;font-size:.8rem;}}
    th{{color:#7ecfef;border-bottom:1px solid #1c2842;padding:6px 8px;text-align:left;}}
    td{{padding:6px 8px;border-bottom:1px solid #131c30;}}
    td.mapname{{font-weight:700;color:#a3b2d1;}}
    .mb{{display:none;position:fixed;inset:0;background:rgba(0,0,0,.75);z-index:100;
          align-items:center;justify-content:center;}}
    .mb.open{{display:flex;}}
    .md{{background:#101726;border:1px solid #1c2842;border-radius:14px;
          padding:24px;min-width:300px;max-width:420px;width:90%;}}
    .md h3{{color:#7ecfef;margin-bottom:12px;font-size:.95rem;}}
    .md input,.md select{{width:100%;padding:9px 12px;border-radius:7px;
          border:1px solid #1c2842;background:#080a12;color:#ccd;font-size:.88rem;margin-bottom:11px;}}
    .mbtns{{display:flex;gap:9px;justify-content:flex-end;}}
    .mbtns button{{padding:8px 18px;border:none;border-radius:7px;cursor:pointer;font-size:.85rem;font-weight:600;}}
    .bcancel{{background:#1a2640;color:#778;}}.bok{{background:#0096c7;color:#fff;}}
    @media(max-width:820px){{.layout{{grid-template-columns:1fr;}}}}
  </style>
</head>
<body>
  <h1>&#129302; DHRUV AUTONOMOUS SLAM &mdash; CONTROL CENTER</h1>
  <p class="sub">Multi-Sensor Navigation &nbsp;|&nbsp; Port 8000 &nbsp;|&nbsp; <a href="/status" style="color:#7ecfef">Diagnostics JSON</a></p>

  <!-- Live Telemetry Ticker -->
  <div class="tele-bar">
    <div class="tele-item"><div class="tele-lbl">Robot Pose (X, Y)</div><div class="tele-val" id="t-pose">0.00m, 0.00m</div></div>
    <div class="tele-item"><div class="tele-lbl">Heading (Yaw)</div><div class="tele-val" id="t-yaw">0.0°</div></div>
    <div class="tele-item"><div class="tele-lbl">Fwd Obstacle</div><div class="tele-val" id="t-obst">>5.0m</div></div>
    <div class="tele-item"><div class="tele-lbl">Visual Odometry</div><div class="tele-val" id="t-vo">0.00 m/s</div></div>
    <div class="tele-item"><div class="tele-lbl">Active Pins</div><div class="tele-val" id="t-pins">0</div></div>
  </div>

  <div class="layout">
    <div>
      <div class="card">
        <h2>&#128247; Live Occupancy Grid Map <span id="map-counter" style="font-size:0.75rem;color:#6f82a8"></span></h2>
        <span class="{status_cls}">{status_txt}</span>
        <div class="mapwrap"><img id="mapimg" src="/map.png" alt="SLAM map"></div>
        <div style="text-align:center;margin-top:6px">
          <a class="btn" href="/map.png" target="_blank">&#11123; Full Resolution Map</a>
        </div>
      </div>

      <div class="card">
        <h2>📷 Monocular Camera &amp; Visual Tracking (Optical Flow)</h2>
        <div class="mapwrap"><img id="voimg" src="/camera.png" alt="VO camera feed" style="max-height:220px"></div>
        <div style="font-size:0.75rem;color:#7ecfef;margin-top:5px;text-align:center" id="vo-telemetry-text">
          Features tracked via Pyramidal Lucas-Kanade + RANSAC
        </div>
      </div>

      <div class="card">
        <h2>&#127918; Navigation Commands</h2>
        <div class="cmd-grid">
          <button class="cbtn c1" onclick="runCmd(1)"><span class="ic">&#128205;</span>1 &mdash; Status<span class="lb">Print Telemetry</span></button>
          <button class="cbtn c2" onclick="showModal('m-cp')"><span class="ic">&#128204;</span>2 &mdash; Set Checkpoint<span class="lb">Drop Named Pin</span></button>
          <button class="cbtn c3" onclick="showModal('m-save')"><span class="ic">&#128190;</span>3 &mdash; Save Map<span class="lb">Save Map + Pins</span></button>
          <button class="cbtn c4" onclick="showModal('m-nav')"><span class="ic">&#128663;</span>4 &mdash; Navigate To<span class="lb">Autonomous Drive</span></button>
          <button class="cbtn" style="background:#b7094c;color:#fff;grid-column:span 2" onclick="cancelNav()"><span class="ic">&#128721;</span>0 &mdash; Immediate Stop / Cancel<span class="lb">Halt robot immediately</span></button>
        </div>
      </div>
    </div>

    <div>
      <!-- Live Checkpoints Table with Real-time Distances -->
      <div class="card">
        <h2>📍 Active Checkpoints &amp; Distances <span id="cp-count-badge" class="badge-ok" style="margin:0;font-size:0.7rem">0 Pins</span></h2>
        <table id="cp-table">
          <thead><tr><th>Name</th><th>Position (X, Y)</th><th>Distance from Robot</th><th>Action</th></tr></thead>
          <tbody id="cp-rows"><tr><td colspan="4" style="color:#667">No checkpoints set yet. Click 'Set Checkpoint' to drop a pin.</td></tr></tbody>
        </table>
      </div>

      <div class="card">
        <h2>&#128220; Live System &amp; Sensor Log</h2>
        <div class="logbox" id="logbox">{log_html}</div>
      </div>

      {downloads_html}
    </div>
  </div>

  <!-- Modals -->
  <div class="mb" id="m-cp">
    <div class="md">
      <h3>&#128204; Set Named Checkpoint</h3>
      <input id="cp-name" type="text" placeholder="e.g. desk_area, entrance, lab" autofocus>
      <div class="mbtns">
        <button class="bcancel" onclick="hideModal('m-cp')">Cancel</button>
        <button class="bok" onclick="doCheckpoint()">Save Pin</button>
      </div>
    </div>
  </div>

  <div class="mb" id="m-save">
    <div class="md">
      <h3>&#128190; Save Map &amp; Checkpoints</h3>
      <input id="save-name" type="text" placeholder="e.g. office_floor1">
      <div class="mbtns">
        <button class="bcancel" onclick="hideModal('m-save')">Cancel</button>
        <button class="bok" onclick="doSave()">Save</button>
      </div>
    </div>
  </div>

  <div class="mb" id="m-nav">
    <div class="md">
      <h3>&#128663; Navigate to Checkpoint</h3>
      <select id="nav-map" onchange="loadCPs()"><option value="" disabled selected>-- Select Map --</option>{map_opts}</select>
      <select id="nav-cp"><option value="">-- select map first --</option></select>
      <div class="mbtns">
        <button class="bcancel" onclick="hideModal('m-nav')">Cancel</button>
        <button class="bok" onclick="doNav()">Go!</button>
      </div>
    </div>
  </div>

  <script>
    // Refresh map every 1.5 seconds
    setInterval(() => {{
      const img = document.getElementById('mapimg');
      if(img) img.src = '/map.png?t=' + Date.now();
    }}, 1500);

    // Refresh VO video feed every 400ms (~2.5 FPS preview)
    setInterval(() => {{
      const vimg = document.getElementById('voimg');
      if(vimg) vimg.src = '/camera.png?t=' + Date.now();
    }}, 400);

    // Refresh live logs every 3 seconds
    setInterval(() => {{
      fetch('/api/log').then(r => r.text()).then(h => {{
        const b = document.getElementById('logbox');
        if(b) b.innerHTML = h;
      }});
    }}, 3000);

    // Fast Telemetry Polling (every 1 second)
    setInterval(() => {{
      fetch('/api/telemetry').then(r => r.json()).then(d => {{
        if(d.pose && d.pose.x !== null) {{
          document.getElementById('t-pose').innerText = d.pose.x.toFixed(2) + 'm, ' + d.pose.y.toFixed(2) + 'm';
          document.getElementById('t-yaw').innerText = d.pose.yaw_deg.toFixed(1) + '°';
        }}
        if(d.fwd_obstacle_dist !== null) {{
          document.getElementById('t-obst').innerText = d.fwd_obstacle_dist.toFixed(2) + 'm';
        }}
        if(d.vo) {{
          document.getElementById('t-vo').innerText = d.vo.vx.toFixed(2) + ' m/s';
          document.getElementById('vo-telemetry-text').innerText = 'VO Linear Vx: ' + d.vo.vx.toFixed(2) + ' m/s | Yaw Rate: ' + d.vo.wz.toFixed(2) + ' rad/s | ' + (d.vo.active ? 'ACTIVE' : 'IDLE');
        }}

        // Render Checkpoints Table
        if(d.checkpoints) {{
          document.getElementById('t-pins').innerText = d.checkpoints.length;
          document.getElementById('cp-count-badge').innerText = d.checkpoints.length + ' Pins';
          const tbody = document.getElementById('cp-rows');
          if(d.checkpoints.length === 0) {{
            tbody.innerHTML = '<tr><td colspan="4" style="color:#667">No checkpoints set yet.</td></tr>';
          }} else {{
            tbody.innerHTML = d.checkpoints.map(cp => {{
              const distStr = cp.distance !== null ? `<b>${{cp.distance.toFixed(2)}} m</b>` : '--';
              return `<tr>
                <td style="font-weight:700;color:#00d2ff">📍 ${{cp.name}}</td>
                <td>(${{cp.x.toFixed(2)}}, ${{cp.y.toFixed(2)}})</td>
                <td>${{distStr}}</td>
                <td>
                  <button class="btn" style="background:#0d4a22;cursor:pointer;border:none" onclick="navDirect('${{cp.name}}')">&#128663; Go</button>
                  <button class="btn" style="background:#555;cursor:pointer;border:none" onclick="delCheckpoint('${{cp.name}}')">&#10005;</button>
                </td>
              </tr>`;
            }}).join('');
          }}
        }}
      }}).catch(() => {{}});
    }}, 1000);

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
    function cancelNav() {{ post({{cmd: 0}}); }}

    function doCheckpoint() {{
      const name = document.getElementById('cp-name').value.trim();
      if(!name) return;
      post({{cmd: 2, checkpoint_name: name}});
      hideModal('m-cp');
      document.getElementById('cp-name').value = '';
    }}

    function delCheckpoint(name) {{
      post({{cmd: 9, checkpoint_name: name}});
    }}

    function navDirect(cpName) {{
      post({{cmd: 4, map_name: 'LIVE', checkpoint: cpName}});
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
      post({{cmd: 6, map_name: mapName}}).then(() => setTimeout(() => location.reload(), 1200));
    }}

    function deselectMap() {{
      post({{cmd: 6, map_name: 'LIVE'}}).then(() => setTimeout(() => location.reload(), 800));
    }}

    function deleteMap(mapName) {{
      if(!confirm('Are you sure you want to delete map \"' + mapName + '\"?')) return;
      post({{cmd: 8, map_name: mapName}}).then(() => setTimeout(() => location.reload(), 800));
    }}
  </script>
</body>
</html>"""
            self._send(200, 'text/html; charset=utf-8', html_page.encode())

        elif path == '/map.png':
            with map_lock:
                img_data = latest_map_img_bytes
            self._send(200, 'image/png', img_data if img_data else PLACEHOLDER_PNG)

        elif path == '/camera.png':
            with vo_img_lock:
                cdata = latest_vo_img_bytes
            self._send(200, 'image/jpeg', cdata if cdata else PLACEHOLDER_VO_PNG)

        elif path == '/api/telemetry':
            cps_info = []
            cur_pose = None
            fwd_obst = 999.0
            if _node_ref is not None:
                cur_pose = _node_ref._get_robot_pose()
                fwd_obst = _node_ref._fwd_obstacle_dist
                for cname, cdata in _node_ref.checkpoints.items():
                    dist_m = None
                    if cur_pose is not None:
                        dist_m = round(math.sqrt((cur_pose[0] - cdata['x'])**2 + (cur_pose[1] - cdata['y'])**2), 2)
                    cps_info.append({
                        'name': cname,
                        'x': round(cdata['x'], 2),
                        'y': round(cdata['y'], 2),
                        'distance': dist_m
                    })

            telemetry_data = {
                'pose': {
                    'x': round(cur_pose[0], 2) if cur_pose else 0.0,
                    'y': round(cur_pose[1], 2) if cur_pose else 0.0,
                    'yaw_deg': round(math.degrees(cur_pose[2]), 1) if cur_pose else 0.0
                },
                'fwd_obstacle_dist': round(fwd_obst, 2) if fwd_obst < 50.0 else 10.0,
                'vo': {
                    'vx': round(latest_vo_vx, 2),
                    'wz': round(latest_vo_wz, 2),
                    'active': (time.time() - latest_vo_time < 3.0) and (latest_vo_time > 0)
                },
                'checkpoints': cps_info
            }
            self._send(200, 'application/json', json.dumps(telemetry_data).encode())

        elif path == '/api/log':
            with _log_lock:
                log_html = '\n'.join(f'<div class="ll">{html.escape(l)}</div>' for l in reversed(_log_lines))
            self._send(200, 'text/html; charset=utf-8', log_html.encode())

        elif path == '/api/checkpoints':
            m = qs.get('map', [''])[0]
            pts = []
            if m == 'LIVE':
                if _node_ref is not None:
                    pts = list(_node_ref.checkpoints.keys())
            elif m:
                cp_file = f"/root/{m}_checkpoints.json"
                if os.path.isfile(cp_file):
                    try:
                        with open(cp_file) as f:
                            pts = list(json.load(f).keys())
                    except Exception:
                        pass
            self._send(200, 'application/json', json.dumps(pts).encode())

        elif path == '/status':
            now = time.time()
            with map_lock:
                count = map_received_count
            scan_fresh = (now - latest_scan_time < 3.0) and (latest_scan_time > 0)
            odom_fresh = (now - latest_odom_time < 3.0) and (latest_odom_time > 0)
            map_fresh  = (now - latest_map_time < 10.0) and (latest_map_time > 0)
            imu_fresh  = (now - latest_imu_time < 3.0) and (latest_imu_time > 0)
            vo_fresh   = (now - latest_vo_time < 3.0) and (latest_vo_time > 0)

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
                'imu_live': imu_fresh,
                'vo_live': vo_fresh,
                'tf_map_to_base_live': tf_available,
                'seconds_since_scan': round(now - latest_scan_time, 1) if latest_scan_time > 0 else None,
                'seconds_since_odom': round(now - latest_odom_time, 1) if latest_odom_time > 0 else None,
                'seconds_since_imu': round(now - latest_imu_time, 1) if latest_imu_time > 0 else None,
                'seconds_since_vo': round(now - latest_vo_time, 1) if latest_vo_time > 0 else None,
                'seconds_since_map': round(now - latest_map_time, 1) if latest_map_time > 0 else None,
            }
            self._send(200, 'application/json', json.dumps(status_payload, indent=2).encode())

        elif path == '/download':
            filename = qs.get('file', [''])[0]
            clean = _sanitize_name(filename.replace('.', ''))
            ext = os.path.splitext(filename)[1]
            filepath = os.path.join(SAVE_DIR, filename)
            if os.path.isfile(filepath):
                with open(filepath, 'rb') as f:
                    data = f.read()
                ctype = _MIME.get(ext, 'application/octet-stream')
                self.send_response(200)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self._send(404, 'text/plain', b'File not found')
        else:
            self._send(404, 'text/plain', b'Not found')

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path != '/api/command':
            self._send(404, 'text/plain', b'Not found')
            return
        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length)
        try:
            payload = json.loads(body)
            if not isinstance(payload, dict) or 'cmd' not in payload:
                self._send(400, 'application/json', b'{"error":"invalid command payload"}')
                return
        except Exception:
            self._send(400, 'application/json', b'{"error":"bad json"}')
            return

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
        self.bridge = CvBridge() if CvBridge is not None else None

        self.last_map_msg = None
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_filtered_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.vo_sub = self.create_subscription(Odometry, '/vo/odom', self.vo_odom_callback, 10)
        self.vo_img_sub = self.create_subscription(RosImage, '/vo/debug_image', self.vo_image_callback, 2)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
        self._fwd_obstacle_dist = 999.0

        self.checkpoints = {}
        # Load saved checkpoints if active map is set
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
                        _log(f"[INIT] Loaded {len(self.checkpoints)} checkpoints from map '{active_m}'")
            except Exception as e:
                _log(f"[WARN] Failed to auto-load checkpoints: {e}")

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._active_nav_goal = None

        # Telemetry Ticker Timer (Logs live status every 2.5 seconds)
        self.telemetry_timer = self.create_timer(2.5, self.telemetry_ticker)

    def _scan_callback(self, msg: LaserScan):
        global latest_scan_time
        latest_scan_time = time.time()
        num = len(msg.ranges)
        if num == 0:
            return
        step = (msg.angle_max - msg.angle_min) / num
        arc_half = math.radians(30)
        fwd_dists = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * step
            if abs(angle) <= arc_half:
                if msg.range_min < r < msg.range_max:
                    fwd_dists.append(r)
        self._fwd_obstacle_dist = min(fwd_dists) if fwd_dists else 999.0

    def odom_callback(self, msg: Odometry):
        global latest_odom_time
        latest_odom_time = time.time()

    def imu_callback(self, msg: Imu):
        global latest_imu_time
        latest_imu_time = time.time()

    def vo_odom_callback(self, msg: Odometry):
        global latest_vo_time, latest_vo_vx, latest_vo_wz
        latest_vo_time = time.time()
        latest_vo_vx = msg.twist.twist.linear.x
        latest_vo_wz = msg.twist.twist.angular.z

    def vo_image_callback(self, msg: RosImage):
        global latest_vo_img_bytes
        try:
            if self.bridge is not None:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
            ret, buf = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if ret:
                with vo_img_lock:
                    latest_vo_img_bytes = buf.tobytes()
        except Exception:
            pass

    def _get_robot_pose(self):
        """Returns (x, y, theta) in map frame or None."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.08)
            )
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            q  = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return (tx, ty, math.atan2(siny, cosy))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def map_callback(self, msg: OccupancyGrid):
        self.last_map_msg = msg
        self.render_map_image(msg)

    def render_map_image(self, msg: OccupancyGrid = None):
        """Renders map image with prominent glowing checkpoint pins and robot heading pointer."""
        global latest_map_img_bytes, map_received_count, latest_map_time
        if msg is None:
            msg = self.last_map_msg
        if msg is None:
            return

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
                pix[x, h - 1 - y] = (255, 255, 255) if v == 0 else (0, 0, 0) if v == 100 else (120, 120, 125)

        draw = ImageDraw.Draw(img)
        res = msg.info.resolution
        ox  = msg.info.origin.position.x
        oy  = msg.info.origin.position.y

        try:
            font = ImageFont.load_default()
        except Exception:
            font = None

        cur_pose = self._get_robot_pose()

        # Draw Prominent Checkpoint Pins
        for name, pt in self.checkpoints.items():
            px = int((pt['x'] - ox) / res)
            py = int(h - 1 - ((pt['y'] - oy) / res))

            dist_str = ""
            if cur_pose is not None:
                d = math.sqrt((cur_pose[0] - pt['x'])**2 + (cur_pose[1] - pt['y'])**2)
                dist_str = f" ({d:.1f}m)"

            # Outer prominent pin halo
            draw.ellipse((px-8, py-8, px+8, py+8), fill=(255, 40, 40), outline=(255, 255, 255))
            # Inner bright yellow core
            draw.ellipse((px-3, py-3, px+3, py+3), fill=(255, 230, 0))

            # Checkpoint name label with contrast shadow
            label = f"[{name}]{dist_str}"
            if font:
                draw.text((px + 10, py - 6), label, fill=(0, 0, 0), font=font)
                draw.text((px + 9, py - 7), label, fill=(255, 255, 0), font=font)

        # Draw Robot Position & Heading Pointer
        if cur_pose is not None:
            rx, ry, r_yaw = cur_pose
            r_px = int((rx - ox) / res)
            r_py = int(h - 1 - ((ry - oy) / res))

            # Robot center circle (cyan with white border)
            draw.ellipse((r_px-7, r_py-7, r_px+7, r_py+7), fill=(0, 190, 255), outline=(255, 255, 255))
            # Directional Heading Pointer Line (pointing along robot yaw)
            pointer_len = 16
            ax = r_px + int(pointer_len * math.cos(r_yaw))
            ay = r_py - int(pointer_len * math.sin(r_yaw))
            draw.line((r_px, r_py, ax, ay), fill=(255, 255, 0), width=3)

        buf = BytesIO()
        img.save(buf, format='PNG')
        with map_lock:
            latest_map_img_bytes = buf.getvalue()

    def record_checkpoint(self, name):
        clean_name = _sanitize_name(name)
        if not clean_name:
            _log("[ERROR] Invalid checkpoint name.")
            return

        pose = self._get_robot_pose()
        if pose is not None:
            tx, ty, yaw = pose
            t = None
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                q = t.transform.rotation
                qx, qy, qz, qw = q.x, q.y, q.z, q.w
            except Exception:
                qx, qy, qz, qw = 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

            self.checkpoints[clean_name] = {'x': tx, 'y': ty, 'z': 0.0,
                                            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}
            _log(f"[SUCCESS] 📍 Checkpoint '{clean_name}' PINNED at (X={tx:.2f}m, Y={ty:.2f}m, Yaw={math.degrees(yaw):.1f}°). Distance: 0.00m.")
            # Immediately refresh map so pin appears instantly on browser
            self.render_map_image()
            self._print_all_checkpoints()
        else:
            _log(f"[ERROR] Could not record checkpoint: 'map' -> 'base_link' TF is not ready!")
            _log("[TROUBLESHOOTING] Make sure RF2O is receiving /scan. Drive or rotate the robot 10-20 cm so SLAM registers initial scans.")

    def delete_checkpoint(self, name):
        clean_name = _sanitize_name(name)
        if clean_name in self.checkpoints:
            del self.checkpoints[clean_name]
            _log(f"[SUCCESS] Deleted checkpoint '{clean_name}'. Remaining: {len(self.checkpoints)}")
            self.render_map_image()
        else:
            _log(f"[WARN] Checkpoint '{clean_name}' not found.")

    def _print_all_checkpoints(self):
        cur_pose = self._get_robot_pose()
        _log(f"[CHECKPOINTS] Total Active Pins: {len(self.checkpoints)}")
        for cname, pt in self.checkpoints.items():
            dist_str = ""
            if cur_pose is not None:
                d = math.sqrt((cur_pose[0] - pt['x'])**2 + (cur_pose[1] - pt['y'])**2)
                dist_str = f" | Distance from Dhruv: {d:.2f}m"
            _log(f"   -> 📍 {cname}: (X={pt['x']:.2f}m, Y={pt['y']:.2f}m{dist_str})")

    def telemetry_ticker(self):
        """Prints readable live telemetry to console every 2.5 seconds."""
        cur_pose = self._get_robot_pose()
        now = time.time()
        scan_ok = "ONLINE" if (now - latest_scan_time < 3.0 and latest_scan_time > 0) else "OFFLINE"
        ekf_ok  = "ONLINE" if (now - latest_odom_time < 3.0 and latest_odom_time > 0) else "OFFLINE"
        vo_ok   = f"ONLINE (vx={latest_vo_vx:+.2f}m/s)" if (now - latest_vo_time < 3.0 and latest_vo_time > 0) else "IDLE"
        imu_ok  = "ONLINE" if (now - latest_imu_time < 3.0 and latest_imu_time > 0) else "PASSIVE"

        if cur_pose is not None:
            x, y, yaw = cur_pose
            fwd_d = f"{self._fwd_obstacle_dist:.2f}m" if self._fwd_obstacle_dist < 50.0 else ">10m"
            cp_summary = f"{len(self.checkpoints)} set"
            if self.checkpoints:
                nearest = min(
                    [(name, math.sqrt((x - pt['x'])**2 + (y - pt['y'])**2)) for name, pt in self.checkpoints.items()],
                    key=lambda t: t[1]
                )
                cp_summary = f"Nearest: '{nearest[0]}' ({nearest[1]:.2f}m)"

            _log(f"[LIVE TELEMETRY] Pose: (X={x:+.2f}m, Y={y:+.2f}m, Yaw={math.degrees(yaw):+.1f}°) | Fwd Obstacle: {fwd_d} | VO: {vo_ok} | Pins: {cp_summary}")

    def send_nav_goal(self, pt, checkpoint_name):
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            _log("[ERROR] Nav2 Action Server 'navigate_to_pose' is not responding.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(pt['x'])
        goal_msg.pose.pose.position.y = float(pt['y'])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = float(pt.get('qx', 0.0))
        goal_msg.pose.pose.orientation.y = float(pt.get('qy', 0.0))
        goal_msg.pose.pose.orientation.z = float(pt.get('qz', 0.0))
        goal_msg.pose.pose.orientation.w = float(pt.get('qw', 1.0))

        cur_pose = self._get_robot_pose()
        dist_str = ""
        if cur_pose is not None:
            d = math.sqrt((cur_pose[0] - pt['x'])**2 + (cur_pose[1] - pt['y'])**2)
            dist_str = f" | Total travel distance: {d:.2f}m"

        _log(f"[NAV] Dispatched goal to '{checkpoint_name}' at ({pt['x']:.2f}, {pt['y']:.2f}){dist_str}")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda f: self._goal_response_callback(f, checkpoint_name))

    def _goal_response_callback(self, future, cp_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            _log(f"[NAV] Goal to '{cp_name}' was REJECTED by Nav2 planner.")
            return
        _log(f"[NAV] Goal to '{cp_name}' ACCEPTED. Nav2 is driving robot...")
        self._active_nav_goal = goal_handle
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        try:
            status = future.result().status
        except Exception:
            status = -1
        self._active_nav_goal = None
        status_map = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
        _log(f"[NAV] Navigation completed with status: {status_map.get(status, f'Code {status}')}")

    def cancel_goal(self):
        if self._active_nav_goal:
            _log("[NAV] Sending cancel request to Nav2...")
            self._active_nav_goal.cancel_goal_async()
            self._active_nav_goal = None
        # Send emergency STOP to cmd_vel
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        _log("[STOP] Published zero velocity to /cmd_vel.")

    def save_map_and_metadata(self, room_name):
        clean_room = _sanitize_name(room_name)
        if not clean_room:
            _log("[ERROR] Invalid map name for saving.")
            return

        _log(f"[SAVE] Saving map '{clean_room}'...")
        import subprocess
        env = os.environ.copy()
        env['ROS_LOCALHOST_ONLY'] = '1'

        try:
            res = subprocess.run(
                ["ros2", "service", "call", "/slam_toolbox/save_map",
                 "slam_toolbox/srv/SaveMap", f"{{name: {{data: '/root/{clean_room}_map'}}}}"],
                capture_output=True, text=True, env=env, timeout=15
            )
            if res.returncode == 0:
                _log(f"[SUCCESS] Saved map grid to /root/{clean_room}_map.pgm & .yaml")
        except Exception as e:
            _log(f"[WARN] SLAM Toolbox save service call failed: {e}")

        meta = f"/root/{clean_room}_checkpoints.json"
        with open(meta, 'w') as f:
            json.dump(self.checkpoints, f, indent=4)
        _log(f"[SUCCESS] Saved {len(self.checkpoints)} checkpoints -> {meta}")

# ── Command Dispatcher ────────────────────────────────────────────────────────

def _handle_command(node: RobotManagerNode, payload):
    cmd = payload.get('cmd')

    if cmd in (0, 'cancel', 'stop'):
        node.cancel_goal()

    elif cmd == 1:
        cur_pose = node._get_robot_pose()
        now = time.time()
        with map_lock:
            count = map_received_count
        _log("=" * 60)
        _log("  DHRUV NAVIGATION & SENSOR STATUS REPORT")
        _log("=" * 60)
        if cur_pose is not None:
            _log(f"  Robot Pose:   X={cur_pose[0]:+.2f}m, Y={cur_pose[1]:+.2f}m, Yaw={math.degrees(cur_pose[2]):+.1f}°")
        else:
            _log("  Robot Pose:   [WAITING] map -> base_link transform not built yet.")
        _log(f"  Fwd Obstacle: {node._fwd_obstacle_dist:.2f}m" if node._fwd_obstacle_dist < 50.0 else "  Fwd Obstacle: Clear (>10m)")
        _log(f"  RPLiDAR:      {'ONLINE' if now - latest_scan_time < 3.0 else 'NO DATA'}")
        _log(f"  EKF Odom:     {'ONLINE' if now - latest_odom_time < 3.0 else 'NO DATA'}")
        _log(f"  Visual Odom:  {'ONLINE (vx=' + str(round(latest_vo_vx, 2)) + ' m/s)' if now - latest_vo_time < 3.0 else 'IDLE'}")
        _log(f"  6-Axis IMU:   {'ONLINE' if now - latest_imu_time < 3.0 else 'PASSIVE (Not wired)'}")
        _log(f"  SLAM Map:     {count} frames received.")
        node._print_all_checkpoints()
        _log("=" * 60)

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
        saved = node.checkpoints
        if map_name != 'LIVE':
            cp_file = f"/root/{map_name}_checkpoints.json"
            if os.path.exists(cp_file):
                with open(cp_file) as f:
                    saved = json.load(f)
        if checkpoint not in saved:
            _log(f"[ERROR] Checkpoint '{checkpoint}' not found.")
            return
        node.send_nav_goal(saved[checkpoint], checkpoint)

    elif cmd == 5:
        node.cancel_goal()
        _log("[EXIT] Exiting robot manager...")
        rclpy.shutdown()
        sys.exit(0)

    elif cmd == 6:
        map_name = _sanitize_name(str(payload.get('map_name', '')).strip())
        if not map_name or map_name == 'LIVE':
            _log("[MAP] Resetting to Live Session.")
            node.checkpoints = {}
        else:
            cp_file = f"/root/{map_name}_checkpoints.json"
            if os.path.isfile(cp_file):
                with open(cp_file) as f:
                    node.checkpoints = json.load(f)
                _log(f"[SUCCESS] Loaded {len(node.checkpoints)} checkpoints from '{map_name}'.")
                node.render_map_image()

    elif cmd == 9:
        name = str(payload.get('checkpoint_name', '')).strip()
        if name:
            node.delete_checkpoint(name)

# ── Main Entry Point ──────────────────────────────────────────────────────────

def interactive_menu():
    global _node_ref
    rclpy.init()
    node = RobotManagerNode()
    _node_ref = node

    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    threading.Thread(target=run_web_server, daemon=True).start()

    _log("==================================================================")
    _log("  DHRUV NAVIGATION & SLAM — INTERACTIVE CONTROL TERMINAL")
    _log("  Web Dashboard: http://<JETSON_IP>:8000")
    _log("  Commands:")
    _log("    1           -> Detailed Status & Sensor Telemetry")
    _log("    2:<name>    -> Set Named Checkpoint (e.g. 2:desk_area)")
    _log("    3:<mapname> -> Save Current Map & Checkpoints (e.g. 3:office)")
    _log("    4:<name>    -> Navigate Directly to Checkpoint (e.g. 4:desk_area)")
    _log("    0           -> Immediate Emergency Stop")
    _log("==================================================================")

    def _terminal():
        while True:
            try:
                line = input("").strip()
                if not line:
                    continue
                if line.lower() in ('0', 'stop', 'cancel'):
                    cmd_queue.put({'cmd': 0})
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
                        cmd_queue.put({'cmd': 4, 'map_name': 'LIVE', 'checkpoint': arg})
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
