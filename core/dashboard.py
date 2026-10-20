import cv2
import asyncio
from fastapi import FastAPI
from fastapi.responses import StreamingResponse, HTMLResponse

# Import the unified HAL camera
from core.hal import Camera

app = FastAPI()
camera = Camera()

# HTML template for the dashboard
html = """
<!DOCTYPE html>
<html>
    <head>
        <title>DhruvBrain Dashboard</title>
        <meta charset="utf-8">
        <style>
            * { box-sizing: border-box; margin: 0; padding: 0; }
            body { background-color: #0b0f19; color: #e2e8f0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; text-align: center; padding: 20px; }
            h1 { font-size: 1.6rem; color: #38bdf8; margin-bottom: 6px; }
            .subtitle { font-size: 0.85rem; color: #64748b; margin-bottom: 18px; }
            .container { max-width: 960px; margin: 0 auto; display: grid; grid-template-columns: 2fr 1fr; gap: 16px; text-align: left; }
            .card { background: #131b2e; border: 1px solid #1e293b; border-radius: 12px; padding: 16px; }
            .card h2 { font-size: 1rem; color: #94a3b8; margin-bottom: 10px; display: flex; align-items: center; justify-content: space-between; }
            .status { font-size: 0.75rem; color: #22c55e; background: #052e16; padding: 3px 10px; border-radius: 999px; }
            img { width: 100%; border: 1px solid #334155; border-radius: 8px; }
            .link-btn { display: inline-flex; align-items: center; justify-content: center; gap: 8px; width: 100%; padding: 12px; background: #0284c7; color: white; border-radius: 8px; text-decoration: none; font-weight: 600; font-size: 0.9rem; transition: background 0.2s; margin-top: 10px; }
            .link-btn:hover { background: #0369a1; }
            .info-item { display: flex; justify-content: space-between; padding: 8px 0; border-bottom: 1px solid #1e293b; font-size: 0.85rem; }
            .info-label { color: #94a3b8; }
            .info-val { font-family: monospace; color: #38bdf8; font-weight: 600; }
            @media(max-width: 768px) { .container { grid-template-columns: 1fr; } }
        </style>
    </head>
    <body>
        <h1>🤖 Dhruv Brain Master Dashboard</h1>
        <p class="subtitle">Vision, Conversation & Autonomous Navigation Integration</p>
        
        <div class="container">
            <div class="card">
                <h2>Vision Stream <span class="status">● CAMERA LIVE</span></h2>
                <img src="/video_feed" alt="Video Feed" />
            </div>
            
            <div>
                <div class="card" style="margin-bottom: 16px;">
                    <h2>🗺️ SLAM & Navigation</h2>
                    <p style="font-size: 0.85rem; color: #94a3b8; line-height: 1.5; margin-bottom: 12px;">
                        Interactive 2D Occupancy Map, Checkpoint Pins & Autonomous Goal Dispatch are active.
                    </p>
                    <a class="link-btn" href="http://" + window.location.hostname + ":8000" id="slam-link" target="_blank">
                        Open SLAM Map Viewer ↗
                    </a>
                </div>
                
                <div class="card">
                    <h2>⚙️ Subsystems</h2>
                    <div class="info-item"><span class="info-label">Brain Port</span><span class="info-val">8080</span></div>
                    <div class="info-item"><span class="info-label">SLAM Viewer</span><span class="info-val">8000</span></div>
                    <div class="info-item"><span class="info-label">Audio Sidecar</span><span class="info-val">5555</span></div>
                    <div class="info-item"><span class="info-label">Laser Odometry</span><span class="info-val">RF2O (/scan → /odom)</span></div>
                    <div class="info-item"><span class="info-label">Motor Protocol</span><span class="info-val">&lt;MIX,vx,vy,w&gt; Holonomic</span></div>
                </div>
            </div>
        </div>

        <script>
            // Automatically point SLAM link to the host's IP on port 8000
            document.getElementById('slam-link').href = 'http://' + window.location.hostname + ':8000';
        </script>
    </body>
</html>
"""

def generate_frames():
    """Generator to read frames and yield them as MJPEG."""
    # camera.open()
    while True:
        try:
            # We grab raw OpenCV frames directly from the HAL
            ret, frame = camera.cap.read()
            if not ret:
                continue

            # Encode as JPEG for the web stream
            ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            frame_bytes = buffer.tobytes()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
        except Exception as e:
            print(f"[Dashboard] Frame generation error: {e}")
            break

@app.get("/")
async def get_dashboard():
    """Serves the main HTML page."""
    return HTMLResponse(html)

@app.get("/video_feed")
async def video_feed():
    """Serves the MJPEG video stream."""
    return StreamingResponse(generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame")