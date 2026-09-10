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
        <style>
            body { background-color: #121212; color: #ffffff; font-family: sans-serif; text-align: center; }
            img { max-width: 100%; border: 2px solid #333; border-radius: 8px; margin-top: 20px; }
            .status { margin-top: 10px; font-size: 1.2em; color: #4CAF50; }
        </style>
    </head>
    <body>
        <h1>Dhruv Vision Feed</h1>
        <div class="status">● LIVE STREAM</div>
        <img src="/video_feed" alt="Video Feed" />
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