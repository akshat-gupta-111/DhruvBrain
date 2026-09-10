import threading
import uvicorn
from core.dashboard import app
from modes.exploration.exploration_state import run_exploration_mode
from core.hal import Camera

def start_dashboard():
    """Runs the FastAPI server on port 8000."""
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")

if __name__ == "__main__":
    print("\n🚀 Starting DhruvBrain Master Node...")
    
    # 1. Start the camera background thread
    cam = Camera()
    cam.start_capture_thread()
    
    # 2. Spin up the Dashboard on a separate thread
    dashboard_thread = threading.Thread(target=start_dashboard, daemon=True)
    dashboard_thread.start()
    
    print("🌐 Dashboard live at: http://<JETSON_IP>:8000")
    
    # 3. Start the LangGraph AI Loop on the main thread
    try:
        run_exploration_mode()
    except KeyboardInterrupt:
        print("\n[Master] Shutting down...")