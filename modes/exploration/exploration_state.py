import os
from typing import TypedDict, Optional
from langgraph.graph import StateGraph, END

from core.hal import Camera, LiDAR
from core.ai_pipeline import analyze_frame_moondream, extract_text_azure, reason_and_decide, RobotDecision
from core.vision_face import FaceIdentityEngine
import cv2
import numpy as np

# ==========================================
# 1. State Definition
# ==========================================
class RobotState(TypedDict):
    image_bytes: Optional[bytes]
    perception_text: str
    ocr_text: str
    safest_direction: str
    decision: Optional[RobotDecision]
    should_quit: bool
    wake_word_triggered: bool

# ==========================================
# 2. Hardware for this module (Safe Singletons)
# ==========================================
camera = Camera()
lidar = LiDAR()

# ==========================================
# 3. Nodes
# ==========================================
def node_capture(state: RobotState) -> dict:
    cmd = lidar.prompt_user() 
    
    if cmd == "q":
        return {"should_quit": True, "wake_word_triggered": False, "image_bytes": None, "perception_text": "", "ocr_text": "", "safest_direction": ""}
        
    print("[Graph Node: Capture] Grabbing frame...")
    try:
        frame = camera.capture_frame_bytes()
    except Exception as e:
        print(f"[Graph Node: Capture] Camera error: {e}")
        frame = None
        
    safest_dir = ""
    if hasattr(lidar, "get_safest_direction"):
        physical_safe = lidar.get_safest_direction()
        if physical_safe:
            # The Camera is facing the rear (Physical Back = LiDAR 'S').
            # We must translate the LiDAR's Physical Direction to Camera-Relative Direction
            # so the AI (which only sees through the camera) steers correctly.
            mapping = {
                "N": "Back",
                "S": "Front",
                "E": "Left",
                "W": "Right",
                "NE": "Back-Left",
                "NW": "Back-Right",
                "SE": "Front-Left",
                "SW": "Front-Right"
            }
            safest_dir = mapping.get(physical_safe, "")
            
    return {"image_bytes": frame, "should_quit": False, "wake_word_triggered": False, "perception_text": "", "ocr_text": "", "safest_direction": safest_dir}

def route_after_capture(state: RobotState) -> str:
    if state.get("should_quit") or state.get("wake_word_triggered"):
        return "end"
    return "perceive"

def node_perceive(state: RobotState) -> dict:
    if not state.get("image_bytes"):
        return {"perception_text": "No visual data available.", "ocr_text": ""}
        
    print("[Graph Node: Perceive] Analyzing scene context...")
    perception = analyze_frame_moondream(state["image_bytes"])
    
    try:
        np_arr = np.frombuffer(state["image_bytes"], np.uint8)
        img_color = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        names, face_desc = FaceIdentityEngine().detect_faces(img_color)
        if face_desc:
            perception += f"\n[Face Identity Output] {face_desc}"
    except Exception as e:
        print(f"[Graph Node: Perceive] Face Identity Error: {e}")
    ocr = ""
    
    if "text" in perception.lower() or "sign" in perception.lower() or "read" in perception.lower():
        print("[Graph Node: Perceive] Readable target detected. Running OCR...")
        ocr = extract_text_azure(state["image_bytes"])
        
    return {"perception_text": perception, "ocr_text": ocr}

def node_reason(state: RobotState) -> dict:
    print("[Graph Node: Reason] Thinking...")
    decision = reason_and_decide(state["perception_text"], state["ocr_text"], state.get("safest_direction", ""))
    return {"decision": decision}

# ==========================================
# 4. Graph Builder
# ==========================================
def build_exploration_graph():
    workflow = StateGraph(RobotState)
    
    workflow.add_node("capture", node_capture)
    workflow.add_node("perceive", node_perceive)
    workflow.add_node("reason", node_reason)
    
    workflow.set_entry_point("capture")
    workflow.add_conditional_edges(
        "capture",
        route_after_capture,
        {"perceive": "perceive", "end": END}
    )
    workflow.add_edge("perceive", "reason")
    workflow.add_edge("reason", END) 
    
    return workflow.compile()

# ==========================================
# 5. Main Loop Execution
# ==========================================
def run_exploration_step():
    graph = build_exploration_graph()
    
    final_state = graph.invoke({
        "image_bytes": None,
        "perception_text": "",
        "ocr_text": "",
        "safest_direction": "",
        "decision": None,
        "should_quit": False,
        "wake_word_triggered": False
    })
    
    return final_state.get("decision")