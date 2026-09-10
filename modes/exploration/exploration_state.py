import os
from typing import TypedDict, Optional
# pyrefly: ignore [missing-import]
from langgraph.graph import StateGraph, END

# Import our modular blocks
from core.hal_mac import MacCamera, MotorController, Speaker, MockLiDAR
from core.ai_pipeline import analyze_frame_moondream, extract_text_azure, reason_and_decide, RobotDecision

# from core.hal import Camera, MotorController, Speaker, LiDAR

camera = MacCamera()
motors = MotorController()
speaker = Speaker()
lidar = MockLiDAR()
# ==========================================
# 1. Define the LangGraph State
# ==========================================
class RobotState(TypedDict):
    image_bytes: Optional[bytes]
    perception_text: str
    ocr_text: str
    decision: Optional[RobotDecision]
    should_quit: bool

# Initialize Hardware (Global to this mode for simplicity)
# camera = Camera()
# motors = MotorController()
# speaker = Speaker()
# lidar = LiDAR()

# ==========================================
# 2. Define the Nodes
# ==========================================
def node_capture(state: RobotState) -> dict:
    """Simulates LiDAR interrupting the wander state to capture a frame."""
    cmd = lidar.prompt_user()
    if cmd == "q":
        return {"should_quit": True}
    
    print("[Graph Node: Capture] Grabbing frame...")
    frame = camera.capture_frame_bytes()
    return {"image_bytes": frame, "should_quit": False, "perception_text": "", "ocr_text": ""}

def node_perceive(state: RobotState) -> dict:
    """Calls Moondream and optionally Azure OCR."""
    print("[Graph Node: Perceive] Analyzing scene context...")
    perception_text = analyze_frame_moondream(state["image_bytes"])
    
    ocr_text = ""
    keywords = ["text", "poster", "board", "written", "screen", "sign", "paper", "magazine", "book"]
    if any(word in perception_text.lower() for word in keywords):
        print("[Graph Node: Perceive] Readable target detected. Running OCR...")
        ocr_text = extract_text_azure(state["image_bytes"])
        
    return {"perception_text": perception_text, "ocr_text": ocr_text}

def node_reason(state: RobotState) -> dict:
    """Passes context to GPT-4o to generate Embodied Actions."""
    print("[Graph Node: Reason] Thinking...")
    decision = reason_and_decide(state["perception_text"], state["ocr_text"])
    
    print("\n" + "=" * 40)
    print(f"🧠 Critique: {decision.visual_critique}")
    print(f"⚙️ Action:   {decision.physical_action}")
    print(f"💬 Speech:   {decision.speech}")
    print(f"💡 LED:      {decision.led_mood}")
    print("=" * 40 + "\n")
    
    return {"decision": decision}

def node_act(state: RobotState) -> dict:
    """Executes the physical and vocal actions through the HAL."""
    decision = state["decision"]
    if decision:
        motors.execute(decision.physical_action,decision.led_mood)
        speaker.speak(decision.speech)
    return {}

# ==========================================
# 3. Routing Logic
# ==========================================
def route_after_capture(state: RobotState) -> str:
    if state["should_quit"]:
        return "end"
    return "perceive"

# ==========================================
# 4. Build and Compile the Graph
# ==========================================
def build_exploration_graph():
    workflow = StateGraph(RobotState)
    
    # Add nodes
    workflow.add_node("capture", node_capture)
    workflow.add_node("perceive", node_perceive)
    workflow.add_node("reason", node_reason)
    workflow.add_node("act", node_act)
    
    # Add edges
    workflow.set_entry_point("capture")
    workflow.add_conditional_edges(
        "capture",
        route_after_capture,
        {
            "perceive": "perceive",
            "end": END
        }
    )
    workflow.add_edge("perceive", "reason")
    workflow.add_edge("reason", "act")
    workflow.add_edge("act", END)
    
    return workflow.compile()

# ==========================================
# 5. The Main Loop (Wander Mode)
# ==========================================
def run_exploration_mode():
    graph = build_exploration_graph()
    
    print("\n🚀 Dhruv Exploration Mode Initiated.")
    # camera.open()
    
    try:
        while True:
            # The robot is "wandering" until LiDAR triggers the graph
            print("\n[WANDER STATE] 🔄 Motors moving forward. LiDAR scanning...")
            
            # Run one full cognitive loop
            final_state = graph.invoke({
                "image_bytes": None,
                "perception_text": "",
                "ocr_text": "",
                "decision": None,
                "should_quit": False
            })
            
            if final_state.get("should_quit"):
                print("[WANDER STATE] Terminating exploration mode.")
                break
                
    finally:
        camera.close()
        print("[HAL] Hardware released safely.")

if __name__ == "__main__":
    run_exploration_mode()