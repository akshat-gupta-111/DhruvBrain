import os
from typing import TypedDict, Optional
# pyrefly: ignore [missing-import]
from langgraph.graph import StateGraph, END

# Import our modular blocks
from core.hal import Camera, MotorController, Speaker, LiDAR, Microphone
from core.ai_pipeline import analyze_frame_moondream, extract_text_azure, reason_and_decide, RobotDecision

# from core.hal import Camera, MotorController, Speaker, LiDAR

camera = Camera()
motors = MotorController()
speaker = Speaker()
lidar = LiDAR()
mic = Microphone()
# ==========================================
# 1. Define the LangGraph State
# ==========================================
class RobotState(TypedDict):
    image_bytes: Optional[bytes]
    perception_text: str
    ocr_text: str
    decision: Optional[RobotDecision]
    should_quit: bool
    wake_word_triggered: bool

# Initialize Hardware (Global to this mode for simplicity)
# camera = Camera()
# motors = MotorController()
# speaker = Speaker()
# lidar = LiDAR()

# ==========================================
# 2. Define the Nodes
# ==========================================
def node_capture(state: RobotState) -> dict:
    # Pass the mic instance so LiDAR can check it non-blockingly
    cmd = lidar.prompt_user() 
    
    if cmd == "q":
        return {"should_quit": True, "wake_word_triggered": False}
        
    if cmd == "WAKE_WORD":
        # Abort the graph instantly! Don't process vision.
        return {"should_quit": True, "wake_word_triggered": True, "image_bytes": None, "perception_text": "", "ocr_text": ""}
    
    print("[Graph Node: Capture] Grabbing frame...")
    frame = camera.capture_frame_bytes()
    return {"image_bytes": frame, "should_quit": False, "wake_word_triggered": False, "perception_text": "", "ocr_text": ""}


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

# Remove node_act and update the graph builder:


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
# ==========================================
# 4. Build and Compile the Graph
# ==========================================
def build_exploration_graph():
    workflow = StateGraph(RobotState)
    
    # Add nodes (Notice: no 'act' node!)
    workflow.add_node("capture", node_capture)
    workflow.add_node("perceive", node_perceive)
    workflow.add_node("reason", node_reason)
    
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
    workflow.add_edge("reason", END) # End right after reasoning!
    
    return workflow.compile()

# ==========================================
# 5. The Main Loop (Wander Mode)
# ==========================================
def run_exploration_step():
    graph = build_exploration_graph()
    
    # Run one full cognitive loop and return the decision
    final_state = graph.invoke({
        "image_bytes": None,
        "perception_text": "",
        "ocr_text": "",
        "decision": None,
        "should_quit": False,
        "wake_word_triggered": False
    })
    
    return final_state.get("decision")

if __name__ == "__main__":
    run_exploration_mode()