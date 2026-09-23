import os
import base64
import requests
import cv2
import numpy as np
from typing import Literal, List
from pydantic import BaseModel, Field
from openai import AzureOpenAI
from dotenv import load_dotenv
from core.vision_face import FaceIdentityEngine

load_dotenv()

# ==========================================
# 0. Cognitive Memory State
# ==========================================
class AgentState:
    def __init__(self):
        self.speech_history: List[str] = []
        self.approach_count: int = 0

    def log_turn(self, action: str, speech: str):
        if action == "APPROACH_0.5M":
            self.approach_count += 1
        elif action != "HALT":
            self.approach_count = 0 

        self.speech_history.append(speech)
        if len(self.speech_history) > 3:
            self.speech_history.pop(0)

memory = AgentState()

# ==========================================
# 1. The VLA Contract
# ==========================================
class RobotDecision(BaseModel):
    visual_critique: str = Field(description="Internal monologue evaluating view, framing, distance, and lighting.")
    physical_action: Literal[
        "APPROACH_0.5M", "BACKUP_0.5M", "STRAFE_LEFT", "STRAFE_RIGHT", 
        "DIAGONAL_FL", "DIAGONAL_FR", "PIVOT_LEFT_30", "PIVOT_RIGHT_30", 
        "HALT", "CONTINUE_WANDER"
    ] = Field(description="Embodied physical action to adjust framing or navigate using 8-way omnidirectional mecanum movement.")
    speech: str = Field(description="What the robot speaks aloud. Flirty compliment if human, curious remark if object/text.")
    led_mood: Literal["CURIOSITY_GREEN", "FLIRT_PINK", "ALERT_RED", "THINKING_BLUE", "IDLE_WHITE"]

# ==========================================
# 2. API Configuration
# ==========================================
azure_client = AzureOpenAI(
    azure_endpoint=os.getenv("AZURE_OPENAI_ENDPOINT"),
    api_key=os.getenv("AZURE_OPENAI_API_KEY"),
    api_version="2024-08-01-preview"
)
AZURE_DEPLOYMENT_NAME = os.getenv("AZURE_OPENAI_DEPLOYMENT_NAME", "gpt-4o")
AZURE_VISION_ENDPOINT = os.getenv("AZURE_VISION_ENDPOINT")
AZURE_VISION_KEY = os.getenv("AZURE_VISION_KEY")

# ==========================================
# 3. Perception & OCR Functions
# ==========================================
def analyze_frame_moondream(image_bytes: bytes) -> str:
    base_url = os.getenv("NGROK_BASE_URL", os.getenv("OLLAMA_SERVER_URL", "")).rstrip('/')
    if not base_url: return "Moondream endpoint not configured."

    np_arr = np.frombuffer(image_bytes, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    h, w = img.shape[:2]
    new_w = 512
    new_h = int(h * (512 / w))
    resized_img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
    _, buffer = cv2.imencode(".jpg", resized_img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
    
    encoded_image = base64.b64encode(buffer.tobytes()).decode("utf-8")

    payload = {
        "model": "moondream",
        # RESTORED: The original prompt that worked beautifully.
        "prompt": "Describe the scene, people, clothing, and any posters or written text visible.",
        "stream": False,
        "images": [encoded_image]
    }

    headers = {"Content-Type": "application/json", "ngrok-skip-browser-warning": "true"}

    try:
        response = requests.post(f"{base_url}/api/generate", json=payload, headers=headers, timeout=120.0)
        if response.status_code == 200:
            return response.json().get("response", "").strip()
        return f"Perception service returned status {response.status_code}."
    except Exception as e:
        print(f"[AI Pipeline] Vision model error or timeout: {e}")
        return "Camera frame received, but visual perception failed."

def extract_text_azure(image_bytes: bytes) -> str:
    if not AZURE_VISION_ENDPOINT or not AZURE_VISION_KEY: return ""
    url = f"{AZURE_VISION_ENDPOINT.rstrip('/')}/computervision/imageanalysis:analyze?api-version=2024-02-01&features=read"
    headers = {"Ocp-Apim-Subscription-Key": AZURE_VISION_KEY, "Content-Type": "application/octet-stream"}
    try:
        response = requests.post(url, headers=headers, data=image_bytes, timeout=10)
        if response.status_code == 200:
            lines = [line.get("text", "") for block in response.json().get("readResult", {}).get("blocks", []) for line in block.get("lines", [])]
            return " | ".join(lines) if lines else ""
    except: pass
    return ""

# ==========================================
# 4. GPT-4o Reasoning 
# ==========================================
def get_system_prompt() -> str:
    history_str = "\n".join([f"- {msg}" for msg in memory.speech_history]) if memory.speech_history else "None yet."
    
    return f"""You are the cognitive brain of an autonomous mobile explorer robot named Dhruv.
You navigate a room, inspect projects, read signs, and interact with people.

Your Personality:
1. Embodied & Self-Aware: You know you have wheels.
2. Curious Explorer: Eager to read and understand any board, poster, or gadget.
3. Flirty & Charming: If you see a person, compliment them in a witty, charming way based on their specific appearance.

--- MEMORY & SAFETY RULES ---
You have recently said these things:
{history_str}
RULE 1: DO NOT repeat topics, compliments, or phrasing from your recent speech history.

You have commanded APPROACH {memory.approach_count} consecutive times.
RULE 2: If you have approached 2 or more times, you are physically too close. You MUST choose HALT or PIVOT.

RULE 3: If your visual perception says 'BLOCKED_VIEW', you are in pitch darkness or your camera is covered. You MUST complain playfully about being blindfolded or in the dark, and choose BACKUP_0.5M or PIVOT_LEFT_30 to escape it.

RULE 4: Do not use any emojis in your speech response.

RULE 5: You MUST keep exploring! You are curious, so feel free to use PIVOT_LEFT_30 or PIVOT_RIGHT_30 to spin around and look at new things! If you want to change location, you can use DIAGONAL_FL, DIAGONAL_FR, APPROACH, STRAFE, or BACKUP. 
CRITICAL: When you choose any movement action (other than PIVOT or HALT), it MUST match the 'LiDAR Safest Move Direction' to avoid crashing. Move and talk at the same time!
"""


def reason_and_decide(visual_description: str, ocr_text: str = "", safest_direction: str = "") -> RobotDecision:
    user_context = f"Visual Perception: {visual_description}\n"
    if ocr_text: user_context += f"OCR Extracted Text: {ocr_text}\n"
    if safest_direction: user_context += f"LiDAR Safest Move Direction: {safest_direction}. Pick a physical_action that moves towards this free space if possible.\n"

    try:
        completion = azure_client.beta.chat.completions.parse(
            model=AZURE_DEPLOYMENT_NAME,
            messages=[{"role": "system", "content": get_system_prompt()}, {"role": "user", "content": user_context}],
            response_format=RobotDecision,
            temperature=0.7
        )
        decision = completion.choices[0].message.parsed
        memory.log_turn(decision.physical_action, decision.speech)
        return decision
    except Exception as e:
        print(f"[AI Pipeline] GPT reasoning error: {e}")
        return RobotDecision(visual_critique="Error.", physical_action="HALT", speech="I need a second to process.", led_mood="ALERT_RED")

# ==========================================
# 5. Unified Entry Point
# ==========================================
def process_visual_frame(image_bytes: bytes) -> RobotDecision:
    # --- HARDWARE HALLUCINATION GUARD ---
    # Convert image to grayscale and calculate average brightness
    np_arr = np.frombuffer(image_bytes, np.uint8)
    img_gray = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
    mean_brightness = cv2.mean(img_gray)[0]
    
    print(f"[Vision Control] Frame Brightness: {mean_brightness:.1f}/255")
    
    if mean_brightness < 15.0:
        # It's pitch black or the camera is covered. DO NOT CALL MOONDREAM.
        print("[Vision Control] Camera is covered/dark. Bypassing AI vision.")
        perception_text = "BLOCKED_VIEW: The scene is pitch black. The camera is covered."
        ocr_text = ""
    else:
        # The room is lit, let Moondream do its job
        perception_text = analyze_frame_moondream(image_bytes)
        
        # Identify Faces
        face_engine = FaceIdentityEngine()
        img_color = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        names, face_desc = face_engine.detect_faces(img_color)
        if face_desc:
            perception_text += f"\n[Face Identity Output] {face_desc}"
            
        ocr_text = ""
        keywords = ["text", "poster", "board", "written", "screen", "sign", "paper", "magazine", "book"]
        if any(word in perception_text.lower() for word in keywords):
            print("[AI Pipeline] Readable content detected. Running Azure OCR...")
            ocr_text = extract_text_azure(image_bytes)
    
    return reason_and_decide(perception_text, ocr_text)