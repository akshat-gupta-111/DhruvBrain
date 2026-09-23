import time
import threading
import uvicorn
import string
from core.dashboard import app
from core.hal import Camera, Microphone, Speaker, MotorController
from core.vision_face import FaceIdentityEngine

from modes.exploration.exploration_state import run_exploration_step
from modes.conversation.conversation_state import generate_chat_response
from modes.find_exit.find_exit_logic import get_exit_action
from core.hal_jetson import JetsonLiDAR

class DhruvOrchestrator:
    def __init__(self, camera: Camera):
        self.state = "IDLE"
        self.mic = Microphone()
        self.speaker = Speaker()
        self.motors = MotorController()
        self.camera = camera
        self.lidar = JetsonLiDAR()
        
    def safe_speak(self, text: str):
        """Mutes the mic, speaks, waits for the cloud echo to pass, then unmutes."""
        self.mic.mute()
        
        # Trigger Arduino light blinking
        self.motors.execute("HALT", "SPEAKING")
        
        self.speaker.speak(text)
        
        # INCREASED to 2.5s: Gives Azure STT enough time to drop late transcriptions 
        time.sleep(2.5) 
        
        self.mic.unmute()
        
        # Return to idle light
        self.motors.execute("HALT", "IDLE_WHITE")

    def get_full_speech(self) -> str:
        """Drains the queue and combines ALL speech heard, instead of just the last phrase."""
        speech_parts = []
        while True:
            s = self.mic.get_speech()
            if s:
                speech_parts.append(s)
            else:
                break
        return " ".join(speech_parts).strip()

    def check_for_interrupts(self, speech: str) -> bool:
        """Checks for global commands. Returns True if a mode switch occurred."""
        if not speech:
            return False
            
        # Strip all punctuation and lowercase it so "hello, drove." becomes "hello drove"
        clean_speech = speech.translate(str.maketrans('', '', string.punctuation)).lower()
        
        if any(w in clean_speech for w in ["hello dhruv", "hello drove", "hey dhruv", "hi dhruv", "hello robot"]):
            if self.state == "FIND_EXIT": 
                self.speaker.stop_loop()
            if self.state in ["FIND_EXIT", "LIDAR_TEST"]:
                self.lidar.stop()
            self.motors.clear_queue()
            self.state = "CONVERSATION"
            self.motors.execute("HALT", "FLIRT_PINK")
            
            greeting = "Hey there! What's on your mind?"
            try:
                # Try to see who we are talking to
                frame = self.camera.capture_frame()
                names, desc = FaceIdentityEngine().detect_faces(frame)
                if names:
                    # Greet the first person found, default to 'Hey there' if unknown
                    first_person = names[0]
                    if first_person != "Unknown":
                        greeting = f"Hey {first_person}! What's on your mind?"
                    else:
                        greeting = "Hey there! I don't think we've met. What's on your mind?"
            except Exception as e:
                print(f"[Orchestrator] Vision greeting error: {e}")

            self.safe_speak(greeting)
            return True
            
        if any(w in clean_speech for w in ["go explore", "start exploring", "explore"]):
            if self.state == "FIND_EXIT": 
                self.speaker.stop_loop()
            if self.state in ["FIND_EXIT", "LIDAR_TEST"]:
                self.lidar.stop()
            self.motors.clear_queue()
            self.state = "EXPLORATION"
            self.motors.execute("CONTINUE_WANDER", "CURIOSITY_GREEN")
            self.safe_speak("Alright, scanning the perimeter.")
            return True
            
        if "sleep" in clean_speech or "shut down" in clean_speech:
            if self.state == "FIND_EXIT": 
                self.speaker.stop_loop()
            if self.state in ["FIND_EXIT", "LIDAR_TEST"]:
                self.lidar.stop()
            self.motors.clear_queue()
            self.state = "IDLE"
            self.motors.execute("HALT", "IDLE_WHITE")
            self.safe_speak("Powering down motors. I'll be listening if you need me.")
            return True
            
        if any(w in clean_speech for w in ["find exit", "find the exit", "escape the room"]):
            if self.state != "FIND_EXIT":
                if self.state == "LIDAR_TEST": self.lidar.stop()
                self.motors.clear_queue()
                self.state = "FIND_EXIT"
                self.lidar.start()
                self.motors.execute("HALT", "ALERT_RED")
                self.safe_speak("Initiating escape sequence. Scanning for exits.")
                self.speaker.play_loop("movement.wav")
            return True
            
        if any(w in clean_speech for w in ["lidar test", "test lidar"]):
            if self.state != "LIDAR_TEST":
                if self.state == "FIND_EXIT": self.speaker.stop_loop()
                self.motors.clear_queue()
                self.state = "LIDAR_TEST"
                self.lidar.start()
                self.motors.execute("HALT", "CURIOSITY_GREEN")
                self.safe_speak("LiDAR test mode activated. Printing distances.")
            return True
            
        return False

    def run(self):
        print("\n🚀 Starting Dhruv Master Orchestrator...")
        self.safe_speak("System online. Say 'go explore' for exploration mode, or 'hello Dhruv' for conversation, or find exit for escape sequence.")
        self.mic.unmute()

        while True:
            # 1. Grab all speech currently in the queue
            speech = self.get_full_speech()
            
            # 2. Handle global voice interrupts first
            if self.check_for_interrupts(speech):
                continue

            # 3. State-Specific Behaviors
            if self.state == "CONVERSATION":
                if speech:
                    self.motors.execute("HALT", "THINKING_BLUE")
                    
                    visual_context = ""
                    try:
                        frame = self.camera.capture_frame()
                        names, face_desc = FaceIdentityEngine().detect_faces(frame)
                        if face_desc:
                            visual_context = f"[Vision System detected faces: {face_desc}]"
                    except Exception as e:
                        print(f"[Orchestrator] Continuous vision error: {e}")
                        
                    reply = generate_chat_response(speech, visual_context)
                    self.safe_speak(reply)
                    self.motors.execute("HALT", "FLIRT_PINK")
                else:
                    time.sleep(0.5)

            elif self.state == "EXPLORATION":
                print("\n[WANDER STATE] 🔄 Motors moving forward. Scanning...")
                
                # This takes ~5 seconds to process vision and reason
                decision = run_exploration_step()
                
                # CRITICAL FIX: Did you say "Hello Dhruv" DURING the 5 second scan?
                # We check the queue AGAIN before executing the action!
                late_speech = self.get_full_speech()
                if self.check_for_interrupts(late_speech):
                    continue # Abort the exploration action, jump straight to conversation!
                
                # If no interrupt was heard, safely execute the exploration decision
                if decision:
                    self.motors.execute(decision.physical_action, decision.led_mood)
                    self.safe_speak(decision.speech)
                    
                time.sleep(0.5) 
                
            elif self.state == "FIND_EXIT":
                # Fast LiDAR navigation loop (No VLM processing)
                safest_dir = self.lidar.get_safest_direction()
                action, led_mood = get_exit_action(safest_dir)
                
                late_speech = self.get_full_speech()
                if self.check_for_interrupts(late_speech):
                    continue
                    
                if action:
                    self.motors.execute(action, led_mood)
                
                # Stream commands at 10Hz for perfectly smooth continuous movement
                time.sleep(0.1)
                
            elif self.state == "LIDAR_TEST":
                # Just print the safest direction to the console, don't move motors
                self.lidar.get_safest_direction()
                time.sleep(0.5)
                
            elif self.state == "IDLE":
                time.sleep(1)

def start_dashboard():
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")

if __name__ == "__main__":
    cam = Camera()
    cam.start_capture_thread()
    threading.Thread(target=start_dashboard, daemon=True).start()
    
    orchestrator = DhruvOrchestrator(cam)
    try:
        orchestrator.run()
    except KeyboardInterrupt:
        print("\n[Master] Shutting down...")