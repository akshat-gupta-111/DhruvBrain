import sys
import os
import time

# Add the project root to the path so we can import from core
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.hal import Microphone, Speaker

def main():
    print("Initializing Speaker and Microphone...")
    speaker = Speaker()
    mic = Microphone()
    
    print("Unmuting microphone...")
    mic.unmute()
    
    print("Ready! Say something, and I will repeat it back to you. Listening for 30 seconds...")
    start_time = time.time()
    
    while time.time() - start_time < 30:
        speech = mic.get_speech()
        if speech:
            print(f"Heard: {speech}")
            print(f"Speaking: {speech}")
            speaker.speak(f"You said: {speech}")
        time.sleep(0.1)
        
    print("Muting microphone and exiting...")
    mic.mute()
    print("Merge test completed.")

if __name__ == "__main__":
    main()
