import sys
import os
import time

# Add the project root to the path so we can import from core
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.hal import Microphone

def main():
    print("Initializing Microphone...")
    mic = Microphone()
    
    print("Microphone initialized. Unmuting...")
    mic.unmute()
    
    print("Listening for 15 seconds. Please say something...")
    start_time = time.time()
    
    while time.time() - start_time < 15:
        speech = mic.get_speech()
        if speech:
            print(f"Received speech: {speech}")
        time.sleep(0.1)
        
    print("Muting microphone...")
    mic.mute()
    print("Microphone test completed.")

if __name__ == "__main__":
    main()