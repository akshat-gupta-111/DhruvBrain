import sys
import os

# Add the project root to the path so we can import from core
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.hal import Speaker
import time

def main():
    print("Initializing Speaker...")
    speaker = Speaker()
    
    print("Testing Speaker...")
    test_phrase = "Hello, this is a test of the Dhruv Brain speaker system."
    speaker.speak(test_phrase)
    
    # Wait a moment for speech to finish
    time.sleep(3)
    print("Speaker test completed.")

if __name__ == "__main__":
    main()
