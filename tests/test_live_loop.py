# test_live_loop.py (Place in DhruvBrain root to test)
from core.hal_mac import MacCamera, MotorController, Speaker, MockLiDAR
from core.ai_pipeline import process_visual_frame

def main():
    camera = MacCamera()
    motors = MotorController()
    speaker = Speaker()
    lidar = MockLiDAR()

    print("\n🚀 DhruvBrain Live Testbed initialized.")
    camera.open()

    try:
        while True:
            cmd = lidar.prompt_user()
            if cmd == "q":
                break

            print("[HAL] Grabbing frame from webcam...")
            frame_bytes = camera.capture_frame_bytes()

            print("[Brain] Running perception & cognitive loop...")
            decision = process_visual_frame(frame_bytes)

            print("\n" + "-" * 40)
            print(f"Critique: {decision.visual_critique}")
            print(f"Action:   {decision.physical_action}")
            print(f"Speech:   {decision.speech}")
            print(f"LED Mood: {decision.led_mood}")
            print("-" * 40)

            # Execute actions through HAL
            motors.execute(decision.physical_action)
            speaker.speak(decision.speech)

    finally:
        camera.close()
        print("[HAL] Shutdown complete.")

if __name__ == "__main__":
    main()