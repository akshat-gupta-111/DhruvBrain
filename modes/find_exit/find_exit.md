# Find Exit Mode - Design & Planning

## Goal
Design a new mode ("Find Exit") that relies *solely* on the LiDAR sensor to navigate a room, avoid obstacles, and locate the exit (such as an open doorway or longest free path). While in this mode, the robot will continuously play a specified sound (`movement.wav`) to signal that it is actively searching.

## Previous Work & Inspiration
- **Exploration Mode (`modes/exploration`)**: We successfully built a state machine that captures images, runs VLM (Moondream/Azure OCR), and makes autonomous decisions. It uses the `ai_pipeline.py`.
- **Motor Control (`H-drive-jetson-cable-v2.ino` & `hal_jetson.py`)**: The movement is now robust. Timed commands (e.g., `<FWD,180,1000>`) correctly block the Jetson queue until completion, ensuring smooth, non-jerky movements.
- **LiDAR Integration**: The `JetsonLiDAR.get_safest_direction()` function splits the 360-degree scan into 8 sectors ('N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW') and returns the sector with the maximum free distance. 

## Technical Approach

### 1. Navigation Logic (The "Mouse" Algorithm)
A real mouse finding its way out of a maze or room typically follows a simple heuristic:
- **Look for the most open space**: We will use `lidar.get_safest_direction()` to find the longest unobstructed path.
- **Move towards it**:
  - If the safest direction is `N` (Forward), we will execute `APPROACH_0.5M`.
  - If the safest direction is `NW`, `W`, or `SW`, we will execute `PIVOT_LEFT_30` to turn towards it.
  - If the safest direction is `NE`, `E`, or `SE`, we will execute `PIVOT_RIGHT_30`.
  - If the safest direction is `S` (Behind), we will execute `PIVOT_LEFT_30` (or `PIVOT_RIGHT_30`) repeatedly to turn around.
- **Continuous Loop**: This logic will run in a fast loop in `main.py` under the `FIND_EXIT` state.

### 2. Audio Playback (`movement.wav`)
We need a robust way to play a background audio file continuously without blocking the navigation logic.
- We will use a background thread or a subprocess calling `aplay` (or similar audio player on Linux/Jetson).
- When entering the `FIND_EXIT` state, the audio starts looping.
- When exiting the state (e.g., by saying "sleep" or "hello dhruv"), the audio playback is terminated.

### 3. State Machine Integration (`main.py`)
- Add a new state: `FIND_EXIT`.
- Add wake phrases: "find the exit", "find exit", "escape the room".
- In the main loop, if `self.state == "FIND_EXIT"`, run the LiDAR navigation logic instead of the VLM logic.

## Implementation Steps
1. Create `modes/find_exit/find_exit_logic.py` with the decision mapping based on LiDAR output.
2. Add audio loop management (start/stop background audio) to `core/hal_jetson.py` or a utility file.
3. Update `main.py` to handle the `FIND_EXIT` state, trigger words, and audio loop.
