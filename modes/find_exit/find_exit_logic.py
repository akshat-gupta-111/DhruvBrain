import time
from typing import Optional, Tuple

def get_exit_action(safest_direction: str) -> Tuple[Optional[str], str]:
    """
    Maps the safest LiDAR direction to a physical motor action.
    Returns (action, led_mood).
    """
    if not safest_direction:
        # If LiDAR is completely blocked or errors, turn to find a new path
        return ("PIVOT_LEFT_30", "ALERT_RED")
        
    print(f"[Find Exit] Safest direction is {safest_direction}. Navigating...")

    # The "Mouse Algorithm": 
    # If the path is straight ahead, walk into it.
    # If it's to the side, pivot towards it.
    # If it's behind, turn around.
    
    if safest_direction == 'N':
        return ("APPROACH_0.5M", "CURIOSITY_GREEN")
    elif safest_direction in ['NW', 'W', 'SW']:
        return ("PIVOT_LEFT_30", "THINKING_BLUE")
    elif safest_direction in ['NE', 'E', 'SE']:
        return ("PIVOT_RIGHT_30", "THINKING_BLUE")
    elif safest_direction == 'S':
        # Turn around
        return ("PIVOT_RIGHT_30", "ALERT_RED")
        
    return (None, "IDLE_WHITE")
