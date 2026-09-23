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
        return ("CONT_FWD", "CURIOSITY_GREEN")
    elif safest_direction == 'NW':
        return ("CONT_DIAG_FL", "CURIOSITY_GREEN")
    elif safest_direction == 'NE':
        return ("CONT_DIAG_FR", "CURIOSITY_GREEN")
    elif safest_direction in ['W', 'SW']:
        return ("CONT_PIVOT_L", "THINKING_BLUE")
    elif safest_direction in ['E', 'SE']:
        return ("CONT_PIVOT_R", "THINKING_BLUE")
    elif safest_direction == 'S':
        # Turn around
        return ("CONT_PIVOT_R", "ALERT_RED")
        
    return (None, "IDLE_WHITE")
