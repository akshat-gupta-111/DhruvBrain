import time
from typing import Optional, Tuple

def get_exit_action(safest_direction: str, obstacles: dict = None) -> Tuple[Optional[str], str]:
    """
    Maps the safest LiDAR direction to a physical motor action.
    Returns (action, led_mood).
    """
    if not safest_direction or not obstacles:
        # If LiDAR is completely blocked or errors, turn to find a new path
        return ("PIVOT_LEFT_30", "ALERT_RED")
        
    print(f"[Find Exit] Safest direction is {safest_direction}. Navigating...")

    # The "Human Algorithm": 
    # Humans don't constantly spin towards the absolute largest open space. 
    # If the path forward has enough clearance (e.g., > 0.6m), we just keep walking forward!
    # We only look for the "safest direction" to turn to if our front is blocked!
    
    front_dist = obstacles.get('N', 0)
    
    if front_dist > 600:
        # Front is safe enough to walk! Just go forward.
        return ("LIDAR_FWD", "CURIOSITY_GREEN")
    
    # If front is blocked (< 600mm), THEN we pivot towards the safest open space!
    if safest_direction == 'N':
        return ("LIDAR_FWD", "CURIOSITY_GREEN")
    elif safest_direction == 'NW':
        return ("LIDAR_DIAG_FL", "CURIOSITY_GREEN")
    elif safest_direction == 'NE':
        return ("LIDAR_DIAG_FR", "CURIOSITY_GREEN")
    elif safest_direction in ['W', 'SW']:
        return ("LIDAR_PIVOT_L", "THINKING_BLUE")
    elif safest_direction in ['E', 'SE']:
        return ("LIDAR_PIVOT_R", "THINKING_BLUE")
    elif safest_direction == 'S':
        # Turn around
        return ("LIDAR_PIVOT_R", "ALERT_RED")
        
    return (None, "IDLE_WHITE")
