import os
import time
from rplidar import RPLidar

# Configuration
PORT_NAME = '/dev/ttyUSB0'
MAX_RANGE = 12000  # Default 12 meters if completely clear
SAFE_DISTANCE = 500  # 500 mm (0.5 meters) - anything closer is a warning

def get_sector(angle):
    """Maps a 0-360 degree angle to one of 8 compass directions."""
    if angle >= 337.5 or angle < 22.5:   return 'N'  
    elif 22.5 <= angle < 67.5:           return 'NE' 
    elif 67.5 <= angle < 112.5:          return 'E'  
    elif 112.5 <= angle < 157.5:         return 'SE' 
    elif 157.5 <= angle < 202.5:         return 'S'  
    elif 202.5 <= angle < 247.5:         return 'SW' 
    elif 247.5 <= angle < 292.5:         return 'W'  
    elif 292.5 <= angle < 337.5:         return 'NW' 

def main():
    print(f"Connecting to LiDAR on {PORT_NAME}...")
    # Explicitly define baudrate and timeout to stabilize connection
    lidar = RPLidar(PORT_NAME, baudrate=115200, timeout=3)
    
    # --- BUFFER CLEARING LOGIC ---
    print("Flushing leftover data from serial buffer...")
    try:
        lidar.stop()
        lidar.stop_motor()
        lidar.clean_input()  # Drops all old bytes from the buffer
        time.sleep(1)        # Give the hardware a second to settle
    except Exception as e:
        print(f"Initial cleanup skipped: {e}")
    # -----------------------------
    
    try:
        print("Starting motor and reading scans...")
        # iter_scans() yields one full 360-degree rotation in an infinite loop
        for scan_count, scan in enumerate(lidar.iter_scans()):
            
            # Dictionary to hold all distance readings for this rotation
            sector_data = { 'N': [], 'NE': [], 'E': [], 'SE': [], 'S': [], 'SW': [], 'W': [], 'NW': [] }
            
            # Sort the raw data into sectors
            for (_, angle, distance) in scan:
                if distance > 0: # 0 means invalid/out-of-bounds
                    sector = get_sector(angle)
                    sector_data[sector].append(distance)
            
            # Find the CLOSEST object in each sector (better for collision avoidance than average)
            closest_obstacles = {}
            for sector, distances in sector_data.items():
                if len(distances) > 0:
                    closest_obstacles[sector] = min(distances)
                else:
                    closest_obstacles[sector] = MAX_RANGE
            
            # Find the safest path (the sector where the closest obstacle is furthest away)
            best_direction = max(closest_obstacles, key=closest_obstacles.get)
            
            # --- HUMAN READABLE DEBUG DASHBOARD ---
            # Clear the terminal screen so the output stays in one place
            os.system('clear' if os.name == 'posix' else 'cls')
            
            print("=== RPLIDAR LIVE NAVIGATION DASHBOARD ===")
            print(f"Scan Count: {scan_count} | Data Points: {len(scan)}")
            print("Press Ctrl+C to exit.\n")
            
            print(f"{'SECTOR':<6} | {'CLOSEST OBSTACLE':<18} | {'STATUS'}")
            print("-" * 50)
            
            # Print in a logical layout (Front row, Middle row, Back row)
            for s in ['NW', 'N', 'NE', 'W', 'E', 'SW', 'S', 'SE']:
                dist = int(closest_obstacles[s])
                
                # Assign a status label based on proximity
                if dist == MAX_RANGE:
                    status = "[ EMPTY ]"
                elif dist < SAFE_DISTANCE:
                    status = "[ WARNING: OBSTACLE ]"
                else:
                    status = "[ CLEAR ]"
                    
                print(f"{s:<6} | {dist:>7} mm         | {status}")
            
            print("-" * 50)
            print(f"--> SAFEST MOVE DIRECTION: [ {best_direction} ] <--")
            print("=" * 50)
            
    except KeyboardInterrupt:
        print("\nUser pressed Ctrl+C. Stopping...")
    finally:
        # Crucial: Always stop the motor and disconnect, otherwise the port gets locked again
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("Hardware disconnected safely.")

if __name__ == '__main__':
    main()