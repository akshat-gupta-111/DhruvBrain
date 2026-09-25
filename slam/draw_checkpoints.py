import json
import yaml
import sys
import os
from PIL import Image, ImageDraw, ImageFont

def annotate_map(map_name):
    # Paths to the files on your Mac desktop
    desktop = os.path.expanduser("~/Desktop")
    pgm_path = os.path.join(desktop, f"{map_name}.pgm")
    yaml_path = os.path.join(desktop, f"{map_name}.yaml")
    
    # Handle the fact that you saved it as 'incubation_map' but checkpoints are 'incubation_checkpoints'
    base_name = map_name.replace("_map", "")
    json_path = os.path.join(desktop, f"{base_name}_checkpoints.json")
    
    out_path = os.path.join(desktop, f"{map_name}_annotated.png")

    if not os.path.exists(pgm_path) or not os.path.exists(yaml_path) or not os.path.exists(json_path):
        print(f"Error: Missing files for {map_name} on your Desktop!")
        print(f"Checking for:\n- {pgm_path}\n- {yaml_path}\n- {json_path}")
        return

    # 1. Load YAML to get resolution and origin
    with open(yaml_path, 'r') as f:
        map_meta = yaml.safe_load(f)
    
    resolution = map_meta['resolution']
    origin_x = map_meta['origin'][0]
    origin_y = map_meta['origin'][1]

    # 2. Load the PGM Image
    img = Image.open(pgm_path).convert("RGBA")
    draw = ImageDraw.Draw(img)
    width, height = img.size

    # Try to load a default font
    try:
        font = ImageFont.truetype("Arial", 20)
    except IOError:
        font = ImageFont.load_default()

    # 3. Load the Checkpoints
    with open(json_path, 'r') as f:
        checkpoints = json.load(f)

    # 4. Draw each checkpoint
    for name, coords in checkpoints.items():
        world_x = coords['x']
        world_y = coords['y']

        # Convert ROS world coordinates (meters) to Pixel coordinates
        # ROS maps have origin at bottom-left, PIL draws from top-left
        px = (world_x - origin_x) / resolution
        py = height - ((world_y - origin_y) / resolution)

        # Draw a red circle (radius 5 pixels)
        r = 5
        draw.ellipse((px - r, py - r, px + r, py + r), fill="red", outline="black")
        
        # Draw the text label next to the circle
        draw.text((px + 10, py - 10), name, fill="red", font=font)
        
        print(f"Drew checkpoint '{name}' at pixel ({int(px)}, {int(py)})")

    # 5. Save the final annotated image
    img.save(out_path)
    print(f"\nSUCCESS! Saved annotated map to: {out_path}")
    print("Go to your Desktop and open it!")

if __name__ == '__main__':
    map_name = "incubation_map"
    annotate_map(map_name)
