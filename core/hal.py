# core/hal.py
import os
from dotenv import load_dotenv

load_dotenv()

ENV = os.getenv("ENV", "MAC_DEV")

if ENV == "PROD_JETSON":
    from core.hal_jetson import JetsonCamera as Camera
    from core.hal_jetson import MotorController
    from core.hal_jetson import Speaker
    from core.hal_jetson import JetsonLiDAR as LiDAR
else:
    from core.hal_mac import MacCamera as Camera
    from core.hal_mac import MotorController
    from core.hal_mac import Speaker
    from core.hal_mac import MockLiDAR as LiDAR

print(f"🔧 Loaded Hardware Abstraction Layer for: {ENV}")