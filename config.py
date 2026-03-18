from pathlib import Path

# --- HARDWARE CONFIGURATION ---
COM_PORT = 'COM7' # Change to '/dev/ttyACM0' if on Linux/Raspberry Pi
BITRATE = 500000

# --- JOGGING CONFIGURATION ---
JOG_STEP = 1.0
JOG_INTERVAL = 50
JOG_DELAY = 250

# --- PATHS ---
SCRIPT_DIR = Path(__file__).parent
GLB_PATH = SCRIPT_DIR / "my-robot" / "robot.glb"
HTML_PATH = SCRIPT_DIR / "my-robot" / "viewer.html"

# --- JOINT DEFINITIONS ---
JOINT_CONFIGS = [
    {"id": 0, "can_id": 0x01, "name": "Joint 0 (Base)", "urdf_link": "base", "invert": True,  "can_enabled": True, "axis": "y", "vis_invert": True},
    {"id": 1, "can_id": 0x02, "name": "Joint 1 (Shoulder)", "urdf_link": "shoulder", "invert": False, "can_enabled": False, "axis": "x", "vis_invert": True},
    {"id": 2, "can_id": 0x03, "name": "Joint 2 (Elbow)", "urdf_link": "elbow", "invert": False, "can_enabled": False, "axis": "y", "vis_invert": False},
    {"id": 3, "can_id": 0x04, "name": "Joint 3 (Rotation)", "urdf_link": "rotation", "invert": False, "can_enabled": False, "axis": "x", "vis_invert": False},
    {"id": 4, "can_id": 0x05, "name": "Joint 4 (Wrist)", "urdf_link": "wrist", "invert": False, "can_enabled": False, "axis": "y", "vis_invert": True},
    {"id": 5, "can_id": 0x06, "name": "Joint 5 (Swivel)", "urdf_link": "swivel", "invert": False, "can_enabled": False, "axis": "x", "vis_invert": True},
]

# --- GPIO CONFIGURATION ---
GPIO_OUTPUT_PINS = [2, 3, 4, 5, 7, 8, 9, 10, 11, 12]
GPIO_INPUT_PINS = [14, 15, 16, 17, 18, 19, 20, 21, 24, 25]