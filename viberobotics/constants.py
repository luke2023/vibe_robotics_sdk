from enum import Enum
from pathlib import Path
import numpy as np

class ControlMode(Enum):
    NONE = 0
    PD_STAND = 1
    RL = 2

ROOT_DIR = Path(__file__).parent
ASSET_DIR = ROOT_DIR / "assets"
CONFIG_DIR = ROOT_DIR / "configs"

CALIBRATION_FILE = CONFIG_DIR / "zero_position.csv"

SIGN_CHANGE_MOTORS_LEG_ONLY = np.array([
    1, -1, -1, -1, 1,
    1, -1, -1, 1, 1
])
