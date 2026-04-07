from enum import Enum
import numpy as np
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R

class WalkState(Enum):
    STAND = 0
    DSP = 1
    SSP = 2


@dataclass
class SE3:
    T: np.ndarray
    
    @property
    def position(self) -> np.ndarray:
        return self.T[:3, 3]
    
    @property
    def rotation(self) -> R:
        return R.from_matrix(self.T[:3, :3])
    
    def translated(self, offset: np.ndarray) -> "SE3":
        """Return a copy translated by offset (rotation unchanged)."""
        T2 = self.T.copy()
        T2[:3, 3] = T2[:3, 3] + offset.reshape(3)
        return SE3(T=T2)

@dataclass
class Footstep(SE3):
    pass

@dataclass
class RobotConfig:
    xml_path: str
    left_foot_name: str
    right_foot_name: str
    foot_size: np.ndarray

@dataclass
class WalkConfig:
    ssp_duration: float
    dsp_duration: float
    step_length: float
    mpc_mode: str = "coupled"
    
@dataclass
class RobotParams:
    com: np.ndarray
    foot_spred: float
    foot_size: np.ndarray
    left_foot_offset: np.ndarray
    right_foot_offset: np.ndarray
    foot_y: float = 0.0

@dataclass
class IKTarget:
    left_foot_pose: SE3
    right_foot_pose: SE3
    com_pos: np.ndarray
    heading: float
    

class FootType(Enum):
    LEFT = 0
    RIGHT = 1

class Foot(SE3):
    def __init__(self, side: FootType, position: np.ndarray, size: np.ndarray):
        self.side = side
        self.T = np.eye(4)
        self.T[:3, 3] = position
        self.size = size / 2
    
    def get_scaled_contact_area(self, scale):
        X = scale * self.size[0]
        Y = scale * self.size[1]
        def tr(p):
            return np.dot(self.T, np.array([p[0], p[1], p[2], 1.]))[:3]
        v1 = tr(np.array([X, Y, 0.]))
        v2 = tr(np.array([-X, Y, 0.]))
        v3 = tr(np.array([-X, -Y, 0.]))
        v4 = tr(np.array([X, -Y, 0.]))
        return np.array([v1, v2, v3, v4])

class PointMass:
    def __init__(self, position: np.ndarray):
        self.position = position
        self.velocity = np.zeros(3)
        self.acceleration = np.zeros(3)
        
    def integrate_constant_jerk(self, pddd, dt):
        self.position = self.position + dt * (
            self.velocity + .5 * dt * (self.acceleration + dt * pddd / 3.))
        self.velocity = self.velocity + dt * (self.acceleration + dt * pddd / 2.)
        self.acceleration = self.acceleration + dt * pddd

class Stance:
    def __init__(self, left_foot, right_foot, com):
        self.left_foot = left_foot
        self.right_foot = right_foot
        self.com = com