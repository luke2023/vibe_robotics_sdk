from enum import Enum
import numpy as np
from dataclasses import dataclass

class WalkState(Enum):
    STAND = 0
    DSP = 1
    SSP = 2


@dataclass
class Footstep:
    x: float
    y: float
    
    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, 0.])

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
    left_foot_pos: np.ndarray
    right_foot_pos: np.ndarray
    com_pos: np.ndarray
    

class FootType(Enum):
    LEFT = 0
    RIGHT = 1

class Foot:
    def __init__(self, position: np.ndarray, size: np.ndarray):
        self.position = position
        self.size = size / 2
        
    def get_scaled_contact_area(self, scale):
        X = scale * self.size[0]
        Y = scale * self.size[1]
        v1 = np.array([X, Y, 0.]) + self.position
        v2 = np.array([-X, Y, 0.]) + self.position
        v3 = np.array([-X, -Y, 0.]) + self.position
        v4 = np.array([X, -Y, 0.]) + self.position
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