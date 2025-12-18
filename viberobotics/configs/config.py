from dataclasses import dataclass
import numpy as np

@dataclass
class SundayA1SimConfig:
    dt: float = 0.002
    asset_path: str

@dataclass
class SundayA1ControlConfig:
    kp_torque: np.ndarray
    kd_torque: np.ndarray
    kp_duty: np.ndarray
    ki_duty: np.ndarray
    voltage: float = 5.
    K_t: float = 0.765
    K_e: float = 1.227
    internal_resistance: float = 2.5

@dataclass
class SundayA1PolicyConfig:
    policy_path: str
    num_actions: int = 21
    num_observations: int = 72
    horizon: int = 10

@dataclass
class SundayA1Config:
    default_qpos: np.ndarray
    sim_config: SundayA1SimConfig
    control_config: SundayA1ControlConfig
    policy_config: SundayA1PolicyConfig
    
