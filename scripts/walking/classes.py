from dataclasses import dataclass
import numpy as np
from typing import List, Tuple, Literal, Optional

@dataclass
class Footstep:
    """
    Represents a single planned footstep in world frame.

    All coordinates in world frame:
      - forward = +Y
      - right   = +X
      - up      = +Z
    """
    foot: Literal["left", "right"]
    pos: np.ndarray      # shape (3,) [x, y, z]
    quat: np.ndarray     # shape (4,) [w, x, y, z]
    t_touchdown: float   # time when this foot becomes stance (start of SS)
    t_liftoff: Optional[float] = None     # OPTIONAL: time when this foot leaves ground, if usef
    

@dataclass
class ZMPReference:
    """
    ZMP reference trajectory in time.

    px_ref, py_ref are *world-frame* coordinates of the desired ZMP.
    phase_type is a list of strings describing the support phase for each time.
    """
    t: np.ndarray                    # shape (N,)
    px_ref: np.ndarray               # shape (N,)
    py_ref: np.ndarray               # shape (N,)
    phase_type: List[Literal["double", "single_left", "single_right"]]

@dataclass
class LIPMModel:
    """
    Discrete LIPM with jerk input for one axis (x or y).
    State: X = [pos; vel; acc]
    Input: u = jerk
    ZMP: p = C X
    """
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    dt: float
    z_c: float
    
@dataclass
class PreviewGains:
    Gi: float          # integral gain
    Gx: np.ndarray     # state feedback gain (1x3)
    Gp: np.ndarray     # preview gains (1 x NL)
    N_L: int           # horizon length

@dataclass
class CoMTrajectory:
    t: np.ndarray   # (N,)
    x: np.ndarray   # (N,) CoM right
    y: np.ndarray   # (N,) CoM forward
    z: np.ndarray   # (N,) CoM up (constant = z_c here)
    px: np.ndarray  # (N,) resulting ZMP x from LIPM
    py: np.ndarray  # (N,) resulting ZMP y from LIPM
    
@dataclass
class FootTrajectory:
    t: np.ndarray      # (N,)
    pos: np.ndarray    # (N, 3)
    quat: np.ndarray   # (N, 4)
    
@dataclass
class GaitParams:
    z_c: float                 # CoM height used in LIPM
    step_width: float          # lateral distance between feet
    foot_length: float         # approx. length (forward)
    foot_width: float          # approx. width (lateral)
    omega: float               # LIPM natural frequency sqrt(g / z_c)
    nominal_step_time: float   # suggested t_step
    ds_time: float             # suggested double-support time
    ss_time: float             # suggested single-support time
    
@dataclass
class WholeBodyTrajectory:
    t: np.ndarray          # (N,)
    q: np.ndarray          # (N, nq) desired joint configs over time