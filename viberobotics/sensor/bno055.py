from viberobotics.utils.math import *

import numpy as np

class BNO055:
    def __init__(self):
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.gyro = np.array([0.0, 0.0, 0.0])
        self.acc = np.array([0.0, 0.0, 0.0])
        
        
        self.forward_world = np.array([0., 1., 0.])
        self.forward = self.forward_world.copy()
        self.forward_is_init = False
        self.xy_ori_offset = 0.
    
    def update(self, quat: np.ndarray, gyro: np.ndarray, acc: np.ndarray):
        new_forward = apply_quat(quat, self.forward_world)
        if self.forward_is_init:
            old_forward_xy = self.forward[[0, 1]] / np.linalg.norm(self.forward[[0, 1]])
            new_forward_xy = new_forward[[0, 1]] / np.linalg.norm(new_forward[[0, 1]])
            if np.sum(old_forward_xy * new_forward_xy) <= np.cos(120 * np.pi / 180):
                self.xy_ori_offset += np.arctan2(new_forward_xy[1], new_forward_xy[0]) - np.arctan2(old_forward_xy[1], old_forward_xy[0])
                self.xy_ori_offset = (self.xy_ori_offset + np.pi) % (2 * np.pi) - np.pi
        self.forward = new_forward
        self.forward_is_init = True
        
        quat = quat_mult(z_rot_quat(-self.xy_ori_offset), quat)
        self.quat = quat
        self.gyro = gyro * np.pi / 180
        self.acc = acc
        return self.quat.copy(), self.gyro.copy(), self.acc.copy()