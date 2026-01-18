from viberobotics.configs.config import SundayA1Config
from viberobotics.policy.policy import Policy
from viberobotics.web.controller_web_server import ControllerWebServer
from viberobotics.constants import ControlMode
from viberobotics.utils.smoothing import EMASmoothing
from viberobotics.utils.pid import PIDController
from viberobotics.utils.math import *

import mujoco, mujoco.viewer
import numpy as np
from abc import ABC, abstractmethod
import time

class SundayA1Controller(ABC):
    def __init__(self, config: SundayA1Config):
        self.config = config
        self.sim_config = config.sim_config
        self.default_qpos = config.default_qpos
        
        self.policy = Policy(config)
        self.controller_server = ControllerWebServer(initial_mode=ControlMode.PD_STAND)
        self.controller_server.start_server()
        
        
        self.kp_torque = self.config.control_config.kp_torque
        self.kd_torque = self.config.control_config.kd_torque
    
        self.mode_map = {
            ControlMode.NONE: self.default_controller,
            ControlMode.PD_STAND: self.pd_stand_controller,
            ControlMode.RL: self.rl_controller
        }
        
        self.smoothed_target = EMASmoothing(0.2, default_value=self.default_qpos)
        self.motor_pd_controller = PIDController(
            kp=self.kp_torque,
            ki=0,
            kd=self.kd_torque,
        )
        
        self.qpos = np.zeros(len(config.default_qpos), dtype=np.float32)
        self.qvel = np.zeros(len(config.default_qpos), dtype=np.float32)
        
        self.next_inference_time = time.perf_counter()
    
    def get_sensor_values(self):
        pass
    
    def get_robot_states(self):
        pass
    
    def default_controller(self):
        return None
    
    def pd_stand_controller(self):
        return self.default_qpos
    
    def rl_controller(self):
        if time.perf_counter() < self.next_inference_time:
            return
        self.next_inference_time = time.perf_counter() + self.config.policy_config.policy_interval
        
        base_ang_vel, projected_gravity = self.get_sensor_values()
        
        control_input = self.controller_server.get_control_input()
        target_qpos = self.policy.inference(
            dof_pos=self.qpos,
            dof_vel=self.qvel,
            base_ang_vel=base_ang_vel,
            projected_gravity=projected_gravity,
            vx=control_input[0],
            vy=control_input[1],
            vyaw=control_input[2]
        )
        
        return target_qpos
    
    def reset(self):
        self.policy.reset()
        self.smoothed_target.reset()