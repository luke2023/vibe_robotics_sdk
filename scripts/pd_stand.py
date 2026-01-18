from viberobotics.motor.motor_controller_manager import MotorControllerManager
from viberobotics.configs.config import load_config
from viberobotics.constants import CALIBRATION_FILE
from viberobotics.utils.pid import PIDController

from pprint import pprint
import numpy as np
import sys

if len(sys.argv) > 1:
    mode = int(sys.argv[1])
else:
    mode = 2

config = load_config("viberobotics/configs/sundaya1_real_config_leg_only.yaml")
pprint(config)
if mode == 2:
    kp = 500
    kd = 50

    motor_manager = MotorControllerManager(config.real_config.motor_controllers, calibration_file=CALIBRATION_FILE, mode=2)
    default_qpos = config.default_qpos
    controller = PIDController(kp, 0, kd)

    while True:
        q, dq = motor_manager.get_state()
        # print(q)
        duty = controller.update(
            setpoint=default_qpos,
            measurement=q,
            derivative=-dq,
        )
        motor_manager.set_duty(duty)
        print(duty)
elif mode == 0:
    kp = 32
    kd = 32
    cfg = config.real_config.motor_controllers
    motor_manager = MotorControllerManager(cfg, calibration_file=CALIBRATION_FILE, mode=0)
    motor_manager.set_kp_kd(kp, kd)
    default_qpos = config.default_qpos
    motor_manager.set_positions(default_qpos, 0, 5)