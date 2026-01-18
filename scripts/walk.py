from viberobotics.motor.motor_controller_manager import MotorControllerManager
from viberobotics.configs.config import load_config

config = load_config("viberobotics/configs/sundaya1_real_config_leg_only.yaml")
motor_manager = MotorControllerManager(config.real_config.motor_controllers, mode=2)

