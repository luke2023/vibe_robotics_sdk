import argparse
from viberobotics.configs.config import load_config
from viberobotics.motor.motor_controller_manager import MotorControllerManager

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default='sundaya1_real_config_short.yaml')
    args = parser.parse_args()
    
    cfg = load_config(args.config)
    
    motor_controller_manager = MotorControllerManager(
        n_motors=cfg.real_config.n_motors,
        motor_mapping=cfg.real_config.motor_controllers,
        calibration_file=cfg.real_config.calibration_file,
        remote=True,
        sender=False,
    )
    motor_controller_manager.receiver_loop()
    