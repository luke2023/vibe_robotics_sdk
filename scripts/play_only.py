from viberobotics.motor.motor_controller_manager import MotorControllerManager
import json
import time

def play(cfg, recording_file):
    motor_manager = MotorControllerManager(cfg.real_config.n_motors, cfg.real_config.motor_controllers, cfg.real_config.calibration_file, mode=0)
    frames = json.load(open(recording_file, 'r'))
    for frame in frames:
        q = frame['q']
        v = frame['v']
        a = frame['a']
        interval = frame['interval']
        motor_manager.set_positions(q, v, a)
        time.sleep(interval)
        
if __name__ == '__main__':
    from viberobotics.configs.config import load_config
    config = load_config('sundaya1_real_config_short.yaml')
    play(config, 'lay_down_motion.json')