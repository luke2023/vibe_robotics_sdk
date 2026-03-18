import argparse
from viberobotics.configs.config import load_config
import mujoco
import yaml

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default="sundaya1_real_config_short.yaml", help='Path to config file')
    args = parser.parse_args()

    cfg = load_config(args.config)
    mujoco_path = cfg.sim_config.asset_path
    
    idx_to_name = {}
    model = mujoco.MjModel.from_xml_path(mujoco_path)
    print('Mujoco joint order:')
    for i in range(model.njnt):
        joint_name = model.joint(i).name
        idx_to_name[i] = joint_name
        
    motor_controllers = cfg.real_config.motor_controllers
    for motor in motor_controllers:
        sim_idxs = motor.sim_idxs
        old_motor_ids = motor.motor_ids
        new_motor_ids = []
        for sim_idx, old_motor_id in zip(sim_idxs, old_motor_ids):
            joint_name = idx_to_name.get(sim_idx, "Unknown")
            motor_id = input(f'Enter new motor ID for joint "{joint_name}({sim_idx})" (Enter to keep default {old_motor_id}): ')
            if motor_id.strip() == "":
                new_motor_ids.append(int(old_motor_id))
            else:
                new_motor_ids.append(int(motor_id))
        motor.motor_ids = new_motor_ids
    
    # Save the updated config back to YAML
    output_path = args.config.replace('.yaml', '_updated.yaml')
    cfg.save_to_yaml(output_path)