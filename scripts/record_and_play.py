import viser
from viser.extras import ViserUrdf
from pathlib import Path
import numpy as np
from viberobotics.configs.config import load_config
from viberobotics.motor.motor_controller_manager import MotorControllerManager
import argparse
from dataclasses import dataclass
from typing import List
import time
import json
import os

def urdf_to_mj(q):
    return np.array(q)

def mj_to_urdf(q):
    return np.array(q)

@dataclass
class Frame:
    q: np.ndarray
    v: int
    a: int
    interval: float
    enable: bool = True

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--disable_ik', action='store_true', help='Use real robot')
    parser.add_argument('--config', type=str, default='sundaya1_real_config_half_2dof.yaml', help='Config file path')
    args = parser.parse_args()
    
    work_queue = []
    
    cfg = load_config(args.config)
    motor_manager = MotorControllerManager(cfg.real_config.n_motors, cfg.real_config.motor_controllers, cfg.real_config.calibration_file, mode=0)
    motor_manager.disable_torque()
    
    
    server = viser.ViserServer()
    viser_urdf = ViserUrdf(
        server,
        urdf_or_path=Path(cfg.sim_config.urdf_path),
        load_meshes=True,
        load_collision_meshes=False,
        root_node_name="/sunday_a1"
    )
    
    op_motor_ids_input = server.gui.add_text("Motor IDs affected by Disable Torque (comma separated, empty for all)", initial_value="")
    
    disable_torque_button = server.gui.add_button('Disable Torque')
    @disable_torque_button.on_click
    def _(_):
        motor_ids_str = op_motor_ids_input.value
        if motor_ids_str:
            motor_ids = [int(x) for x in motor_ids_str.split(',')]
        else:
            motor_ids = None
        work_queue.append({'type': 'disable_torque', 'q': motor_ids})
    
    home_button = server.gui.add_button('Go to Default qpos')
    @home_button.on_click
    def _(_):
        work_queue.append({'type': 'set_positions', 'q': cfg.default_qpos, 'v': 0, 'a': 5})
    
    playing = False
    current_frame_idx = 0
    next_frame_time = time.time()
    
    frames: List[Frame] = []
    frame_controls = []
    
    def add_frame(q, v, a, interval):
        idx = len(frames)
        frames.append(Frame(q, v, a, interval))
        with server.gui.add_folder(f'Frame{idx}'):
            v_input =server.gui.add_number(f'v', initial_value=v, min=0, max=1000, step=1)
            @v_input.on_update
            def _(_):
                frames[idx].v = v_input.value
            a_input = server.gui.add_number(f'a', initial_value=a, min=0, max=50, step=1)
            @a_input.on_update
            def _(_):
                frames[idx].a = a_input.value
            interval_input = server.gui.add_number(f'interval', initial_value=0.1, min=0.01, max=5.0, step=0.01)
            @interval_input.on_update
            def _(_):
                frames[idx].interval = interval_input.value
            
            go_to_frame_button = server.gui.add_button(f'Go to Frame{idx}')
            @go_to_frame_button.on_click
            def _(_):
                frame = frames[idx]
                print(idx, frame)
                work_queue.append({'type': 'set_positions', 'q': frame.q, 'v': 0, 'a': 20})
            
            enable_checkbox = server.gui.add_checkbox(f'Enable Frame{idx}', initial_value=True)
            @enable_checkbox.on_update
            def _(_):
                frames[idx].enable = enable_checkbox.value
                
            set_q_button = server.gui.add_button(f'Set Current Pose to Frame{idx}')
            @set_q_button.on_click
            def _(_):
                q, _ = motor_manager.get_state()
                frames[idx].q = q
    
    load_button = server.gui.add_button('Load Motion')
    @load_button.on_click
    def _(_):
        global frames
        if not os.path.exists('recorded_motion.json'):
            print("No recorded_motion.json file found")
            return
        if len(frames) > 0:
            print("Frames already loaded")
            return
        with open('recorded_motion.json', 'r') as f:
            motion_data = json.load(f)
        for frame_data in motion_data:
            add_frame(
                np.array(frame_data['q']),
                frame_data['v'],
                frame_data['a'],
                frame_data['interval']
            )
    
    save_button = server.gui.add_button('Save Motion')
    @save_button.on_click
    def _(_):
        motion_data = []
        for frame in frames:
            motion_data.append({
                'q': frame.q.tolist(),
                'v': frame.v,
                'a': frame.a,
                'interval': frame.interval,
                'enable': frame.enable,
            })
        with open('recorded_motion.json', 'w') as f:
            json.dump(motion_data, f, indent=4)
    
    loop_checkbox = server.gui.add_checkbox('Loop Motion', initial_value=True)
    
    play_button = server.gui.add_button('Play Motion')
    @play_button.on_click
    def _(_):
        global playing, current_frame_idx, next_frame_time
        if len(frames) == 0:
            return
        playing = not playing
        play_button.label = "Stop Motion" if playing else "Play Motion"
        if not playing:
            current_frame_idx = 0
            next_frame_time = time.time()
            work_queue.append({'type': 'disable_torque'})
        
    
    
    add_pose_button = server.gui.add_button('Add Frame')
    
    @add_pose_button.on_click
    def _(_):
        q, _ = motor_manager.get_state()
        v = 0
        a = 5
        interval = 0.1
        add_frame(q, v, a, interval)
 
    while True:
        # q, _ = motor_manager.get_state()
        # viser_urdf.update_cfg(mj_to_urdf(q))
        work = work_queue.pop(0) if len(work_queue) > 0 else None
        if work is not None:
            if work['type'] == 'disable_torque':
                motor_ids = work.get('q', None)
                motor_manager.disable_torque(motor_ids)
            elif work['type'] == 'set_positions':
                print(f"Setting positions to {work['q']} with v={work['v']} and a={work['a']}")
                motor_manager.set_positions(work['q'], work['v'], work['a'])
    
        if playing:
            current_time = time.time()
            if current_time >= next_frame_time:
                frame = frames[current_frame_idx]
                if not frame.enable:
                    # current_frame_idx = (current_frame_idx + 1) % len(frames)
                    current_frame_idx += 1
                    if current_frame_idx >= len(frames):
                        if loop_checkbox.value:
                            current_frame_idx = 0
                        else:
                            playing = False
                            play_button.label = "Play Motion"
                            current_frame_idx = 0
                    next_frame_time = current_time
                    continue
                motor_manager.set_positions(frame.q, frame.v, frame.a)
                next_frame_time = current_time + frame.interval
                current_frame_idx = (current_frame_idx + 1) % len(frames)