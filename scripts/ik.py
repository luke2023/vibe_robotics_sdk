from viberobotics.configs.config import load_config
from viberobotics.motor.motor_controller_manager import MotorControllerManager

import pinocchio as pin
import viser
from viser.extras import ViserUrdf
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R
from loop_rate_limiters import RateLimiter

import qpsolvers
import pink
from pink import solve_ik
from pink.tasks import ComTask, FrameTask, PostureTask
from pink.visualization import start_meshcat_visualizer

def urdf_to_mj(q):
    return (np.array(q)[
        [4, 3, 2, 1, 0,
        9, 8, 7, 6, 5,]
    ] + np.array([
        0, 0.7, 0, 0, 0,
        0, -0.7, 0, 0, 0,
    ]))

def mj_to_urdf(q):
    return (np.array(q) - np.array([
        0, 0.7, 0, 0, 0,
        0, -0.7, 0, 0, 0,
    ]))[
        [4, 3, 2, 1, 0,
        9, 8, 7, 6, 5,]
    ]

RIGHT_FOOT_OFFSET = [-0.01743562, 0.0320829, -0.04188434]
LEFT_FOOT_OFFSET = [0.02135078, 0.0320829, -0.04188434]

if __name__ == "__main__":
    server = viser.ViserServer()
    
    cfg = load_config("viberobotics/configs/sundaya1_real_config_leg_only.yaml")
    
    motor_manager = MotorControllerManager(cfg.real_config.motor_controllers, mode=0)
    motor_manager.set_positions(cfg.default_qpos, 0, 5)
    
    robot = pin.RobotWrapper.BuildFromMJCF(
        filename=(Path(cfg.sim_config.asset_path).parent / "robot.xml").as_posix(),
        root_joint=None,
    )
    default_q = np.concatenate([np.array([0, 0, 0., 0, 0, 0, 1]), cfg.default_qpos])
    configuration = pink.Configuration(robot.model, robot.data, default_q)
    print(default_q)
    pin.forwardKinematics(robot.model, robot.data, default_q)
    pin.updateFramePlacements(robot.model, robot.data)
    
    fid_right = robot.model.getFrameId("foot1016")
    right_foot_pos = robot.data.oMf[fid_right].translation.copy()
    r_right = R.from_matrix(robot.data.oMf[fid_right].rotation)
    right_foot_rot = r_right.as_quat()
    
    fid_left = robot.model.getFrameId("foot1016_2")
    left_foot_pos = robot.data.oMf[fid_left].translation.copy()
    r_left = R.from_matrix(robot.data.oMf[fid_left].rotation)
    left_foot_rot = r_left.as_quat()
    
    tasks = [
        FrameTask(
            "foot1016",
            position_cost=4.0,
            orientation_cost=1.0,
        ),
        FrameTask(
            "foot1016_2",
            position_cost=4.0,
            orientation_cost=1.0,
        ),
    ]
    for task in tasks:
        task.set_target_from_configuration(configuration)
        
    solver = qpsolvers.available_solvers[0]
    if "proxqp" in qpsolvers.available_solvers:
        solver = "proxqp"
    
    
    robot_base = server.scene.add_frame("/sunday_a1", show_axes=False)
    robot_base.position = (0, 0, 0.207)
    viser_urdf = ViserUrdf(
        server,
        urdf_or_path=Path('/home/danielchen09/dc/vibe/viberobotics-python/viberobotics/assets/urdf/SundayA1_Leg_only/robot.urdf'), 
        load_meshes=True,
        load_collision_meshes=False,
        root_node_name="/sunday_a1"
    )
    viser_urdf.update_cfg(mj_to_urdf(default_q[7:]))
    
    server.scene.add_grid(
        "/grid",
        width=2,
        height=2,
        position=(
            0.0,
            0.0,
            0.0,
        ),
    )
    
    right_foot_control = server.scene.add_transform_controls(
        f"/right_foot_control",
        depth_test=False,
        scale=0.1,
        disable_axes=False,
        disable_sliders=True,
        disable_rotations=True,
        visible=True,
        position=right_foot_pos + RIGHT_FOOT_OFFSET
    )
    @right_foot_control.on_update
    def _(_) -> None:
        tasks[0].transform_target_to_world.translation = right_foot_control.position - RIGHT_FOOT_OFFSET
        
    
    left_foot_control = server.scene.add_transform_controls(
        f"/left_foot_control",
        depth_test=False,
        scale=0.1,
        disable_axes=False,
        disable_sliders=True,
        disable_rotations=True,
        visible=True,
        position=left_foot_pos + LEFT_FOOT_OFFSET
    )
    @left_foot_control.on_update
    def _(_) -> None:
        tasks[1].transform_target_to_world.translation = left_foot_control.position - LEFT_FOOT_OFFSET
    
    
    gui_reset_button = server.gui.add_button("reset")
    @gui_reset_button.on_click
    def _(_) -> None:
        print('reset')
        left_foot_control.position = left_foot_pos + LEFT_FOOT_OFFSET
        right_foot_control.position = right_foot_pos + RIGHT_FOOT_OFFSET
        tasks[0].transform_target_to_world.translation = right_foot_pos
        tasks[1].transform_target_to_world.translation = left_foot_pos
            
    
    dt = 1 / 200
    rate = RateLimiter(frequency=200.0, warn=False)
    dt = rate.period
    while True:
        velocity = solve_ik(
            configuration,
            tasks,
            dt,
            solver=solver,
            damping=0.01,
            safety_break=False,
        )
        configuration.integrate_inplace(velocity, dt)
        q = configuration.q
        # motor_manager.set_positions(q[7:], 0, 5)
        viser_urdf.update_cfg(mj_to_urdf(q[7:]))
        robot_base.position = q[:3] + np.array([0, 0, 0.207])
        robot_base.wxyz = (q[6], q[3], q[4], q[5])
        rate.sleep()