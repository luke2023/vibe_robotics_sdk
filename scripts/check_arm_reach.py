import pinocchio as pin
import viser
from viser.extras import ViserUrdf
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R
from loop_rate_limiters import RateLimiter
import argparse
import qpsolvers
import pink
from pink import solve_ik
from pink.tasks import ComTask, FrameTask, PostureTask
from pink.visualization import start_meshcat_visualizer
import mujoco

def urdf_to_mj(q):
    return np.array(q)[[0, 
                       5, 4, 3, 2, 1,
                       11, 10, 9, 8, 7, 6,
                       16, 15, 14, 13, 12,
                       22, 21, 20, 19, 18, 17,]]
def mj_to_urdf(q):
    return np.array(q)[[0,
                       5, 4, 3, 2, 1,
                       11, 10, 9, 8, 7, 6,
                       16, 15, 14, 13, 12,
                       22, 21, 20, 19, 18, 17,]]

EE_OFFSET_LOCAL = np.array([0.04638411, -0.01473608, -0.01692857])

if __name__ == '__main__':
    server = viser.ViserServer()
    viser_urdf = ViserUrdf(
        server,
        urdf_or_path=Path('/home/danielchen09/dc/vibe/viberobotics-python/viberobotics/assets/urdf/SundayA1_full_2dof/robot.urdf'),
        load_meshes=True,
        load_collision_meshes=False,
        root_node_name="/sunday_a1"
    )
    default_qpos = np.zeros(23,)
    viser_urdf.update_cfg(np.zeros(23,))
    
    
    robot = pin.RobotWrapper.BuildFromMJCF(
        filename="/home/danielchen09/dc/vibe/viberobotics-python/viberobotics/assets/mujoco/SundayA1_full_2dof/robot.xml",
        root_joint=None,
    )
    configuration = pink.Configuration(robot.model, robot.data, default_qpos)
    pin.forwardKinematics(robot.model, robot.data, default_qpos)
    pin.updateFramePlacements(robot.model, robot.data)
    right_hand_id = robot.model.getFrameId("thumb_0112")
    right_hand_pos = robot.data.oMf[right_hand_id].translation.copy()
    right_hand_rot = R.from_matrix(robot.data.oMf[right_hand_id].rotation)
    right_hand_quat = right_hand_rot.as_quat(scalar_first=True)
    
    ee_pos = right_hand_pos + right_hand_rot.apply(EE_OFFSET_LOCAL)
    
    ee_sphere = server.scene.add_icosphere(
        "ee_pos",
        radius=0.02,
        color=(1, 0, 0),
        position=ee_pos,
    )
    
    rate = RateLimiter(frequency=0.2, warn=False)
    while True:
        rand_q = np.zeros((23,))
        rand_q[1:5] = np.random.uniform(-np.pi, np.pi, size=5)
        configuration.q = rand_q
        pin.forwardKinematics(robot.model, robot.data, rand_q)
        pin.updateFramePlacements(robot.model, robot.data)
        right_hand_id = robot.model.getFrameId("thumb_0112")
        right_hand_pos = robot.data.oMf[right_hand_id].translation.copy()
        right_hand_rot = R.from_matrix(robot.data.oMf[right_hand_id].rotation)
        right_hand_quat = right_hand_rot.as_quat(scalar_first=True)

        ee_pos = right_hand_pos + right_hand_rot.apply(EE_OFFSET_LOCAL)
        ee_sphere.position = ee_pos
        viser_urdf.update_cfg(mj_to_urdf(rand_q))
        rate.sleep()