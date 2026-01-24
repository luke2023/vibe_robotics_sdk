import numpy as np

import pinocchio as pin
import hppfcl as fcl
import numpy as np
import sys
 
from pinocchio.visualize import MeshcatVisualizer


pc_xyz = np.load("pc_xyz.npy")
robot = pin.RobotWrapper.BuildFromMJCF(
    filename="/home/danielchen09/dc/vibe/viberobotics-python/viberobotics/assets/mujoco/SundayA1_full_2dof/robot.xml",
    root_joint=None,
)
pin.forwardKinematics(robot.model, robot.data, np.zeros(23,))
pin.updateFramePlacements(robot.model, robot.data)
