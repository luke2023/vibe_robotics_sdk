import pinocchio as pin
import pink

import numpy as np

class Robot:
    def __init__(self, 
                 xml_path: str,
                 left_foot_name: str,
                 right_foot_name: str):
        self.left_foot_name = left_foot_name
        self.right_foot_name = right_foot_name
        
        self.robot = pin.RobotWrapper.BuildFromMJCF(
            filename=xml_path,
            root_joint=None,
        )
        self.zero_qpos = np.zeros(self.robot.model.nq)
        self.configuration = pink.Configuration(
            self.robot.model, 
            self.robot.data, 
            np.concatenate([np.array([0, 0, 0., 0, 0, 0, 1]), self.zero_qpos])
        )
        pin.forwardKinematics(self.robot.model, self.robot.data, self.configuration.q)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
    
    def get_foot_positions(self):
        fid_right = self.robot.model.getFrameId(self.right_foot_name)
        right_foot_pos = self.robot.data.oMf[fid_right].translation.copy()
        
        fid_left = self.robot.model.getFrameId(self.left_foot_name)
        left_foot_pos = self.robot.data.oMf[fid_left].translation.copy()
        
        return left_foot_pos, right_foot_pos