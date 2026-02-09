from viberobotics.constants import ASSET_DIR
import viser
from viser.extras import ViserUrdf
import mujoco

# mujoco joint order:
# Joint 0: body_freejoint
# Joint 1: head_yaw
# Joint 2: right_shoulder_pitch
# Joint 3: right_shoulder_roll
# Joint 4: right_hip_roll
# Joint 5: right_hip_pitch
# Joint 6: right_hip_yaw
# Joint 7: right_knee
# Joint 8: right_ankle_pitch
# Joint 9: right_ankle_roll
# Joint 10: left_shoulder_pitch
# Joint 11: left_shoulder_roll
# Joint 12: left_elbow_pitch
# Joint 13: left_wrist_yaw
# Joint 14: left_hand
# Joint 15: left_hip_roll
# Joint 16: left_hip_pitch
# Joint 17: left_hip_yaw
# Joint 18: left_knee
# Joint 19: left_ankle_pitch
# Joint 20: left_ankle_roll
# Joint 21: right_wrist0124_freejoint
# Joint 22: right_elbow_pitch
# Joint 23: right_wrist_yaw
# Joint 24: right_hand
# Joint 25: deltoid_freejoint
# urdf joint order:
# Joint 0: head_yaw
# Joint 1: right_shoulder_roll
# Joint 2: right_shoulder_pitch
# Joint 3: right_ankle_roll
# Joint 4: right_ankle_pitch
# Joint 5: right_knee
# Joint 6: right_hip_yaw
# Joint 7: right_hip_pitch
# Joint 8: right_hip_roll
# Joint 9: left_hand
# Joint 10: left_wrist_yaw
# Joint 11: left_elbow_pitch
# Joint 12: left_shoulder_roll
# Joint 13: left_shoulder_pitch
# Joint 14: left_ankle_roll
# Joint 15: left_ankle_pitch
# Joint 16: left_knee
# Joint 17: left_hip_yaw
# Joint 18: left_hip_pitch
# Joint 19: left_hip_roll

if __name__ == '__main__':
    xml_path = ASSET_DIR / 'mujoco/SundayA1_Arm_test/robot.xml'
    urdf_path = ASSET_DIR / 'urdf/SundayA1_Arm_test/robot.urdf'
    
    server = viser.ViserServer()
    viser_urdf = ViserUrdf(
        server,
        urdf_or_path=urdf_path,
        load_meshes=True,
        load_collision_meshes=True,
        root_node_name="/sunday_a1"
    )
    
    
    print('mujoco joint order:')
    model = mujoco.MjModel.from_xml_path(xml_path.as_posix())
    for i in range(model.njnt):
        joint_name = model.joint(i).name
        print(f"Joint {i}: {joint_name}")
    print('urdf joint order:')
    for i, name in enumerate(viser_urdf.get_actuated_joint_names()):
        print(f"Joint {i}: {name}")