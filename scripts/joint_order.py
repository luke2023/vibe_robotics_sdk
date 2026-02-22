import mujoco

import sys

assert len(sys.argv) == 2, "Usage: python joint_order.py <mujoco_xml_path>"

model = mujoco.MjModel.from_xml_path(sys.argv[1])
print('mujoco joint order:')
for i in range(model.njnt):
    joint_name = model.joint(i).name
    print(f"Joint {i - 1}: {joint_name}")