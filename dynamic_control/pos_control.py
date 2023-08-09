from omni.isaac.dynamic_control import _dynamic_control
import numpy as np

dc = _dynamic_control.acquire_dynamic_control_interface()
art = dc.get_articulation("/UR16e")

# Call this each frame of simulation step if the state of the articulation is changing
dc.wake_up_articulation(art)
joint_angles = [np.random.rand(6) * 2 - 1]

print(joint_angles)

#dc.set_articulation_dof_position_targets(articulation, joint_angles)
