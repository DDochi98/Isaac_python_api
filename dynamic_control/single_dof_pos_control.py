from omni.isaac.dynamic_control import _dynamic_control
import numpy as np
dc = _dynamic_control.acquire_dynamic_control_interface()
articulation = dc.get_articulation("/UR16e")
dc.wake_up_articulation(articulation)
dof_ptr = dc.find_articulation_dof(articulation, "shoulder_lift_joint")
# -1.5 rad = -85 degree
# 0.6 rad = 40 degree
dc.set_dof_position_target(dof_ptr, -1.5)