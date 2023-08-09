from omni.isaac.dynamic_control import _dynamic_control
dc = _dynamic_control.acquire_dynamic_control_interface()

# Get a handle to the Franka articulation
# This handle will automatically update if simulation is stopped and restarted
art = dc.get_articulation("/UR16e")

# Get a specific degree of freedom on an articulation
dof_ptr = dc.find_articulation_dof(art, "shoulder_lift_joint")

# get_dof_state는 하나의 joint만 얻을 수 있음 dof_ptr 대신 art 들어갈수없음
# _dynamic_control.STATE_POS = 1
# _dynamic_control.STATE_ALL = 7

# print position for the degree of freedom
dof_state = dc.get_dof_state(dof_ptr,7)
print(dof_state.pos)

# or print position for the degree of greedom
dof_state = dc.get_dof_state(dof_ptr,1)
print(dof_state)

# get articulation dof pos
dof_ptrs = dc.get_articulation_dof_states(art,1)
print(dof_ptrs[1])
