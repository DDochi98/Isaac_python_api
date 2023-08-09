from omni.isaac.dynamic_control import _dynamic_control

dc = _dynamic_control.acquire_dynamic_control_interface()
art = dc.get_articulation("/UR16e")

# Get information about the structure of the articulation
num_joints = dc.get_articulation_joint_count(art)
num_dofs = dc.get_articulation_dof_count(art)
num_bodies = dc.get_articulation_body_count(art)


# Get a specific degree of freedom on an articulation
# Finds actor degree-of-freedom given its name
dof_ptr = dc.find_articulation_dof(art, "shoulder_lift_joint")

# Get index in articulation DOF array, -1 on error
# Only Revolute joint, except fixed joint
# First Joint = 0
dof_index_1 = dc.find_articulation_dof_index(art, "shoulder_pan_joint")
dof_index_2 = dc.find_articulation_dof_index(art, "finger_joint")
dof_index_3 = dc.find_articulation_dof_index(art, "right_outer_knuckle_joint")

print(dof_index_1)
print(dof_index_2)
print(dof_index_3)