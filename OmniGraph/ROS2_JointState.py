import omni.graph.core as og
from omni.isaac.core_nodes.scripts.utils import set_target_prims

ROS_CAMERA_GRAPH_PATH = "/ROS2_JointStates"

og.Controller.edit(
    {"graph_path": ROS_CAMERA_GRAPH_PATH, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
            ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
            ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),

            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),

            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
            ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
        ],
        og.Controller.Keys.SET_VALUES: [
            # Providing path to /panda robot to Articulation Controller node
            # Providing the robot path is equivalent to setting the targetPrim in Articulation Controller node
            ("ArticulationController.inputs:usePath", True),
            ("ArticulationController.inputs:robotPath", "/UR16e"),
            ("SubscribeJointState.inputs:topicName","joint_states"),
            ("PublishJointState.inputs:topicName","isaac_joint_states"),
        ],
    },
)

# Setting the /panda target prim to Publish JointState node
set_target_prims(primPath= ROS_CAMERA_GRAPH_PATH + "/PublishJointState", targetPrimPaths=["/UR16e"])
