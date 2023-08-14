import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.core.utils import stage

CAMERA_STAGE_PATH = "/World/Camera"
ROS_CAMERA_GRAPH_PATH = "/ROS2_Camera"

#Create a new graph with the path /ActionGraph
keys = og.Controller.Keys
og.Controller.edit(
    {
        "graph_path": ROS_CAMERA_GRAPH_PATH, 
        "evaluator_name": "execution",
    },
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
            ("getRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
            ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
            ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("cameraHelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "createViewport.inputs:execIn"),
            ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
            ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
            ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
            ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
            ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
            ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
            ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
        ],
        keys.SET_VALUES: [
            ("createViewport.inputs:viewportId", 0),
            ("createViewport.inputs:name", "NewViewport"),
            ("cameraHelperRgb.inputs:frameId", "sim_camera"),
            ("cameraHelperRgb.inputs:topicName", "rgb"),
            ("cameraHelperRgb.inputs:type", "rgb"),
            ("cameraHelperInfo.inputs:frameId", "sim_camera"),
            ("cameraHelperInfo.inputs:topicName", "camera_info"),
            ("cameraHelperInfo.inputs:type", "camera_info"),
            ("cameraHelperDepth.inputs:frameId", "sim_camera"),
            ("cameraHelperDepth.inputs:topicName", "depth"),
            ("cameraHelperDepth.inputs:type", "depth"),
        ],
    },
)

set_targets(
    prim=stage.get_current_stage().GetPrimAtPath(ROS_CAMERA_GRAPH_PATH + "/setCamera"),
    attribute="inputs:cameraPrim",
    target_prim_paths=[CAMERA_STAGE_PATH],
)