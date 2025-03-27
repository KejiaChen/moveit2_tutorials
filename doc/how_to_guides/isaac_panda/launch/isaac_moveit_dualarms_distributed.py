# -*- coding: utf-8 -*-
# Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# Copyright (c) 2023 PickNik, LLC. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
import re
import os

import carb
import numpy as np
from pathlib import Path

# In older versions of Isaac Sim (prior to 4.0), SimulationApp is imported from
# omni.isaac.kit rather than isaacsim.
try:
    from isaacsim import SimulationApp
except:
    from omni.isaac.kit import SimulationApp

FRANKA_STAGE_PATH_1 = "/Franka_Left"
FRANKA_USD_PATH_1 = "/home/tp2/ws_humble/src/moveit2_tutorials/doc/how_to_guides/isaac_panda/assets/left_franka_flattened.usd"
FRANKA_STAGE_PATH_2 = "/Franka_Right"
FRANKA_USD_PATH_2 = "/home/tp2/ws_humble/src/moveit2_tutorials/doc/how_to_guides/isaac_panda/assets/right_franka_flattened.usd"
# FRANKA_USD_PATH = "/home/tp2/ws_humble/src/moveit2_tutorials/doc/how_to_guides/isaac_panda/assets/panda_multiple_arms.usd"
# FRANKA_USD_PATH = "/Isaac/Robots/Franka/franka_alt_fingers.usd"
# CAMERA_PRIM_PATH = f"{FRANKA_STAGE_PATH}/panda_hand/geometry/realsense/realsense_camera"
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
GRAPH_PATH = "/ActionGraph"
# REALSENSE_VIEWPORT_NAME = "realsense_viewport"

CONFIG = {"renderer": "RayTracedLighting", "headless": False}

simulation_app = SimulationApp(CONFIG)

import omni.client

# Use this flag to identify whether current release is Isaac Sim 4.5 or higher
isaac_sim_ge_4_5_version = True

# In older versions of Isaac Sim (prior to 4.5), get_version is imported from
# omni.isaac.kit rather than isaacsim.core.version.
try:
    from isaacsim.core.version import get_version
except:
    from omni.isaac.version import get_version

    isaac_sim_ge_4_5_version = False

# Check the major version number of Isaac Sim to see if it's four digits, corresponding
# to Isaac Sim 2023.1.1 or older.  The version numbering scheme changed with the
# Isaac Sim 4.0 release in 2024.
is_legacy_isaacsim = len(get_version()[2]) == 4

# More imports that need to compare after we create the app

# In older versions of Isaac Sim (prior to 4.5), modules are imported from
# omni.isaac.core rather than isaacsim.core.
try:
    from isaacsim.core.api import SimulationContext  # noqa E402
    from isaacsim.core.utils.prims import set_targets  # noqa E402
    from isaacsim.core.utils import (  # noqa E402
        extensions,
        prims,
        rotations,
        stage,
        viewports,
    )
except:
    from omni.isaac.core import SimulationContext  # noqa E402
    from omni.isaac.core.utils.prims import set_targets  # noqa E402
    from omni.isaac.core.utils import (  # noqa E402
        extensions,
        prims,
        rotations,
        stage,
        viewports,
    )

# In older versions of Isaac Sim (prior to 4.5), nucleus is imported from
# omni.isaac.core.utils rather than isaacsim.storage.native.
if isaac_sim_ge_4_5_version:
    from isaacsim.storage.native import nucleus
else:
    from omni.isaac.core.utils import nucleus  # noqa E402

from pxr import Gf, UsdGeom, UsdPhysics # noqa E402
import omni.graph.core as og  # noqa E402
import omni

# enable ROS2 bridge extension
# In older versions of Isaac Sim (prior to 4.5), the ROS 2 bridge is loaded from
# omni.isaac.ros2_bridge rather than isaacsim.ros2.bridge
if isaac_sim_ge_4_5_version:
    extensions.enable_extension("isaacsim.ros2.bridge")
else:
    extensions.enable_extension("omni.isaac.ros2_bridge")

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()
# https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5
# assets_root_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.5"

# Preparing stage
# viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))
full_background_path = assets_root_path + BACKGROUND_USD_PATH
# full_background_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Environments/Simple_Warehouse/simple_warehouse.usd"
print("✅ Final env path:", full_background_path)

# Loading the simple_room environment
stage.add_reference_to_stage(
    full_background_path, BACKGROUND_STAGE_PATH
)
# Rotate the background
background_prim = stage.get_current_stage().GetPrimAtPath(BACKGROUND_STAGE_PATH)
UsdGeom.Xformable(background_prim).AddRotateXYZOp().Set(Gf.Vec3f(0, 0, -90))  # Rotate 90 degrees around Z-axis

import omni.physx.scripts.utils as physxUtils

def create_cube_with_physics(cube_path="/Base",
                            size=1.0,
                            offset=Gf.Vec3f(0.0, 0.0, 0.0),
                            rotation=Gf.Vec3f(0.0, 0.0, 0.0),
                            dynamic=False,
                            mass=1.0):
    stage = omni.usd.get_context().get_stage()

    # Create a parent Xform for transform control
    xform_prim = UsdGeom.Xform.Define(stage, cube_path)
    
    # Apply translation and rotation to the Xform
    xform_prim.AddTranslateOp().Set(offset)
    xform_prim.AddRotateXYZOp().Set(rotation)

    # Create Cube under the Xform
    cube_geom_path = cube_path + "/Cube"
    cube_geom = UsdGeom.Cube.Define(stage, cube_geom_path)
    cube_geom.CreateSizeAttr(size)
    cube_geom.AddScaleOp().Set(Gf.Vec3f(0.8, 1.6, 1.0))

    # Apply physics
    cube_prim = stage.GetPrimAtPath(cube_geom_path)
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(cube_prim)
    rigid_api.CreateRigidBodyEnabledAttr(True)
    UsdPhysics.CollisionAPI.Apply(cube_prim)

    if not dynamic:
        # Mark it as immovable
        rigid_api.GetRigidBodyEnabledAttr().Set(False)

    physxUtils.createJoint(stage, "Fixed", None, cube_prim.GetPrim())

    print(f"Created cube at {cube_path} with size={size}, offset={offset}. dynamic={dynamic}")

# Loading the base cube
create_cube_with_physics("/Base", 1.0, Gf.Vec3f(0.09, 0, 0.5), Gf.Vec3f(0, 0, 0), dynamic=True, mass=1.0)

# Loading the franka robot USD
prims.create_prim(
    FRANKA_STAGE_PATH_1,
    "Xform",
    position=np.array([0, -0.281, 1]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 0), 90)),
    usd_path=FRANKA_USD_PATH_1,
)

prims.create_prim(
    FRANKA_STAGE_PATH_2,
    "Xform",
    position=np.array([0, 0.281, 1]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 0), 90)),
    usd_path=FRANKA_USD_PATH_2,
)

simulation_app.update()


try:
    ros_domain_id = int(os.environ["ROS_DOMAIN_ID"])
    print("Using ROS_DOMAIN_ID: ", ros_domain_id)
except ValueError:
    print("Invalid ROS_DOMAIN_ID integer value. Setting value to 0")
    ros_domain_id = 0
except KeyError:
    print("ROS_DOMAIN_ID environment variable is not set. Setting value to 0")
    ros_domain_id = 0
if isaac_sim_ge_4_5_version:
    # Create an action graph with ROS component nodes from Isaac Sim 4.5 release and higher
    try:
        og_keys_set_values = [
            ("Context.inputs:domain_id", ros_domain_id),
            
            # Set the /Franka target prim to Articulation Controller node
            ("LeftArticulationController.inputs:robotPath", FRANKA_STAGE_PATH_1),
            ("LeftPublishJointState.inputs:topicName", "left_isaac_joint_states"),
            ("LeftSubscribeJointState.inputs:topicName", "left_isaac_joint_commands"),

            ("RightArticulationController.inputs:robotPath", FRANKA_STAGE_PATH_2),
            ("RightPublishJointState.inputs:topicName", "right_isaac_joint_states"),
            ("RightSubscribeJointState.inputs:topicName", "right_isaac_joint_commands"),

            ("createViewport.inputs:viewportId", 1),
        ]

        # In older versions of Isaac Sim, the articulation controller node contained a
        # "usePath" checkbox input that should be enabled.
        if is_legacy_isaacsim:
            og_keys_set_values.insert(
                1, ("LeftArticulationController.inputs:usePath", True)
            )
            og_keys_set_values.insert(
                2, ("RightArticulationController.inputs:usePath", True)
            )

        og.Controller.edit(
            {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),

                    ("LeftPublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    (
                        "LeftSubscribeJointState",
                        "isaacsim.ros2.bridge.ROS2SubscribeJointState",
                    ),
                    (
                        "LeftArticulationController",
                        "isaacsim.core.nodes.IsaacArticulationController",
                    ),

                    ("RightPublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    (
                        "RightSubscribeJointState",
                        "isaacsim.ros2.bridge.ROS2SubscribeJointState",
                    ),
                    (
                        "RightArticulationController",
                        "isaacsim.core.nodes.IsaacArticulationController",
                    ),

                    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ("OnTick", "omni.graph.action.OnTick"),
                    # ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                    # (
                    #     "getRenderProduct",
                    #     "isaacsim.core.nodes.IsaacGetViewportRenderProduct",
                    # ),
                ],
                og.Controller.Keys.CONNECT: [
                    (
                        "OnImpulseEvent.outputs:execOut",
                        "LeftPublishJointState.inputs:execIn",
                    ),
                    (
                        "OnImpulseEvent.outputs:execOut",
                        "LeftSubscribeJointState.inputs:execIn",
                    ),
                    (  
                        "OnImpulseEvent.outputs:execOut",
                        "RightPublishJointState.inputs:execIn",
                    ),
                    (
                        "OnImpulseEvent.outputs:execOut",
                        "RightSubscribeJointState.inputs:execIn",
                    ),

                    ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),

                    (
                        "OnImpulseEvent.outputs:execOut",
                        "LeftArticulationController.inputs:execIn",
                    ),
                    (
                        "OnImpulseEvent.outputs:execOut",
                        "RightArticulationController.inputs:execIn",
                    ),
 
                    ("Context.outputs:context", "LeftPublishJointState.inputs:context"),
                    ("Context.outputs:context", "LeftSubscribeJointState.inputs:context"),
                    ("Context.outputs:context", "RightPublishJointState.inputs:context"),
                    ("Context.outputs:context", "RightSubscribeJointState.inputs:context"),

                    ("Context.outputs:context", "PublishClock.inputs:context"),

                    (
                        "ReadSimTime.outputs:simulationTime",
                        "LeftPublishJointState.inputs:timeStamp",
                    ),
                    (   
                        "ReadSimTime.outputs:simulationTime",
                        "RightPublishJointState.inputs:timeStamp",
                    ),
                    
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishClock.inputs:timeStamp",
                    ),

                    (
                        "LeftSubscribeJointState.outputs:jointNames",
                        "LeftArticulationController.inputs:jointNames",
                    ),
                    (
                        "LeftSubscribeJointState.outputs:positionCommand",
                        "LeftArticulationController.inputs:positionCommand",
                    ),
                    (
                        "LeftSubscribeJointState.outputs:velocityCommand",
                        "LeftArticulationController.inputs:velocityCommand",
                    ),
                    (
                        "LeftSubscribeJointState.outputs:effortCommand",
                        "LeftArticulationController.inputs:effortCommand",
                    ),
                    (   
                        "RightSubscribeJointState.outputs:jointNames",
                        "RightArticulationController.inputs:jointNames",
                    ),
                    (
                        "RightSubscribeJointState.outputs:positionCommand",
                        "RightArticulationController.inputs:positionCommand",
                    ),
                    (
                        "RightSubscribeJointState.outputs:velocityCommand",
                        "RightArticulationController.inputs:velocityCommand",
                    ),
                    (
                        "RightSubscribeJointState.outputs:effortCommand",
                        "RightArticulationController.inputs:effortCommand",
                    ),

                    # ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                    # (
                    #     "createViewport.outputs:execOut",
                    #     "getRenderProduct.inputs:execIn",
                    # ),
                    # (
                    #     "createViewport.outputs:viewport",
                    #     "getRenderProduct.inputs:viewport",
                    # ),
                ],
                og.Controller.Keys.SET_VALUES: og_keys_set_values,
            },
        )
    except Exception as e:
        print(e)

else:

    # Create an action graph with ROS component nodes from a pre Isaac Sim 4.5 release
    try:
        og_keys_set_values = [
            ("Context.inputs:domain_id", ros_domain_id),
            # Set the /Franka target prim to Articulation Controller node
            ("ArticulationController.inputs:robotPath", FRANKA_STAGE_PATH_1),
            ("PublishJointState.inputs:topicName", "isaac_joint_states"),
            ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
            # ("createViewport.inputs:name", REALSENSE_VIEWPORT_NAME),
            ("createViewport.inputs:viewportId", 1),
            # ("cameraHelperRgb.inputs:frameId", "sim_camera"),
            # ("cameraHelperRgb.inputs:topicName", "rgb"),
            # ("cameraHelperRgb.inputs:type", "rgb"),
            # ("cameraHelperInfo.inputs:frameId", "sim_camera"),
            # ("cameraHelperInfo.inputs:topicName", "camera_info"),
            # ("cameraHelperInfo.inputs:type", "camera_info"),
            # ("cameraHelperDepth.inputs:frameId", "sim_camera"),
            # ("cameraHelperDepth.inputs:topicName", "depth"),
            # ("cameraHelperDepth.inputs:type", "depth"),
        ]

        # In older versions of Isaac Sim, the articulation controller node contained a
        # "usePath" checkbox input that should be enabled.
        if is_legacy_isaacsim:
            og_keys_set_values.insert(
                1, ("ArticulationController.inputs:usePath", True)
            )

        og.Controller.edit(
            {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    (
                        "PublishJointState",
                        "omni.isaac.ros2_bridge.ROS2PublishJointState",
                    ),
                    (
                        "SubscribeJointState",
                        "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
                    ),
                    (
                        "ArticulationController",
                        "omni.isaac.core_nodes.IsaacArticulationController",
                    ),
                    ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                    (
                        "getRenderProduct",
                        "omni.isaac.core_nodes.IsaacGetViewportRenderProduct",
                    ),
                    # (
                    #     "setCamera",
                    #     "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct",
                    # ),
                    # ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    # ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    # ("cameraHelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ],
                og.Controller.Keys.CONNECT: [
                    (
                        "OnImpulseEvent.outputs:execOut",
                        "PublishJointState.inputs:execIn",
                    ),
                    (
                        "OnImpulseEvent.outputs:execOut",
                        "SubscribeJointState.inputs:execIn",
                    ),
                    ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                    (
                        "OnImpulseEvent.outputs:execOut",
                        "ArticulationController.inputs:execIn",
                    ),
                    ("Context.outputs:context", "PublishJointState.inputs:context"),
                    ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                    ("Context.outputs:context", "PublishClock.inputs:context"),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishJointState.inputs:timeStamp",
                    ),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishClock.inputs:timeStamp",
                    ),
                    (
                        "SubscribeJointState.outputs:jointNames",
                        "ArticulationController.inputs:jointNames",
                    ),
                    (
                        "SubscribeJointState.outputs:positionCommand",
                        "ArticulationController.inputs:positionCommand",
                    ),
                    (
                        "SubscribeJointState.outputs:velocityCommand",
                        "ArticulationController.inputs:velocityCommand",
                    ),
                    (
                        "SubscribeJointState.outputs:effortCommand",
                        "ArticulationController.inputs:effortCommand",
                    ),
                    ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                    (
                        "createViewport.outputs:execOut",
                        "getRenderProduct.inputs:execIn",
                    ),
                    (
                        "createViewport.outputs:viewport",
                        "getRenderProduct.inputs:viewport",
                    ),
                    # ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                    # (
                    #     "getRenderProduct.outputs:renderProductPath",
                    #     "setCamera.inputs:renderProductPath",
                    # ),
                    # ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                    # ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                    # ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                    # ("Context.outputs:context", "cameraHelperRgb.inputs:context"),
                    # ("Context.outputs:context", "cameraHelperInfo.inputs:context"),
                    # ("Context.outputs:context", "cameraHelperDepth.inputs:context"),
                    # (
                    #     "getRenderProduct.outputs:renderProductPath",
                    #     "cameraHelperRgb.inputs:renderProductPath",
                    # ),
                    # (
                    #     "getRenderProduct.outputs:renderProductPath",
                    #     "cameraHelperInfo.inputs:renderProductPath",
                    # ),
                    # (
                    #     "getRenderProduct.outputs:renderProductPath",
                    #     "cameraHelperDepth.inputs:renderProductPath",
                    # ),
                ],
                og.Controller.Keys.SET_VALUES: og_keys_set_values,
            },
        )
    except Exception as e:
        print(e)

    simulation_app.update()

if isaac_sim_ge_4_5_version:
    # Setting the /Franka target prim to Publish JointState node
    set_targets(
        prim=stage.get_current_stage().GetPrimAtPath("/ActionGraph/LeftPublishJointState"),
        attribute="inputs:targetPrim",
        target_prim_paths=[FRANKA_STAGE_PATH_1],
    )
    set_targets(
        prim=stage.get_current_stage().GetPrimAtPath("/ActionGraph/RightPublishJointState"),
        attribute="inputs:targetPrim",
        target_prim_paths=[FRANKA_STAGE_PATH_2],
    )
else:
    from omni.isaac.core_nodes.scripts.utils import set_target_prims  # noqa E402

    # Setting the /Franka target prim to Publish JointState node
    set_target_prims(
        primPath="/ActionGraph/PublishJointState", targetPrimPaths=[FRANKA_STAGE_PATH_1]
    )

# Fix camera settings since the defaults in the realsense model are inaccurate
# realsense_prim = camera_prim = UsdGeom.Camera(
#     stage.get_current_stage().GetPrimAtPath(CAMERA_PRIM_PATH)
# )
# realsense_prim.GetHorizontalApertureAttr().Set(20.955)
# realsense_prim.GetVerticalApertureAttr().Set(15.7)
# realsense_prim.GetFocalLengthAttr().Set(18.8)
# realsense_prim.GetFocusDistanceAttr().Set(400)

# set_targets(
#     prim=stage.get_current_stage().GetPrimAtPath(GRAPH_PATH + "/setCamera"),
#     attribute="inputs:cameraPrim",
#     target_prim_paths=[CAMERA_PRIM_PATH],
# )

# Run app update for multiple frames to re-initialize the ROS action graph after setting new prim inputs
simulation_app.update()
simulation_app.update()

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()

simulation_context.play()
simulation_app.update()

# Dock the second camera window
# viewport = omni.ui.Workspace.get_window("Viewport")
# rs_viewport = omni.ui.Workspace.get_window(REALSENSE_VIEWPORT_NAME)
# rs_viewport.dock_in(viewport, omni.ui.DockPosition.RIGHT)


while simulation_app.is_running():

    # Run with a fixed step size
    simulation_context.step(render=True)

    # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
    og.Controller.set(
        og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True
    )

simulation_context.stop()
simulation_app.close()
