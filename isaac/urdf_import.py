# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from isaacsim import SimulationApp

# URDF import, configuration and simulation sample
kit = SimulationApp({"renderer": "RaytracedLighting", "headless": False})
import omni.kit.commands
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.extensions import get_extension_path_from_name
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdLux, UsdPhysics

# Setting up import configuration:
status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = False
import_config.distance_scale = 1.0

# Get path to extension data:
extension_path = get_extension_path_from_name("isaacsim.asset.importer.urdf")
# Import URDF, prim_path contains the path the path to the usd prim in the stage.
status, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path="/fourier-sim/ros_ws/isaac/assets/GR1T2_fourier_hand_6dof.urdf",
    import_config=import_config,
    get_articulation_root=True,
)
# Get stage handle
stage = omni.usd.get_context().get_stage()

# Enable physics
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
# Set gravity
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(9.81)
# Set solver settings
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
physxSceneAPI.CreateEnableCCDAttr(True)
physxSceneAPI.CreateEnableStabilizationAttr(True)
physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
physxSceneAPI.CreateSolverTypeAttr("TGS")

# Add ground plane
PhysicsSchemaTools.addGroundPlane(stage, "/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, -0.25), Gf.Vec3f(0.5))

# Add lighting
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(500)

# Start simulation
omni.timeline.get_timeline_interface().play()
# perform one simulation step so physics is loaded and dynamic control works.
kit.update()
art = Articulation(prim_path)
art.initialize()

if not art.is_physics_handle_valid():
    print(f"{prim_path} is not an articulation")
else:
    print(f"Got articulation ({prim_path})")

# perform simulation
for frame in range(1000):
    kit.update()

# Shutdown and exit
omni.timeline.get_timeline_interface().stop()
kit.close()
