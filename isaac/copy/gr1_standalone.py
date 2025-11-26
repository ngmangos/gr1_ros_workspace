# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RaytracedLighting"}
simulation_app = SimulationApp(launch_config=CONFIG)

import carb
import numpy as np
import omni.appwindow  # Contains handle to keyboard

import sys
sys.path.append('/fourier-sim/ros_ws/isaac')
from gr1_policy import GR1Policy

from isaacsim.storage.native import get_assets_root_path

first_step = True
reset_needed = False
robots = []

import omni.usd
usd_path = "/fourier-sim/ros_ws/isaac/gr1_envless.usda"
omni.usd.get_context().open_stage(usd_path)
simulation_app.update() 
simulation_app.update() 

import omni.timeline
timeline = omni.timeline.get_timeline_interface()	

from isaacsim.core.utils.stage import is_stage_loading	

# Wait for the stage to load
while is_stage_loading():
    simulation_app.update() 

from isaacsim.core.api import World
# spawn world
my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=6 / 120)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

# initialize robot on first step, run robot advance
def on_physics_step(step_size) -> None:
    global first_step
    global reset_needed
    if first_step:
        for robot in robots:
            robot.initialize()
        first_step = False
    elif reset_needed:
        my_world.reset(True)
        reset_needed = False
        first_step = True
    else:
        for robot in robots:
            robot.forward(step_size, base_command)

# spawn robot
gr1 = GR1Policy(
    prim_path="/World/Gr1",
    name="Gr1",
    usd_path=assets_root_path + "/Isaac/Robots/FourierIntelligence/GR-1/GR1T2_fourier_hand_6dof/GR1T2_fourier_hand_6dof.usd",
    position=np.array([0, 0, 0]), # [-0.5, 0, 0.9300000071525574]
)

robots.append(gr1)

my_world.reset()
my_world.add_physics_callback("physics_step", callback_fn=on_physics_step)

# robot command
base_command = np.zeros(3)


i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped():
        reset_needed = True
    if my_world.is_playing():
        if i >= 0 and i < 80:
            # forward
            base_command = np.array([0.5, 0, 0])
        elif i >= 80 and i < 130:
            # rotate
            base_command = np.array([0.5, 0, 0.5])
        elif i >= 130 and i < 200:
            # side ways
            base_command = np.array([0, 0, 0.5])
        elif i == 200:
            i = 0
        i += 1

simulation_app.close()
