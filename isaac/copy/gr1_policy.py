
from typing import Optional

import numpy as np
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.storage.native import get_assets_root_path

joint_dof_names = ['left_hip_roll_joint', 'right_hip_roll_joint', 'waist_yaw_joint', 'left_hip_yaw_joint', 'right_hip_yaw_joint', 'waist_pitch_joint', 'left_hip_pitch_joint', 'right_hip_pitch_joint', 'waist_roll_joint', 'left_knee_pitch_joint', 'right_knee_pitch_joint', 'head_roll_joint', 'left_shoulder_pitch_joint', 'right_shoulder_pitch_joint', 'left_ankle_pitch_joint', 'right_ankle_pitch_joint', 'head_pitch_joint', 'left_shoulder_roll_joint', 'right_shoulder_roll_joint', 'left_ankle_roll_joint', 'right_ankle_roll_joint', 'head_yaw_joint', 'left_shoulder_yaw_joint', 'right_shoulder_yaw_joint', 'left_elbow_pitch_joint', 'right_elbow_pitch_joint', 'left_wrist_yaw_joint', 'right_wrist_yaw_joint', 'left_wrist_roll_joint', 'right_wrist_roll_joint', 'left_wrist_pitch_joint', 'right_wrist_pitch_joint', 'L_index_proximal_joint', 'L_middle_proximal_joint', 'L_pinky_proximal_joint', 'L_ring_proximal_joint', 'L_thumb_proximal_yaw_joint', 'R_index_proximal_joint', 'R_middle_proximal_joint', 'R_pinky_proximal_joint', 'R_ring_proximal_joint', 'R_thumb_proximal_yaw_joint', 'L_index_intermediate_joint', 'L_middle_intermediate_joint', 'L_pinky_intermediate_joint', 'L_ring_intermediate_joint', 'L_thumb_proximal_pitch_joint', 'R_index_intermediate_joint', 'R_middle_intermediate_joint', 'R_pinky_intermediate_joint', 'R_ring_intermediate_joint', 'R_thumb_proximal_pitch_joint', 'L_thumb_distal_joint', 'R_thumb_distal_joint']

class GR1Policy(PolicyController):
    def __init__(
        self,
        prim_path: str,
        root_path: Optional[str] = None,
        name: str = "gr1",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        assets_root_path = get_assets_root_path()
        if usd_path == None:
            usd_path = assets_root_path + "/Isaac/Robots/FourierIntelligence/GR-1/GR1T2_fourier_hand_6dof/GR1T2_fourier_hand_6dof.usd"
        super().__init__(name, prim_path, root_path, usd_path, position, orientation)
        self.load_policy(
            "/fourier-sim/ros_ws/isaac/fourier.pt",
            "/fourier-sim/ros_ws/isaac/fourier.yaml",
        )

        self._action_scale = 0.5
        self._previous_action = np.zeros(7)
        self._policy_counter = 0

        self.object_position = np.array([-0.45, 0.45, 1.0541])
        self.object_quat = np.array([1, 0, 0, 0])

        target_names = ["left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_pitch_joint", "left_wrist_yaw_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint"]
        joint_list = joint_dof_names # self.robot.dof_names

        indexes = []
        for joint_name in target_names:
            index = joint_list.index(joint_name)
            indexes.append(index)

        self._controlled_joint_ids = indexes
    
    def _compute_observation(self, command):
        current_joint_pos = self.robot.get_joint_positions() - self.default_pos
        current_joint_vel = self.robot.get_joint_velocities()
    
        obs = np.zeros(28, dtype=np.float32)

        return obs
    
    def forward(self, dt, command):
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            normalised_action = np.tanh(self._compute_action(obs) / 2)
            
            self._previous_action = self.action.copy()

        joint_pos_array = self.default_pos.tolist()
        filtered_joints = [
            self.default_pos[i] for i in self._controlled_joint_ids if i < len(joint_pos_array)
        ]
        action = ArticulationAction(joint_positions=np.array(filtered_joints) + (self.action * self._action_scale), joint_indices=np.array(self._controlled_joint_ids))
        self.robot.apply_action(action)

        self._policy_counter += 1
    
    def initialize(self):
        # Initialize articulation only (no gain setup, no wrong action mapping)
        self.robot.initialize()

        # Record default joint positions
        self.default_pos = self.robot.get_joint_positions()
        self.default_vel = self.robot.get_joint_velocities()

        # Action buffer sized to robot DOF
        dof = self.robot.num_dof
        self.action = np.zeros(dof)

        return True