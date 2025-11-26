
from typing import Optional

import numpy as np
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.prims import Articulation, RigidPrim
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
        self.object_orientation = np.array([1, 0, 0, 0])


        self.object = RigidPrim(prim_paths_expr="/World/Cup/beaker", name="beaker")
        self.left_hand = Articulation(prim_paths_expr="/World/Gr1/left_hand_roll_link", name="left_hand_roll_link")
        self.left_finger = Articulation(prim_paths_expr="/World/Gr1/L_middle_intermediate_link", name="L_middle_intermediate_link")
    
    def _compute_observation(self):
        current_joint_pos = self.robot.get_joint_positions()
        target_joint_pos = current_joint_pos[self.controlled_joint_ids]

        current_joint_vel = self.robot.get_joint_velocities()
        target_joint_vel = current_joint_vel[self.controlled_joint_ids]
        
        left_hand_position, left_hand_orientation = self.left_hand.get_world_poses(indices=np.array([0]))
        object_position, object_orientation = self.object.get_world_poses(indices=np.array([0]))

        obs = np.zeros(28)

        obs[:7] = target_joint_pos
        obs[7:14] = target_joint_vel

        obs[14:17] = object_position
        obs[17:21] = object_orientation

        obs[21:24] = left_hand_position
        obs[24:] = left_hand_orientation

        noise = 1 + 0.05 * (2 * np.random.rand(len(obs)) - 1)
        print(f"Obs noise: {noise}")
        obs = obs * noise

        return obs
    
    def forward(self, dt):
        if self._policy_counter % self._decimation == 0:
            # print(f'Here policy: {self._policy_counter} dec: {self._decimation}')
            obs = self._compute_observation()
            print(obs)
            normalised_actions = np.tanh(self._compute_action(obs) / 2)
            object_position, object_orientation = self.object.get_world_poses(indices=np.array([0]))
            left_hand_position, left_hand_orientation = self.left_hand.get_world_poses(indices=np.array([0]))
            distance = np.linalg.norm(left_hand_position - object_position)
            normalised_distance = np.tanh(distance * 2)

            noise = 1 + 0.05 * (2 * np.random.rand(len(self.action)) - 1)
            self.action = normalised_actions * normalised_distance * noise
            self._previous_action = self.action.copy()

        action = ArticulationAction(joint_positions=self.action, joint_indices=np.array(self.controlled_joint_ids))
        self.robot.apply_action(action)

        self._policy_counter += 1
    
    def initialize(self):
        # Initialize articulation only (no gain setup, no wrong action mapping)
        self.robot.initialize()

        self.base_position = np.array([-0.5, 0, 0.9300000071525574])
        self.base_orientation = np.array([ 0.7071068, 0, 0, 0.7071068 ])
        self.robot.set_world_pose(position=self.base_position, orientation=self.base_orientation)

        # Record default joint positions
        num_dof = self.robot.num_dof
        self.default_pos = np.zeros(num_dof)
        self.default_vel = np.zeros(num_dof)
        self.robot.set_joint_positions(self.default_pos)
        self.robot.set_joint_velocities(self.default_vel)

        # Action buffer sized to robot DOF
        self.action = np.zeros(7)

        target_names = ["left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_pitch_joint", "left_wrist_yaw_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint"]
        joint_list = self.robot.dof_names # self.robot.dof_names

        indexes = []
        for joint_name in target_names:
            index = joint_list.index(joint_name)
            indexes.append(index)

        self.controlled_joint_ids = indexes

        return True