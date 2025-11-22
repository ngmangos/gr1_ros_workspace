#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import rclpy
import torch
import numpy as np
import io
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState, Imu
from message_filters import Subscriber, TimeSynchronizer
from tf2_ros import Buffer, TransformListener
import tf2_ros



class Gr1RosController(Node):
    """Fullbody controller for GR1 humanoid robot.
    
    This ROS 2 node subscribes to velocity commands and synchronized joint/IMU
    data, processes the data through a neural network policy, and publishes
    joint commands for controlling the GR1 robot's movements.
    """

    def __init__(self):
        """Initialize the gr1 ros controller node."""
        super().__init__('gr1_ros_controller')

        # Declare and set parameters
        self.declare_parameter('publish_period_ms', 5)
        self.declare_parameter('policy_path', 'src/gr1_ros/policy/fourier.pt')
        self.set_parameters(
            [rclpy.parameter.Parameter(
                'use_sim_time', 
                rclpy.Parameter.Type.BOOL, 
                True
            )]
        )

        self._logger = self.get_logger()
        
        # Configure QoS profile for simulation
        sim_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_ALL,
        )

        # Create subscription for velocity commands
        self._cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            qos_profile=10)

        # Create publisher for joint commands
        self._joint_publisher = self.create_publisher(
            JointState,
            '/joint_command',
            qos_profile=sim_qos_profile)

        # Setup synchronized subscribers for IMU and joint state data
        self._imu_sub_filter = Subscriber(
            self,
            Imu,
            '/imu',
            qos_profile=sim_qos_profile,
        )
        self._joint_states_sub_filter = Subscriber(
            self,
            JointState,
            '/joint_states',
            qos_profile=sim_qos_profile,
        )
        queue_size = 10
        subscribers = [self._joint_states_sub_filter, self._imu_sub_filter]

        # Time synchronizer to ensure joint state and IMU data are processed
        # together
        self.sync = TimeSynchronizer(subscribers, queue_size)
        self.sync.registerCallback(self._tick)

        # TF2 setup for getting link transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Load neural network policy
        self.policy_path = self.get_parameter('policy_path').value
        self.load_policy()

        # Initialize state variables
        self._joint_state = JointState()
        self._joint_command = JointState()
        self._cmd_vel = Twist()
        self._imu = Imu()
        self._action_scale = 0.5  # Scale factor for policy output
        self._previous_action = np.zeros(7)
        self._policy_counter = 0
        self._decimation = 6  # Run policy every 6 ticks to reduce computation
        self._last_tick_time = self.get_clock().now().nanoseconds * 1e-9
        self._lin_vel_b = np.zeros(3)  # Linear velocity in body frame
        self._dt = 0.0  # Time delta between ticks

        # Default joint positions representing the nominal stance
        self.default_pos = np.array([
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ])

        # Joint names in the order expected by the policy
        self.joint_names = [
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_pitch_joint",
            "left_wrist_yaw_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint"
        ]

        self._logger.info("Initializing Gr1RosController")

    def _cmd_vel_callback(self, msg):
        """Store the latest velocity command."""
        self._cmd_vel = msg

    def _get_left_hand_transform(self):
        """Get the transform of the left hand roll link in world frame.
        
        Returns:
            tuple: (success, position, quaternion) where:
                - success: bool indicating if transform was found
                - position: 3D position in world frame as numpy array [x, y, z]
                - quaternion: orientation as numpy array [w, x, y, z]
        """
        try:
            # Look up transform from world frame to left hand roll link
            transform = self.tf_buffer.lookup_transform(
                'world',  # target frame
                'left_hand_roll_link',  # source frame  
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Extract position
            pos = transform.transform.translation
            position = np.array([pos.x, pos.y, pos.z])
            
            # Extract orientation as quaternion
            quat = transform.transform.rotation
            quaternion = np.array([quat.w, quat.x, quat.y, quat.z])
            
            return True, position, quaternion
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self._logger.warn(f'Could not transform from world to left_hand_roll_link: {e}')
            return False, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion

    def _tick(self, joint_state: JointState, imu: Imu):
        """Process synchronized joint state and IMU data to generate robot commands.
        
        This method is called whenever new joint state and IMU data are available.
        It computes the policy's action and publishes the resulting joint
        commands.
        
        Args:
            joint_state: Current joint positions and velocities
            imu: Current IMU data (orientation, angular velocity, acceleration)
        """
        # Reset if time jumped backwards (most likely due to sim time reset)
        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self._last_tick_time:
            self._logger.error(
                f'{self._get_stamp_prefix()} Time jumped backwards. Resetting.'
            )
        
        # Calculate time delta since last tick
        self._dt = (now - self._last_tick_time)
        self._last_tick_time = now

        # Run the control policy
        self.forward(joint_state, imu)

        # Prepare and publish the joint command message
        self._joint_command.header.stamp = self.get_clock().now().to_msg()
        self._joint_command.name = self.joint_names
        
        # Compute final joint positions by adding scaled actions to default positions
        action_pos = self.default_pos + self.action * self._action_scale
        self._joint_command.position = action_pos.tolist()
        self._joint_command.velocity = np.zeros(len(self.joint_names)).tolist()
        self._joint_command.effort = np.zeros(len(self.joint_names)).tolist()
        self._joint_publisher.publish(self._joint_command)

    def _compute_observation(self, joint_state: JointState, imu: Imu):
        """Compute the policy observation vector from robot state.
        
        Constructs a 69-dimensional observation vector from robot sensor data:
        - Linear velocity (body frame)
        - Angular velocity (body frame)
        - Gravity direction (body frame)
        - Command velocity
        - Joint positions (relative to default)
        - Joint velocities
        - Previous action
        
        Args:
            joint_state: Current joint positions and velocities
            imu: Current IMU data
            
        Returns:
            np.ndarray: 69-dimensional observation vector for the policy
        """

        success, left_hand_pos, left_hand_quat = self._get_left_hand_transform() 
        
        # Joint states (7 positions + 7 velocities)
        current_joint_pos = np.zeros(7)
        current_joint_vel = np.zeros(7)

        # Map joint states from message to our ordered arrays
        for i, name in enumerate(self.joint_names):
            if name in joint_state.name:
                idx = joint_state.name.index(name)
                current_joint_pos[i] = joint_state.position[idx]
                current_joint_vel[i] = joint_state.velocity[idx]
        
        obs_size = 7 + 7 + 3 + 4 + 7  # Adjust this based on your actual observation structure
        obs = np.zeros(obs_size) 

        # Store joint positions relative to default pose
        obs[:7] = current_joint_pos - self.default_pos
        
        # Store joint velocities
        obs[7:14] = current_joint_vel

        obs[14:17] = np.array([-0.45, 0.45, 1.0541])
        obs[17:21] = np.array([1, 0, 0, 0])

        obs[21:24] = left_hand_pos
        obs[24:] = left_hand_quat

        return obs

    def _compute_action(self, obs):
        """Run the neural network policy to compute an action from the observation.
        
        Args:
            obs: Observation vector containing robot state information
            
        Returns:
            np.ndarray: Action vector containing joint position adjustments
        """
        # Run inference with the PyTorch policy
        with torch.no_grad():
            obs = torch.from_numpy(obs).view(1, -1).float()
            action = self.policy(obs).detach().view(-1).numpy()
        return action

    def forward(self, joint_state: JointState, imu: Imu):
        """Process sensor data and compute control actions.
        
        This combines observation computation and policy evaluation.
        The policy is run at a reduced rate (decimation) to save computation.
        
        Args:
            joint_state: Current joint positions and velocities
            imu: Current IMU data
        """
        # Compute observation from current state
        obs = self._compute_observation(joint_state, imu)

        # Run policy at reduced frequency (every _decimation ticks)
        if self._policy_counter % self._decimation == 0:
            self.action = self._compute_action(obs)
            self._previous_action = self.action.copy()
        self._policy_counter += 1

    def quat_to_rot_matrix(self, quat: np.ndarray) -> np.ndarray:
        """Convert input quaternion to rotation matrix.

        Args:
            quat (np.ndarray): Input quaternion (w, x, y, z).

        Returns:
            np.ndarray: A 3x3 rotation matrix.
        """
        q = np.array(quat, dtype=np.float64, copy=True)
        nq = np.dot(q, q)
        if nq < 1e-10:
            return np.identity(3)
        q *= np.sqrt(2.0 / nq)
        q = np.outer(q, q)
        return np.array(
            (
                (1.0 - q[2, 2] - q[3, 3], q[1, 2] - q[3, 0], q[1, 3] + q[2, 0]),
                (q[1, 2] + q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] - q[1, 0]),
                (q[1, 3] - q[2, 0], q[2, 3] + q[1, 0], 1.0 - q[1, 1] - q[2, 2]),
            ),
            dtype=np.float64,
        )

    def load_policy(self):
        """Load the neural network policy from the specified path."""
        # Load policy from file to io.BytesIO object
        with open(self.policy_path, 'rb') as f:
            buffer = io.BytesIO(f.read())
        # Load TorchScript model from buffer
        self.policy = torch.jit.load(buffer)

    def _get_stamp_prefix(self) -> str:
        """Create a timestamp prefix for logging with both system and ROS time.
        
        Returns:
            str: Formatted timestamp string with system and ROS time
        """
        now = time.time()
        now_ros = self.get_clock().now().nanoseconds / 1e9
        return f'[{now}][{now_ros}]'

    def header_time_in_seconds(self, header) -> float:
        """Convert a ROS message header timestamp to seconds.
        
        Args:
            header: ROS message header containing timestamp
            
        Returns:
            float: Time in seconds
        """
        return header.stamp.sec + header.stamp.nanosec * 1e-9


def main(args=None):
    """Main function to initialize and run the H1 fullbody controller node."""
    rclpy.init(args=args)
    node = Gr1RosController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()