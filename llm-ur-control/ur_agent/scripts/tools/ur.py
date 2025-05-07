#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import rospy
from langchain.agents import tool
from sensor_msgs.msg import JointState
import time 
from geometry_msgs.msg import PoseStamped

node_initialized = False
current_joint_states = None
joint_states_received = False 
current_ef_pose = None

def initialize_node():
    global node_initialized

    if not node_initialized:
        print("Initializing ROS node...")
        rospy.init_node('ur_agent_node', anonymous=True)

        rospy.Subscriber('/joint_states', JointState, joint_state_callback)
        rospy.Subscriber('/cartesian_motion_controller/current_pose', PoseStamped, pose_callback)

        node_initialized = True
        print("Node initialized.")

def joint_state_callback(msg):
    global current_joint_states, joint_states_received
    current_joint_states = [msg.position[-1]] + list(msg.position[:-1])
    joint_states_received = True

def pose_callback(msg):
    global current_ef_pose
    current_ef_pose = msg

@tool
def retrieve_joint_states() -> str:
    """
    Retrieves the current joint states of the UR5e robot.

    :return: Joint states as a formatted string or an error message.
    """
    global current_joint_states, joint_states_received

    initialize_node()
    print("Waiting for joint states to be available...")

    try:
        timeout = 10  # seconds
        start_time = time.time()

        while not joint_states_received:
            if rospy.is_shutdown():
                return "Error: ROS shutdown before receiving joint states."
            rospy.sleep(0.1)
            if time.time() - start_time > timeout:
                return "Error: Timed out waiting for joint states to become available."

        # Sleep a little to ensure we have the latest message
        rospy.sleep(0.3)

        joint_states_str = ", ".join([f"{state:.4f}" for state in current_joint_states])
        result = f"Current joint states: [{joint_states_str}]"
        print(result)
        return result

    except Exception as e:
        error_msg = f"Error retrieving joint states: {e}"
        print(error_msg)
        return error_msg