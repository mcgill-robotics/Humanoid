#!/usr/bin/env python

import rospy
from humanoid_msgs.msg import ServoCommand
import numpy as np
import torch
from stable_baselines3 import PPO
from state_generator import PPOStateGenerator, model_output_mapping, joint_limits
    
def scaleActionToJointLimits(action):
    scaled_action = []
    for i in range(len(action)):
        joint_range = joint_limits[model_output_mapping[i]]
        if action[i] < 0:
            scaled_cmd = 150 + action[i] * joint_range[0]
        else:
            scaled_cmd = 150 + action[i] * joint_range[1]
        scaled_action.append(scaled_cmd)
    return np.array(scaled_action)
    
def generateControl(_):
    obs = stateGenerator.getStateObservation()
    action, _ = ppo_agent.predict(obs, deterministic=True)
    stateGenerator.updateAction(action)
    scaled_action = scaleActionToJointLimits(action)
    
    setpoints = ServoCommand()
    setpoints.left_leg_ankle_setpoint = scaled_action[model_output_mapping.index("left_ankle_pitch")]
    setpoints.left_leg_knee_setpoint = scaled_action[model_output_mapping.index("left_knee")]
    setpoints.left_leg_hip_roll_setpoint = scaled_action[model_output_mapping.index("left_hip_roll")]
    setpoints.left_leg_hip_pitch_setpoint = scaled_action[model_output_mapping.index("left_hip_pitch")]
    setpoints.left_leg_hip_yaw_setpoint = scaled_action[model_output_mapping.index("left_hip_yaw")]
    setpoints.right_leg_ankle_setpoint = scaled_action[model_output_mapping.index("right_ankle_pitch")]
    setpoints.right_leg_knee_setpoint = scaled_action[model_output_mapping.index("right_knee")]
    setpoints.right_leg_hip_roll_setpoint = scaled_action[model_output_mapping.index("right_hip_roll")]
    setpoints.right_leg_hip_pitch_setpoint = scaled_action[model_output_mapping.index("right_hip_pitch")]
    setpoints.right_leg_hip_yaw_setpoint = scaled_action[model_output_mapping.index("right_hip_yaw")]
    setpoints.torso_roll_setpoint = scaled_action[model_output_mapping.index("torso_roll")]
    setpoints.torso_yaw_setpoint = scaled_action[model_output_mapping.index("torso_yaw")]
    setpoints.left_arm_elbow_setpoint = scaled_action[model_output_mapping.index("left_elbow")]
    setpoints.right_arm_elbow_setpoint = scaled_action[model_output_mapping.index("right_elbow")]
    setpoints.left_arm_shoulder_setpoint = scaled_action[model_output_mapping.index("left_shoulder_pitch")]
    setpoints.right_arm_shoulder_setpoint = scaled_action[model_output_mapping.index("right_shoulder_pitch")]
    
    setpoint_publisher.publish(setpoints)

if not torch.cuda.is_available(): print("WARN: Running policy on CPU!")

if __name__ == '__main__':
    rospy.init_node('policy_controller')
    setpoint_publisher = rospy.Publisher('/servosCommand', ServoCommand, queue_size=1)

    stateGenerator = PPOStateGenerator()

    ppo_agent = PPO.load(path=rospy.get_param("~model_checkpoint_path"))
    
    timer = rospy.Timer(rospy.Duration(float(rospy.get_param("~control_interval"))), generateControl)

    rospy.spin()