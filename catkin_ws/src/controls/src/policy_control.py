#!/usr/bin/env python

import rospy
from humanoid_msgs.msg import ServoCommand
import numpy as np
import torch
from stable_baselines3 import SAC
from state_generator import StateGenerator, JOINTS


def scaleActionToJointLimits(action):
    scaled_action = []
    for i in range(len(action)):
        _, joint_range = JOINTS[i]
        action_deg = 90 * action[i]
        action_limited = np.clip(action_deg, joint_range[0], joint_range[1])
        scaled_cmd = 150 + action_limited
        scaled_action.append(scaled_cmd)
    return np.array(scaled_action)


def generateControl(_):
    obs = stateGenerator.getStateObservation()
    action, _ = ppo_agent.predict(obs, deterministic=True)
    # print("ACTION: " + str(action))
    scaled_action = scaleActionToJointLimits(action)

    setpoints = ServoCommand()
    for i in range(len(JOINTS)):
        try:
            setattr(setpoints, JOINTS[i][0], scaled_action[i])
        except:
            print("Failed to set setpoint for joint: {}".format(JOINTS[i][0]))

    setpoint_publisher.publish(setpoints)


if not torch.cuda.is_available():
    print("WARN: Running policy on CPU!")

if __name__ == "__main__":
    rospy.init_node("policy_controller")
    setpoint_publisher = rospy.Publisher("/servosCommand", ServoCommand, queue_size=1)

    stateGenerator = StateGenerator()

    ppo_agent = SAC.load(path=rospy.get_param("~model_checkpoint_path"))

    timer = rospy.Timer(
        rospy.Duration(float(rospy.get_param("~control_interval"))), generateControl
    )

    rospy.spin()
