#!/usr/bin/env python

import rospy
from humanoid_msgs.msg import ServoCommand
import numpy as np
import torch
from stable_baselines3 import SAC
from state_generator import StateGenerator, JOINTS


def generateControl(_):
    obs = stateGenerator.getStateObservation()
    action, _ = ppo_agent.predict(obs, deterministic=True)
    # print("ACTION: " + str(action))
    scaled_action = action * (np.pi / 2)

    setpoints = ServoCommand()
    for i in range(len(JOINTS)):
        try:
            setattr(setpoints, JOINTS[i], scaled_action[i])
        except:
            print("Failed to set setpoint for joint: {}".format(JOINTS[i]))

    setpoint_publisher.publish(setpoints)


if not torch.cuda.is_available():
    print("WARN: Running policy on CPU!")

if __name__ == "__main__":
    rospy.init_node("policy_controller")
    setpoint_publisher = rospy.Publisher("/servos/command", ServoCommand, queue_size=1)

    stateGenerator = StateGenerator()

    ppo_agent = SAC.load(path=rospy.get_param("~model_checkpoint_path"))

    control_interval = float(rospy.get_param("~control_interval"))
    if control_interval < 0:
        while not rospy.is_shutdown():
            generateControl(None)
    else:
        timer = rospy.Timer(rospy.Duration(), generateControl)

        rospy.spin()
