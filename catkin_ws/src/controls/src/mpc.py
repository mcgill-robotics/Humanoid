#!/usr/bin/env python3

import rospy
import zmq
from humanoid_msgs.msg import ServoCommand
import numpy as np
import torch
from stable_baselines3 import SAC
from state_generator import StateGenerator, JOINTS


def generateControl(_):
    obs = stateGenerator.getStateObservation()
    socket.send_string("Hello, Server!")  # Send request

    message = socket.recv_string()  # Receive the reply
    print(f"Received reply: {message}")

    # setpoint_publisher.publish(setpoints)


if not torch.cuda.is_available():
    print("WARN: Running policy on CPU!")

if __name__ == "__main__":
    rospy.init_node("policy_controller")
    setpoint_publisher = rospy.Publisher("/servos/command", ServoCommand, queue_size=1)
    stateGenerator = StateGenerator()

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://humanoid-mpc:5555")

    control_interval = float(rospy.get_param("~control_interval"))
    if control_interval < 0:
        while not rospy.is_shutdown():
            generateControl(None)
    else:
        timer = rospy.Timer(rospy.Duration(), generateControl)

        rospy.spin()
