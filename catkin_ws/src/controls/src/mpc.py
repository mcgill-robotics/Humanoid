#!/usr/bin/env python3

import rospy
import zmq
from humanoid_msgs.msg import ServoCommand
import numpy as np
import torch
from stable_baselines3 import SAC
from state_generator import StateGenerator, JOINTS
import json


def setupConnection():
    global context, socket
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")
    socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout


def generateControl(_):
    joint_pos, joint_vel, ang_vel, quat = stateGenerator.getMPCObservation()
    state_json = {
        "joint_pos": [float(x) for x in joint_pos],
        "joint_vel": [float(x) for x in joint_vel],
        "ang_vel": [float(x) for x in ang_vel],
        "quat": [float(x) for x in quat],
    }
    str_state_json = json.dumps(state_json)

    try:
        socket.send_string(str_state_json)
        message = socket.recv_string()
    except zmq.Again as e:
        print("Timeout while waiting for MPC server.")
        setupConnection()
        message = json.dumps([0.0] * len(JOINTS))

    command_arr = json.loads(message)
    command = ServoCommand()
    for i in range(len(JOINTS)):
        try:
            setattr(command, JOINTS[i], command_arr[i])
        except Exception as e:
            print(
                "Failed to set command for joint: {}. Error: {}".format(
                    JOINTS[i], str(e)
                )
            )

    command_pub.publish(command)


if __name__ == "__main__":
    rospy.init_node("mpc")
    command_pub = rospy.Publisher("/servos/command", ServoCommand, queue_size=1)
    stateGenerator = StateGenerator()
    setupConnection()

    control_interval = float(rospy.get_param("~control_interval"))
    if control_interval < 0:
        while not rospy.is_shutdown():
            generateControl(None)
    else:
        timer = rospy.Timer(rospy.Duration(), generateControl)

        rospy.spin()
