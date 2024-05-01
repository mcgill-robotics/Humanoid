#!/usr/bin/env python

import rospy
from humanoid_msgs.msg import ServoCommand
import numpy as np


def play_back_keyframes():
    raw_keyframes = []
    interpolated_keyframes = []

    TIME_PER_KEYFRAME = 1
    X = 100  # number of interpolated keyframes per raw keyframe

    # load in raw keyframes
    with open(rospy.get_param("hardcoded_sequence_location"), "r") as f:
        rows = f.readlines()
        for row in rows:
            row = [float(x) for x in row.split(",")]
            raw_keyframes.append(row)

    raw_keyframes.append(raw_keyframes[0])

    interpolated_keyframes.append(raw_keyframes[0])
    # generate interpolated keyframes between the raw ones
    for i in range(len(raw_keyframes) - 1):
        interpolated_keyframes.extend(
            np.linspace(raw_keyframes[i], raw_keyframes[i + 1], X).tolist()
        )

    # Add the last raw keyframe to the interpolated keyframes
    interpolated_keyframes.append(raw_keyframes[-1])

    position_pub = rospy.Publisher("/servosCommand", ServoCommand, queue_size=1)
    setpoints = ServoCommand()
    time_per_interpolated_keyframe = TIME_PER_KEYFRAME / X
    for keyframe in interpolated_keyframes:
        setpoints.left_leg_ankle_setpoint = float(keyframe[0])
        setpoints.left_leg_knee_setpoint = float(keyframe[1])
        setpoints.left_leg_hip_roll_setpoint = float(keyframe[2])
        setpoints.left_leg_hip_pitch_setpoint = float(keyframe[3])
        setpoints.left_leg_hip_yaw_setpoint = float(keyframe[4])
        setpoints.right_leg_ankle_setpoint = float(keyframe[5])
        setpoints.right_leg_knee_setpoint = float(keyframe[6])
        setpoints.right_leg_hip_roll_setpoint = float(keyframe[7])
        setpoints.right_leg_hip_pitch_setpoint = float(keyframe[8])
        setpoints.right_leg_hip_yaw_setpoint = float(keyframe[9])
        setpoints.torso_roll_setpoint = float(keyframe[10])
        setpoints.torso_yaw_setpoint = float(keyframe[11])
        setpoints.left_arm_elbow_setpoint = float(keyframe[12])
        setpoints.right_arm_elbow_setpoint = float(keyframe[13])
        setpoints.left_arm_shoulder_setpoint = float(keyframe[14])
        setpoints.right_arm_shoulder_setpoint = float(keyframe[15])

        position_pub.publish(setpoints)

        rospy.sleep(time_per_interpolated_keyframe)


if __name__ == "__main__":
    rospy.init_node("play_hardcoded_sequence")
    while not rospy.is_shutdown():
        print("Playing back sequence...")
        play_back_keyframes()
