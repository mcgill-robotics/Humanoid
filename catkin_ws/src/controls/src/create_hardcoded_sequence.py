#!/usr/bin/env python

import rospy
from humanoid_msgs.msg import ServoCommand, ServoFeedback
import numpy as np
import keyboard


def update_setpoints(feedback_msg):
    global setpoints
    setpoints = ServoCommand()

    setpoints.left_leg_ankle_setpoint = feedback_msg.left_leg_ankle_fb
    setpoints.left_leg_knee_setpoint = feedback_msg.left_leg_knee_fb
    setpoints.left_leg_hip_roll_setpoint = feedback_msg.left_leg_hip_roll_fb
    setpoints.left_leg_hip_pitch_setpoint = feedback_msg.left_leg_hip_pitch_fb
    setpoints.left_leg_hip_yaw_setpoint = feedback_msg.left_leg_hip_yaw_fb
    setpoints.right_leg_ankle_setpoint = feedback_msg.right_leg_ankle_fb
    setpoints.right_leg_knee_setpoint = feedback_msg.right_leg_knee_fb
    setpoints.right_leg_hip_roll_setpoint = feedback_msg.right_leg_hip_roll_fb
    setpoints.right_leg_hip_pitch_setpoint = feedback_msg.right_leg_hip_pitch_fb
    setpoints.right_leg_hip_yaw_setpoint = feedback_msg.right_leg_hip_yaw_fb
    setpoints.torso_roll_setpoint = feedback_msg.torso_roll_fb
    setpoints.torso_yaw_setpoint = feedback_msg.torso_yaw_fb
    setpoints.left_arm_elbow_setpoint = feedback_msg.left_arm_elbow_fb
    setpoints.right_arm_elbow_setpoint = feedback_msg.right_arm_elbow_fb
    setpoints.left_arm_shoulder_setpoint = feedback_msg.left_arm_shoulder_fb
    setpoints.right_arm_shoulder_setpoint = feedback_msg.right_arm_shoulder_fb

    position_pub.publish(setpoints)


def save_keyframe():
    global setpoints
    if setpoints is not None:
        with open(rospy.get_param("hardcoded_sequence_location"), "a+") as f:
            f.write(
                f"{setpoints.left_leg_ankle_setpoint}, \
                {setpoints.left_leg_knee_setpoint}, \
                {setpoints.left_leg_hip_roll_setpoint}, \
                {setpoints.left_leg_hip_pitch_setpoint}, \
                {setpoints.left_leg_hip_yaw_setpoint}, \
                {setpoints.right_leg_ankle_setpoint}, \
                {setpoints.right_leg_knee_setpoint}, \
                {setpoints.right_leg_hip_roll_setpoint}, \
                {setpoints.right_leg_hip_pitch_setpoint}, \
                {setpoints.right_leg_hip_yaw_setpoint}, \
                {setpoints.torso_roll_setpoint}, \
                {setpoints.torso_yaw_setpoint}, \
                {setpoints.left_arm_elbow_setpoint}, \
                {setpoints.right_arm_elbow_setpoint}, \
                {setpoints.left_arm_shoulder_setpoint}, \
                {setpoints.right_arm_shoulder_setpoint}\n"
            )
        print("Keyframe saved.")


if __name__ == "__main__":
    rospy.init_node("create_hardcoded_sequence")
    setpoints = None
    position_pub = rospy.Publisher("/servosCommand", ServoCommand, queue_size=1)
    position_sub = rospy.Subscriber("/servosFeedback", ServoFeedback, update_setpoints)

    keyboard.on_press_key("enter", save_keyframe)

    rospy.spin()
