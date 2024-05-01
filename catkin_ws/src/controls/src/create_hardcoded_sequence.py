#!/usr/bin/env python

import rospy
from humanoid_msgs.msg import ServoCommand, ServoFeedback
import numpy as np


def update_setpoints(feedback_msg):
    global setpoints
    setpoints = ServoCommand()
    setpoints.left_leg_ankle_setpoint = 150
    setpoints.left_leg_knee_setpoint = 150
    setpoints.left_leg_hip_roll_setpoint = 150
    setpoints.left_leg_hip_pitch_setpoint = 150
    setpoints.left_leg_hip_yaw_setpoint = 150
    setpoints.right_leg_ankle_setpoint = 150
    setpoints.right_leg_knee_setpoint = 150
    setpoints.right_leg_hip_roll_setpoint = 150
    setpoints.right_leg_hip_pitch_setpoint = 150
    setpoints.right_leg_hip_yaw_setpoint = 150
    setpoints.torso_roll_setpoint = 150
    setpoints.torso_yaw_setpoint = 150
    setpoints.left_arm_elbow_setpoint = 150
    setpoints.right_arm_elbow_setpoint = 150
    setpoints.left_arm_shoulder_setpoint = 150
    setpoints.right_arm_shoulder_setpoint = 150

    if len(feedback_msg.left_leg_ankle_fb) > 0: setpoints.left_leg_ankle_setpoint = feedback_msg.left_leg_ankle_fb[0]
    if len(feedback_msg.left_leg_knee_fb) > 0: setpoints.left_leg_knee_setpoint = feedback_msg.left_leg_knee_fb[0]
    if len(feedback_msg.left_leg_hip_roll_fb) > 0: setpoints.left_leg_hip_roll_setpoint = feedback_msg.left_leg_hip_roll_fb[0]
    if len(feedback_msg.left_leg_hip_pitch_fb) > 0: setpoints.left_leg_hip_pitch_setpoint = feedback_msg.left_leg_hip_pitch_fb[0]
    if len(feedback_msg.left_leg_hip_yaw_fb) > 0: setpoints.left_leg_hip_yaw_setpoint = feedback_msg.left_leg_hip_yaw_fb[0]
    if len(feedback_msg.right_leg_ankle_fb) > 0: setpoints.right_leg_ankle_setpoint = feedback_msg.right_leg_ankle_fb[0]
    if len(feedback_msg.right_leg_knee_fb) > 0: setpoints.right_leg_knee_setpoint = feedback_msg.right_leg_knee_fb[0]
    if len(feedback_msg.right_leg_hip_roll_fb) > 0: setpoints.right_leg_hip_roll_setpoint = feedback_msg.right_leg_hip_roll_fb[0]
    if len(feedback_msg.right_leg_hip_pitch_fb) > 0: setpoints.right_leg_hip_pitch_setpoint = feedback_msg.right_leg_hip_pitch_fb[0]
    if len(feedback_msg.right_leg_hip_yaw_fb) > 0: setpoints.right_leg_hip_yaw_setpoint = feedback_msg.right_leg_hip_yaw_fb[0]
    if len(feedback_msg.torso_roll_fb) > 0: setpoints.torso_roll_setpoint = feedback_msg.torso_roll_fb[0]
    if len(feedback_msg.torso_yaw_fb) > 0: setpoints.torso_yaw_setpoint = feedback_msg.torso_yaw_fb[0]
    if len(feedback_msg.left_arm_elbow_fb) > 0: setpoints.left_arm_elbow_setpoint = feedback_msg.left_arm_elbow_fb[0]
    if len(feedback_msg.right_arm_elbow_fb) > 0: setpoints.right_arm_elbow_setpoint = feedback_msg.right_arm_elbow_fb[0]
    if len(feedback_msg.left_arm_shoulder_fb) > 0: setpoints.left_arm_shoulder_setpoint = feedback_msg.left_arm_shoulder_fb[0]
    if len(feedback_msg.right_arm_shoulder_fb) > 0: setpoints.right_arm_shoulder_setpoint = feedback_msg.right_arm_shoulder_fb[0]

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

    rospy.spin()
