#!/usr/bin/env python

import rospy
import numpy as np
from humanoid_msgs.msg import (
    HardwareServoCommand,
    ServoFeedback,
    HardwareServoFeedback,
    ServoCommand,
)
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float32

JOINTS_MAPPINGS_SOFTWARE_HARDWARE = {
    "right_shoulder_pitch": "right_shoulder_pitch",
    "right_shoulder_roll": "right_shoulder_roll",
    "right_elbow": "right_elbow",
    "left_shoulder_pitch": "left_shoulder_pitch",
    "left_shoulder_roll": "left_shoulder_roll",
    "left_elbow": "left_elbow",
    "left_hip_roll": "left_hip_roll",
    "left_hip_pitch": "left_hip_pitch",
    "left_knee": "left_knee",
    "right_hip_roll": "right_hip_roll",
    "right_hip_pitch": "right_hip_pitch",
    "right_knee": "right_knee",
}


def modulo_angle(angle):
    return (angle + 180) % 360 - 180


def radians_to_degrees(radians_value):
    return modulo_angle(radians_value * (180.0 / np.pi))


def degrees_to_radians(degrees_value):
    return modulo_angle(degrees_value) / (180.0 / np.pi)


def clamp(x, lower_limit, upper_limit):
    return max(lower_limit, min(x, upper_limit))


allow_commands = False


def _position_command_cb(msg):
    if not allow_commands:
        return
    setpoints = HardwareServoCommand()

    for software_joint, hardware_joint in JOINTS_MAPPINGS_SOFTWARE_HARDWARE.items():
        joint_specs = rospy.get_param("~" + software_joint)
        direction = joint_specs[2]
        joint_center = rospy.get_param("~" + software_joint + "_center")

        if direction > 0:
            upper_joint_limit = joint_specs[0]
            lower_joint_limit = joint_specs[1]
        if direction < 0:
            upper_joint_limit = -joint_specs[1]
            lower_joint_limit = -joint_specs[0]

        joint_setpoint = joint_center + clamp(
            direction * radians_to_degrees(getattr(msg, software_joint)),
            lower_joint_limit,
            upper_joint_limit,
        )
        setattr(setpoints, hardware_joint, joint_setpoint)

    setpoint_publisher.publish(setpoints)


def _torque_command_cb(msg):
    if not allow_commands:
        return
    setpoints = HardwareServoCommand()

    for software_joint, hardware_joint in JOINTS_MAPPINGS_SOFTWARE_HARDWARE.items():
        joint_specs = rospy.get_param("~" + software_joint)
        direction = joint_specs[2]

        torque_setpoint = 885 * (direction * getattr(msg, software_joint) / 1.4)
        setattr(setpoints, hardware_joint, torque_setpoint)

    setpoint_publisher.publish(setpoints)


def feedbackCb(msg):
    feedback = ServoFeedback()

    for software_joint, hardware_joint in JOINTS_MAPPINGS_SOFTWARE_HARDWARE.items():
        joint_specs = rospy.get_param("~" + software_joint)
        direction = joint_specs[2]

        joint_center = rospy.get_param("~" + software_joint + "_center")

        hw_feedback_tuple = getattr(msg, hardware_joint)
        sw_feedback_tuple = []

        if len(hw_feedback_tuple) > 0:
            joint_pos = direction * degrees_to_radians(
                hw_feedback_tuple[0] - joint_center
            )
            sw_feedback_tuple.append(joint_pos)

        if len(hw_feedback_tuple) > 1:
            joint_vel = direction * degrees_to_radians(hw_feedback_tuple[1])
            sw_feedback_tuple.append(joint_vel)

        if len(hw_feedback_tuple) > 2:
            joint_load = direction * hw_feedback_tuple[2]
            sw_feedback_tuple.append(joint_load)

        setattr(feedback, software_joint, sw_feedback_tuple)

    feedback_publisher.publish(feedback)


commandCb = _position_command_cb

def commandCb_caller(msg):
    global commandCb
    commandCb(msg)

def control_mode_cb(msg):
    global commandCb
    global allow_commands

    allow_commands = False

    if msg.data == 0:
        control_mode_change_pub.publish(msg)
        commandCb = _position_command_cb
    elif msg.data == 1:
        control_mode_change_pub.publish(msg)
        commandCb = _torque_command_cb
    else:
        rospy.logwarn(
            "Invalid control mode: %d. Continuing with previous control mode.", msg.data
        )
    rospy.sleep(1)
    allow_commands = True


if __name__ == "__main__":
    rospy.init_node("hardware_interface")

    rospy.Subscriber("/servos/set_control_mode", Float32, control_mode_cb)
    control_mode_change_pub = rospy.Publisher(
        "hardware/servos/set_control_mode", Float32, queue_size=1
    )

    rospy.Subscriber("/servos/command", ServoCommand, commandCb_caller)
    setpoint_publisher = rospy.Publisher(
        "/hardware/servos/command", HardwareServoCommand, queue_size=1
    )

    rospy.Subscriber("/hardware/servos/feedback", HardwareServoFeedback, feedbackCb)
    feedback_publisher = rospy.Publisher(
        "/servos/feedback", ServoFeedback, queue_size=1
    )

    rospy.spin()
