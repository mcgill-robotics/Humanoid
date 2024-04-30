#!/usr/bin/env python

import rospy
from humanoid_msgs.msg import ServoCommand
import numpy as np

def generate_sine_wave(frequency, amplitude, phase):
    return amplitude * np.sin(2*3.1415*frequency*rospy.get_time() + (3.1415*phase)/180)

def publish_joint_angles(): 
    setpoint_publisher = rospy.Publisher('/servosCommand', ServoCommand, queue_size=1)
    rate = rospy.Rate(100)
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
    
    while not rospy.is_shutdown():
    
        setpoints.left_leg_ankle_setpoint = 150 + generate_sine_wave(rospy.get_param("~ankle_frequency"), rospy.get_param("~ankle_ampl"), rospy.get_param("~left_ankle_phase"))
        setpoints.left_leg_knee_setpoint = 100 + generate_sine_wave(rospy.get_param("~knee_frequency"), rospy.get_param("~knee_ampl"), rospy.get_param("~left_knee_phase"))
        setpoints.left_leg_hip_pitch_setpoint = 150 + generate_sine_wave(rospy.get_param("~hip_frequency"), rospy.get_param("~hip_ampl"), rospy.get_param("~left_hip_phase"))
        setpoints.right_leg_ankle_setpoint = 150 + generate_sine_wave(rospy.get_param("~ankle_frequency"), rospy.get_param("~ankle_ampl"), rospy.get_param("~right_ankle_phase"))
        setpoints.right_leg_knee_setpoint = 100 + generate_sine_wave(rospy.get_param("~knee_frequency"), rospy.get_param("~knee_ampl"), rospy.get_param("~right_knee_phase"))
        setpoints.right_leg_hip_pitch_setpoint = 150 + generate_sine_wave(rospy.get_param("~hip_frequency"), rospy.get_param("~hip_ampl"), rospy.get_param("~right_hip_phase"))

        setpoint_publisher.publish(setpoints)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('sine_control')
    try:
        publish_joint_angles()
    except rospy.ROSInterruptException:
        pass