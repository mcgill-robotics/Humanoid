#!/usr/bin/env python

import rospy
from humanoid_msgs.msg import ServoCommand
import numpy as np

def generate_sine_wave(time):
    amplitude = 40

    sine_wave = amplitude * np.sin(time)

    return sine_wave

def publish_joint_angles():
    setpoint_publisher = rospy.Publisher('/servosCommand', ServoCommand, queue_size=1)

    ros_update_interval = 1
    rate = rospy.Rate(1 / ros_update_interval)
    x = 0
    while not rospy.is_shutdown():
        
        sine_wave_update = 0.1 # 10 Hz

        setpoints = ServoCommand()
        
        time_offsets = [0, 0, 1.0, 1.0, 2.0, 2.0] 
        
        for i, offset in enumerate(time_offsets):
            sine_value = generate_sine_wave(x + offset)
            
            if i == 0:
                setpoints.left_leg_hip_pitch_setpoint = sine_value 
            elif i == 1:
                setpoints.right_leg_hip_pitch_setpoint = sine_value
            elif i == 2:
                setpoints.left_leg_knee_setpoint = sine_value 
            elif i == 3:
                setpoints.right_leg_knee_setpoint = sine_value 
            elif i == 4:
                setpoints.left_leg_ankle_setpoint = sine_value 
            elif i== 5:
                setpoints.right_leg_ankle_setpoint = sine_value

        setpoint_publisher.publish(setpoints)
        x+= sine_wave_update
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('sine_control')
    try:
        publish_joint_angles()
    except rospy.ROSInterruptException:
        pass