import rospy
import numpy as np
from humanoid_msgs.msg import HardwareServoCommand, ServoFeedback, HardwareServoFeedback, ServoCommand

JOINTS_MAPPINGS_SOFTWARE_HARDWARE = {
    'right_shoulder_pitch': 'right_shoulder_pitch',
    'right_shoulder_roll': 'right_shoulder_roll',
    'right_elbow': 'right_elbow',
    'left_shoulder_pitch': 'left_shoulder_pitch',
    'left_shoulder_roll': 'left_shoulder_roll',
    'left_elbow': 'left_elbow',
    'left_hip_roll': 'left_hip_roll',
    'left_hip_pitch': 'left_hip_pitch',
    'left_knee': 'left_knee',
    'right_hip_roll': 'right_hip_roll',
    'right_hip_pitch': 'right_hip_pitch',
    'right_knee': 'right_knee',
}

def radians_to_degrees(radians_value):
    return radians_value * (180.0 / np.pi)

def degrees_to_radians(degrees_value):
    return degrees_value / (180.0 / np.pi)

def clamp(x, lower_limit, upper_limit):
    return max(lower_limit, min(x, upper_limit))

def simServoCommandCb(msg):
    setpoints = HardwareServoCommand()

    for software_joint, hardware_joint in JOINTS_MAPPINGS_SOFTWARE_HARDWARE.items():
        joint_specs = rospy.getparam(software_joint)
        direction = joint_specs[2]
        
        if direction > 0:
            upper_joint_limit = joint_specs[0]
            lower_joint_limit = joint_specs[1]
        if direction < 0:
            upper_joint_limit = -joint_specs[1]
            lower_joint_limit = -joint_specs[0]
        
        joint_setpoint = 180 + clamp(direction * radians_to_degrees(getattr(msg, software_joint)), lower_joint_limit, upper_joint_limit)
        setattr(setpoints, hardware_joint, joint_setpoint)

    setpoint_publisher.publish(setpoints)
    
def simServoFeedbackCb(msg):
    feedback = ServoFeedback()
    
    for software_joint, hardware_joint in JOINTS_MAPPINGS_SOFTWARE_HARDWARE.items():
        joint_specs = rospy.getparam(software_joint)
        direction = joint_specs[2]
        
        joint_feedback = direction * radians_to_degrees(getattr(msg, hardware_joint)-180)
        setattr(feedback, software_joint, joint_feedback)

    setpoint_publisher.publish(feedback)


if __name__ == "__main__":
    rospy.init_node("hardware_interface")
    
    # publishers
    setpoint_publisher = rospy.Publisher('/hardware/servos/command', HardwareServoCommand, queue_size=1)
    feedback_publisher = rospy.Publisher('/servos/feedback', ServoFeedback, queue_size=1)
    # subscribers
    rospy.Subscriber("/servos/command", ServoCommand, simServoCommandCb)
    rospy.Subscriber("/hardware/servos/feedback", HardwareServoFeedback, simServoFeedbackCb)

    rospy.spin()