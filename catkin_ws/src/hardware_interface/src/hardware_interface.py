import rospy
from humanoid_msgs.msg import ServoCommand, SimServoCommand

# TODO change servo control publisher in unity
# TODO create mujoco connection to ros

def simServoCommandCb(msg):
    setpoints = ServoCommand()
    setpoints.left_leg_ankle_setpoint = msg.left_leg_ankle_setpoint
    setpoints.left_leg_knee_setpoint = msg.left_leg_knee_setpoint
    setpoints.left_leg_hip_roll_setpoint = msg.left_leg_hip_roll_setpoint
    setpoints.left_leg_hip_pitch_setpoint = msg.left_leg_hip_pitch_setpoint
    setpoints.left_leg_hip_yaw_setpoint = msg.left_leg_hip_yaw_setpoint
    setpoints.right_leg_ankle_setpoint = msg.right_leg_ankle_setpoint
    setpoints.right_leg_knee_setpoint = msg.right_leg_knee_setpoint
    setpoints.right_leg_hip_roll_setpoint = msg.right_leg_hip_roll_setpoint
    setpoints.right_leg_hip_pitch_setpoint = msg.right_leg_hip_pitch_setpoint
    setpoints.right_leg_hip_yaw_setpoint = msg.right_leg_hip_yaw_setpoint
    setpoints.torso_roll_setpoint = msg.torso_roll_setpoint
    setpoints.torso_yaw_setpoint = msg.torso_yaw_setpoint
    setpoints.left_arm_elbow_setpoint = msg.left_arm_elbow_setpoint
    setpoints.right_arm_elbow_setpoint = msg.right_arm_elbow_setpoint
    setpoints.left_arm_shoulder_setpoint = msg.left_arm_shoulder_setpoint
    setpoints.right_arm_shoulder_setpoint = msg.right_arm_shoulder_setpoint
    setpoint_publisher.publish(setpoints)
    
    

if __name__ == "__main__":
    rospy.init_node("hardware_interface")
    
    # publishers
    setpoint_publisher = rospy.Publisher('/hardware/servos/command', HardwareServoCommand, queue_size=1)
    feedback_publisher = rospy.Publisher('/servos/feedback', ServoFeedback, queue_size=1)
    # subscribers
    rospy.Subscriber("/servos/command", ServoCommand, simServoCommandCb)
    rospy.Subscriber("/hardware/servos/feedback", HardwareServoFeedback, simServoCommandCb)

    rospy.spin()