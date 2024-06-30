import rospy
from humanoid_msgs.msg import ServoFeedback
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Bool
import numpy as np
from scipy.spatial.transform import Rotation

inverseRotateVectors = (
    lambda q, v: Rotation.from_quat([q.x, q.y, q.z, q.w]).inv().apply(v)
)

# STATE
# joint positions                  12          Joint positions in radians (stacked last 5 timesteps)
# angular velocity                 3           Angular velocity (roll, pitch, yaw) from IMU (stacked)
# gravity                          3           Gravity direction, derived from angular velocity using Madgwick filter (stacked)
# binary left foot contact state   1           1 if left foot is in contact with the ground, 0 otherwise
# binary right foot contact state  1           1 if left foot is in contact with the ground, 0 otherwise

# NOTE: to train, we stack like [joint positions, ....., previous action, joint positions, ..... etc.]


class StateGenerator:
    def __init__(self):

        self.current_joint_positions = np.array([0] * len(JOINTS.keys()))
        self.current_ang_vel = np.array([0, 0, 0])
        self.current_gravity_dir = np.array([0, 0, -1])
        self.left_foot_contact = np.array([1])
        self.right_foot_contact = np.array([1])

        self.joint_state_sub = rospy.Subscriber(
            "/servosFeedback", ServoFeedback, self.updateJointStates
        )
        self.ang_vel_sub = rospy.Subscriber(
            "/state/ang_vel", Vector3, self.updateAngVel
        )
        self.left_con_sub = rospy.Subscriber(
            "/sensor/contact/left", Bool, self.updateLeftContact
        )
        self.right_con_sub = rospy.Subscriber(
            "/sensor/contact/right", Bool, self.updateRightContact
        )
        self.quat_sub = rospy.Subscriber("/state/quat", Quaternion, self.updateQuat)

    def updateJointStates(self, msg):
        joint_positions = []
        for joint_name in JOINTS.keys():
            try:
                joint_pos = msg.get_attr(joint_name + "_fb")
                joint_positions.append(joint_pos)
            except:
                print(
                    "WARN: Joint feedback values incomplete. Missing {}!".format(
                        joint_name
                    )
                )
                joint_positions.append(0)
        self.current_joint_positions = np.array(joint_positions)

    def updateAngVel(self, msg):
        self.current_ang_vel = np.array([msg.x, msg.y, msg.z])

    def updateLeftContact(self, msg):
        self.left_foot_contact = np.array([1 if msg.data else 0])

    def updateRightContact(self, msg):
        self.right_foot_contact = np.array([1 if msg.data else 0])

    def updateQuat(self, msg):
        self.current_gravity_dir = inverseRotateVectors(msg, np.array([0, 0, -1]))

    def getStateObservation(self):
        return np.concatenate(
            (
                self.current_joint_positions,
                self.current_ang_vel,
                self.current_gravity_dir,
                self.left_foot_contact,
                self.right_foot_contact,
            )
        )


JOINTS = {
    "right_shoulder_pitch": [-90, 90],
    "right_shoulder_roll": [0, 90],
    "right_elbow": [-90, 0],
    "left_shoulder_pitch": [-90, 90],
    "left_shoulder_roll": [-90, 0],
    "left_elbow": [-90, 0],
    "left_hip_roll": [-90, 90],
    "left_hip_pitch": [-90, 90],
    "left_knee": [0, 90],
    "right_hip_roll": [-90, 90],
    "right_hip_pitch": [-90, 90],
    "right_knee": [0, 90],
}
