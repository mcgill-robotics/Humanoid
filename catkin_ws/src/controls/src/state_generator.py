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

        self.current_joint_positions = np.array([0] * len(JOINTS))
        self.current_joint_velocities = np.array([0] * len(JOINTS))
        self.current_local_ang_vel = np.array([0, 0, 0])
        self.current_local_gravity_vector = np.array([0, 0, -1])
        self.received_states = [False, False, False, False]

        self.joint_state_sub = rospy.Subscriber(
            "/servosFeedback", ServoFeedback, self.updateJointStates
        )
        self.ang_vel_sub = rospy.Subscriber(
            "/state/ang_vel", Vector3, self.updateAngVel
        )
        self.quat_sub = rospy.Subscriber("/state/quat", Quaternion, self.updateQuat)

    def updateJointStates(self, msg):
        joint_positions = []
        joint_velocities = []
        for joint_name, _ in JOINTS:
            try:
                joint_pos = getattr(msg, joint_name)[0]
                # convert to radians
                joint_pos_scaled = np.deg2rad(joint_pos - 150)
                joint_positions.append(joint_pos_scaled)

                joint_vel = getattr(msg, joint_name)[1]
                joint_vel_scaled = np.deg2rad(joint_vel)
                joint_velocities.append(joint_vel_scaled)
            except Exception as e:
                print(str(e))
                print(
                    "WARN: Joint feedback values incomplete. Missing {}!".format(
                        joint_name
                    )
                )
                joint_positions.append(0)
                joint_velocities.append(0)
        self.current_joint_positions = np.array(joint_positions)
        self.current_joint_velocities = np.array(joint_velocities)
        self.received_states[0] = True
        self.received_states[1] = True

    def updateAngVel(self, msg):
        self.current_ang_vel = np.array([msg.x, msg.y, msg.z])
        self.received_states[2] = True

    def updateQuat(self, msg):
        self.current_gravity_dir = inverseRotateVectors(msg, np.array([0, 0, -1]))
        self.received_states[3] = True

    def getStateObservation(self):
        if not all(self.received_states):
            print(
                "WARN: State space for policy incomplete. {}".format(
                    self.received_states
                )
            )

        print("Current joint positions: {}".format(self.current_joint_positions))
        print("Current joint velocities: {}".format(self.current_joint_velocities))
        print("Current angular velocity: {}".format(self.current_ang_vel))
        print("Current gravity direction: {}".format(self.current_gravity_dir))

        return np.concatenate(
            (
                self.current_joint_positions,
                self.current_joint_velocities,
                self.current_ang_vel,
                self.current_gravity_dir,
            )
        )


JOINTS = [
    ("right_shoulder_pitch", [-90, 90]),
    ("right_shoulder_roll", [-90, 0]),
    ("right_elbow", [-90, 0]),
    ("left_shoulder_pitch", [-90, 90]),
    ("left_shoulder_roll", [0, 90]),
    ("left_elbow", [-90, 0]),
    ("left_hip_roll", [-90, 90]),
    ("left_hip_pitch", [-90, 90]),
    ("left_knee", [0, 90]),
    ("right_hip_roll", [-90, 90]),
    ("right_hip_pitch", [-90, 90]),
    ("right_knee", [0, 90]),
]
