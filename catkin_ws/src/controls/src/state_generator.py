import rospy
from humanoid_msgs.msg import ServoFeedback, PressureSensors
from geometry_msgs.msg import Vector3, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation

inverseRotateVectors = lambda q, v : Rotation.from_quat([q.x, q.y, q.z, q.w]).inv().apply(v)

# STATE
    # joint positions     5 · 16          Joint positions in radians (stacked last 5 timesteps)
    # angular velocity    5 · 3           Angular velocity (roll, pitch, yaw) from IMU (stacked)
    # agent velocity      5 · 2           X and Y velocity of robot torso (stacked)
    # linear acceleration 5 · 3           Linear acceleration from IMU (stacked)
    # gravity             5 · 3           Gravity direction, derived from angular velocity using Madgwick filter (stacked)
    # foot pressure       5 · 8           Pressure values from foot sensors (stacked)
    # previous action     5 · 16          Action filter state (stacked)
    
# NOTE: to train, we stack like [joint positions, ....., previous action, joint positions, ..... etc.]

class PPOStateGenerator:
    def __init__(self):
        self.current_joint_positions = np.array([0] * 16)
        self.current_lin_accel = np.array([0,0,0])
        self.current_ang_vel = np.array([0,0,0])
        self.current_foot_pressures = np.array([0] * 8)
        self.current_gravity_dir = np.array([0,0,-9.81])
        self.current_agent_vel = np.array([0, 0])
        self.previous_action = np.array([0]*16)
        
        self.joint_state_sub = rospy.Subscriber('/servosFeedback', ServoFeedback, self.updateJointStates)
        self.lin_accel_sub = rospy.Subscriber('/state/local_lin_accel', Vector3, self.updateLinAccel)
        self.ang_vel_sub = rospy.Subscriber('/state/ang_vel', Vector3, self.updateAngVel)
        self.pressures_sub = rospy.Subscriber('/sensor/pressure_sensors', PressureSensors, self.updatePressure)
        self.quat_sub = rospy.Subscriber('/state/quat', Quaternion, self.updateQuat)
        self.vel_sub = rospy.Subscriber('/state/global_vel', Vector3, self.updateVelocity)
        
    def updateJointStates(self, msg):
        try:
            self.current_joint_positions[model_output_mapping.index("left_ankle_pitch")] = msg.left_leg_ankle_fb[0]
            self.current_joint_positions[model_output_mapping.index("left_knee")] = msg.left_leg_knee_fb[0]
            self.current_joint_positions[model_output_mapping.index("left_hip_roll")] = msg.left_leg_hip_roll_fb[0]
            self.current_joint_positions[model_output_mapping.index("left_hip_pitch")] = msg.left_leg_hip_pitch_fb[0]
            self.current_joint_positions[model_output_mapping.index("left_hip_yaw")] = msg.left_leg_hip_yaw_fb[0]
            self.current_joint_positions[model_output_mapping.index("right_ankle_pitch")] = msg.right_leg_ankle_fb[0]
            self.current_joint_positions[model_output_mapping.index("right_knee")] = msg.right_leg_knee_fb[0]
            self.current_joint_positions[model_output_mapping.index("right_hip_roll")] = msg.right_leg_hip_roll_fb[0]
            self.current_joint_positions[model_output_mapping.index("right_hip_pitch")] = msg.right_leg_hip_pitch_fb[0]
            self.current_joint_positions[model_output_mapping.index("right_hip_yaw")] = msg.right_leg_hip_yaw_fb[0]
            self.current_joint_positions[model_output_mapping.index("torso_roll")] = msg.torso_roll_fb[0]
            self.current_joint_positions[model_output_mapping.index("torso_yaw")] = msg.torso_yaw_fb[0]
            self.current_joint_positions[model_output_mapping.index("left_elbow")] = msg.left_arm_elbow_fb[0]
            self.current_joint_positions[model_output_mapping.index("right_elbow")] = msg.right_arm_elbow_fb[0]
            self.current_joint_positions[model_output_mapping.index("left_shoulder_pitch")] = msg.left_arm_shoulder_fb[0]
            self.current_joint_positions[model_output_mapping.index("right_shoulder_pitch")] = msg.right_arm_shoulder_fb[0]
        except IndexError:
            print("WARN: Joint feedback values are incomplete (some servos not sending feedback)!")
        
    def updateLinAccel(self, msg):
        self.current_lin_accel = np.array([msg.x, msg.y, msg.z])
        
    def updateAngVel(self, msg):
        self.current_ang_vel = np.array([msg.x, msg.y, msg.z])
        
    def updatePressure(self, msg):
        self.current_foot_pressures[model_pressure_sensor_mapping.index("pressure_geom_RLF")] = msg.right_left_forward
        self.current_foot_pressures[model_pressure_sensor_mapping.index("pressure_geom_RLB")] = msg.right_left_back
        self.current_foot_pressures[model_pressure_sensor_mapping.index("pressure_geom_RRF")] = msg.right_right_forward
        self.current_foot_pressures[model_pressure_sensor_mapping.index("pressure_geom_RRB")] = msg.right_right_back
        self.current_foot_pressures[model_pressure_sensor_mapping.index("pressure_geom_LLF")] = msg.left_left_forward
        self.current_foot_pressures[model_pressure_sensor_mapping.index("pressure_geom_LLB")] = msg.left_left_back
        self.current_foot_pressures[model_pressure_sensor_mapping.index("pressure_geom_LRF")] = msg.left_right_forward
        self.current_foot_pressures[model_pressure_sensor_mapping.index("pressure_geom_LRB")] = msg.left_right_back
        
    def updateQuat(self, msg):
        self.current_gravity_dir = inverseRotateVectors(msg, np.array([0,0,-9.81]))
        
    def updateVelocity(self, msg):
        self.current_agent_vel = np.array([msg.x, msg.y])
        
    def updateAction(self, action_taken):
        self.previous_action = action_taken
    
    def getStateObservation(self):
        return np.concatenate((self.current_joint_positions,
                                        self.current_ang_vel,
                                        self.current_agent_vel,
                                        self.current_lin_accel,
                                        self.current_gravity_dir,
                                        self.current_foot_pressures,
                                        self.previous_action))
            

model_output_mapping = ["right_shoulder_pitch", "right_elbow", "left_shoulder_pitch", "left_elbow", "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_pitch", "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle_pitch", "torso_yaw", "torso_roll"]

model_pressure_sensor_mapping = ["pressure_geom_LLB", "pressure_geom_LRB", "pressure_geom_LRF", "pressure_geom_LLF", "pressure_geom_RLB", "pressure_geom_RRB", "pressure_geom_RRF", "pressure_geom_RLF"]

joint_limits = {
    "right_shoulder_pitch": [-90, 90],
    "right_elbow": [-90, 90],
    "left_shoulder_pitch": [-90, 90],
    "left_elbow": [-90, 90],
    "left_hip_yaw": [-90, 90],
    "left_hip_roll": [-90, 90],
    "left_hip_pitch": [-90, 90],
    "left_knee": [-90, 90],
    "left_ankle_pitch": [-90, 90],
    "right_hip_yaw": [-90, 90],
    "right_hip_roll": [-90, 90],
    "right_hip_pitch": [-90, 90],
    "right_knee": [-90, 90],
    "right_ankle_pitch": [-90, 90],
    "torso_yaw": [-90, 90],
    "torso_roll": [-90, 90]
}