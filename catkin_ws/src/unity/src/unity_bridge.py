#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Quaternion, Vector3
from humanoid_msgs.msg import UnityState
import quaternion


def unityStateCb(msg):
    unityQuat = msg.quat
    quat = (
        q_NWU_NED
        * np.quaternion(unityQuat.w, unityQuat.x, unityQuat.y, unityQuat.z)
        * q_NWU_NED.conj()
    )
    ros_quat = Quaternion(x=quat.x, y=quat.y, z=quat.z, w=quat.w)

    unityGlobalVel = msg.global_vel
    global_vel = Vector3(x=unityGlobalVel.z, y=-unityGlobalVel.x, z=0)

    unityLinAccel = msg.local_lin_accel
    global_lin_accel = np.array([-unityLinAccel.z, unityLinAccel.x, -unityLinAccel.y])
    local_lin_accel = quaternion.rotate_vectors(quat.conj(), global_lin_accel)
    ros_local_lin = Vector3(
        x=local_lin_accel[0], y=local_lin_accel[1], z=local_lin_accel[2]
    )

    unityAngVel = msg.ang_vel
    global_ang_vel = (
        np.pi * np.array([-unityAngVel.z, unityAngVel.x, -unityAngVel.y]) / 180
    )
    local_ang_vel = quaternion.rotate_vectors(quat.conj(), global_ang_vel)
    ros_ang_vel = Vector3(x=local_ang_vel[0], y=local_ang_vel[1], z=local_ang_vel[2])

    pub_global_vel.publish(global_vel)
    pub_quat.publish(ros_quat)
    pub_ang_vel.publish(ros_ang_vel)
    pub_local_lin.publish(ros_local_lin)


if __name__ == "__main__":
    rospy.init_node("unity_bridge")

    # publishers
    pub_local_lin = rospy.Publisher("/state/local_lin_accel", Vector3, queue_size=1)
    pub_quat = rospy.Publisher("/state/quat", Quaternion, queue_size=1)
    pub_ang_vel = rospy.Publisher("/state/ang_vel", Vector3, queue_size=1)
    pub_global_vel = rospy.Publisher("/state/global_vel", Vector3, queue_size=1)

    # REFERENCE FRAME DEFINITIONS
    q_NWU_NED = np.quaternion(0, 1, 0, 0)

    # subscribers
    rospy.Subscriber("/unity/state", UnityState, unityStateCb)

    rospy.spin()
