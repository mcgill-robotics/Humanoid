#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import numpy as np
from geometry_msgs.msg import Quaternion, Vector3
import quaternion

def imuDataCb(msg):
    imuQuat = msg.orientation
    quat = (
        q_NWU_imuFrame
        * np.quaternion(imuQuat.w, imuQuat.x, imuQuat.y, imuQuat.z)
        * q_NWU_imuFrame.conj()
    )
    ros_quat = Quaternion(x=quat.x, y=quat.y, z=quat.z, w=quat.w)

    # unityGlobalVel = msg.global_vel
    # global_vel = Vector3(x=unityGlobalVel.z, y=-unityGlobalVel.x, z=0)

    imuLinAccel = msg.linear_acceleration
    global_lin_accel = np.array([-imuLinAccel.z, imuLinAccel.x, -imuLinAccel.y])
    local_lin_accel = quaternion.rotate_vectors(quat.conj(), global_lin_accel)
    ros_local_lin = Vector3(
        x=local_lin_accel[0], y=local_lin_accel[1], z=local_lin_accel[2]
    )

    imuAngVel = msg.angular_velocity
    global_ang_vel = (np.array([imuAngVel.x, imuAngVel.y, imuAngVel.z]))

    local_ang_vel = quaternion.rotate_vectors(quat.conj(), global_ang_vel)
    ros_ang_vel = Vector3(x=local_ang_vel[0], y=local_ang_vel[1], z=local_ang_vel[2])

    # pub_global_vel.publish(global_vel)
    pub_quat.publish(ros_quat)
    pub_ang_vel.publish(ros_ang_vel)
    pub_local_lin.publish(ros_local_lin)


if __name__ == "__main__":
    rospy.init_node("imu_data")

    # publishers
    pub_local_lin = rospy.Publisher("/state/local_lin_accel", Vector3, queue_size=1)
    pub_quat = rospy.Publisher("/state/quat", Quaternion, queue_size=1)
    pub_ang_vel = rospy.Publisher("/state/ang_vel", Vector3, queue_size=1)
    # pub_global_vel = rospy.Publisher("/state/global_vel", Vector3, queue_size=1)

    # REFERENCE FRAME DEFINITIONS
    q_NWU_imuFrame = np.quaternion(0.707, 0, 0.707, 0)

    # subscribers
    rospy.Subscriber("/imu/data", Imu, imuDataCb)

    rospy.spin()
