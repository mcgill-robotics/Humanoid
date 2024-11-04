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

    lin_accel_imu = np.array(
        [
            msg.linear_acceleration.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
        ]
    )
    local_lin_accel = quaternion.rotate_vectors(q_NWU_imuFrame.conj(), lin_accel_imu)
    ros_local_lin = Vector3(
        x=local_lin_accel[0], y=local_lin_accel[1], z=local_lin_accel[2]
    )

    ang_vel_imu = np.array(
        [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
    )
    local_ang_vel = quaternion.rotate_vectors(q_NWU_imuFrame.conj(), ang_vel_imu)
    ros_ang_vel = Vector3(x=local_ang_vel[0], y=local_ang_vel[1], z=local_ang_vel[2])

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
