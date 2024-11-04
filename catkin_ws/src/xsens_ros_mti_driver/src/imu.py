#!/usr/bin/env python

import rospy
# from sensor_msgs.msg import Imu
import numpy as np
from geometry_msgs.msg import TwistStamped, QuaternionStamped, Vector3Stamped, Vector3, Quaternion
import quaternion


def quatDataCb(msg):
    imuQuat = msg.quaternion
    quat = (
        q_NWU_imuFrame
        * np.quaternion(imuQuat.w, imuQuat.x, imuQuat.y, imuQuat.z)
        * q_NWU_imuFrame.conj()
    )
    ros_quat = Quaternion(x=quat.x, y=quat.y, z=quat.z, w=quat.w)

    pub_quat.publish(ros_quat)

def twistDataCb(msg):

    imuTwist = msg.twist

    ang_vel_imu = np.array(
    [imuTwist.x, imuTwist.y, imuTwist.z]
    )
    local_ang_vel = quaternion.rotate_vectors(q_NWU_imuFrame.conj(), ang_vel_imu)
    ros_ang_vel = Vector3(x=local_ang_vel[1], y=local_ang_vel[2], z=local_ang_vel[0])

    pub_ang_vel.publish(ros_ang_vel)

def accelDataCb(msg):

    imuLinAccel = msg.vector

    lin_accel_imu = np.array(
        [
            imuLinAccel.x,
            imuLinAccel.y,
            imuLinAccel.z,
        ]
    )
    local_lin_accel = quaternion.rotate_vectors(q_NWU_imuFrame.conj(), lin_accel_imu)
    ros_local_lin = Vector3(
        x=local_lin_accel[1], y=local_lin_accel[2], z=local_lin_accel[0]
    )

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
    rospy.Subscriber("/filter/free_acceleration", Vector3Stamped, accelDataCb)
    rospy.Subscriber("/filter/quaternion", QuaternionStamped, quatDataCb)
    rospy.Subscriber("/filter/twist", TwistStamped, twistDataCb)

    rospy.spin()
