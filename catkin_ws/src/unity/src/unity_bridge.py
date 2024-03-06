#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Quaternion, Vector3
from humanoid_msgs.msg import UnityState
import quaternion


def unityStateCb(msg):
    unityQuat = msg.quat
    unityAngVel = msg.ang_vel
    unityLinAccel = msg.local_lin_accel
    unityGlobalVel = msg.global_vel

    quat  = q_NWU_NED * np.quaternion(unityQuat.w, unityQuat.x, unityQuat.y, unityQuat.z) * q_NWU_NED.conj()
    ros_quat = Quaternion(x=quat.x, y=quat.y, z=quat.z, w=quat.w)

    global_vel = Vector3(x=unityGlobalVel.z, y=-unityGlobalVel.x, z=unityGlobalVel.y)
    
    global_lin_accel = np.array([unityLinAccel.z, -unityLinAccel.x, unityLinAccel.y])
    local_lin_accel = quat * global_lin_accel * quat.conj()
    ros_local_lin = Vector3(x=local_lin_accel[0], y=local_lin_accel[1], z=local_lin_accel[2])

    global_ang_vel = np.array([unityAngVel.z, -unityAngVel.x, unityAngVel.y])
    local_ang_vel = quat * global_ang_vel * quat.conj() 
    ros_ang_vel = Vector3(x=local_ang_vel[0], y=local_ang_vel[1], z=local_ang_vel[2])

    pub_global_vel.publish(global_vel)
    pub_quat.publish(ros_quat)
    pub_ang_vel.publish(ros_ang_vel)
    pub_local_lin.publish(ros_local_lin)
    

if __name__ == '__main__':
    rospy.init_node('unity_bridge')

    # publishers
    pub_local_lin = rospy.Publisher('/state/local_lin_accel', Vector3, queue_size=1)
    pub_quat = rospy.Publisher('/state/quat', Quaternion, queue_size=1)
    pub_ang_vel = rospy.Publisher('/state/ang_vel', Vector3, queue_size=1)
    pub_global_vel = rospy.Publisher('/state/global_vel', Vector3, queue_size=1)

    # REFERENCE FRAME DEFINITIONS
    q_NWU_NED = np.quaternion(0,1,0,0)
    
    # subscribers
    rospy.Subscriber('/unity/state', UnityState, unityStateCb)

    rospy.spin()