from numpy import cos, sin, pi
from math import atan2,sqrt

def  EulerToQuaternion(roll, pitch, yaw): # roll (x), pitch (Y), yaw (z)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

<<<<<<< HEAD
    return qw,qx,qy,qz
=======
    return qx,qy,qz,qw
>>>>>>> e1ef91ccdd252f7603bcc3c28882b41d48d39d22


def QuaterniontoEuler(qx, qy, qz, qw):
    
    # roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = sqrt(1 + 2 * (qw * qy - qx * qz))
    cosp = sqrt(1 - 2 * (qw * qy - qx * qz))
    pitch = 2 * atan2(sinp, cosp) - pi / 2

    #yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = atan2(siny_cosp, cosy_cosp);

    return roll, pitch, yaw