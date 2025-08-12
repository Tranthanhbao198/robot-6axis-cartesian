import math
import numpy as np
from .kinematics import DHRobot

def deg(rad):
    """Convert radians to degrees."""
    return rad * 180.0 / math.pi

def rad(deg):
    """Convert degrees to radians."""
    return deg * math.pi / 180.0

class Simple6DOF(DHRobot):
    """
    6-DOF với bộ tham số DH chuẩn (standard DH) kiểu UR5 đã hiệu chỉnh.
    """
    def __init__(self):
        # Standard DH (validated for sensible zero pose)
        a =      [0.0,   -0.425,  -0.39225, 0.0,     0.0,    0.0]
        alpha =  [math.pi/2, 0.0,  0.0,     math.pi/2, -math.pi/2, 0.0]
        d =      [0.1625,     0.0,  0.0,     0.1333,    0.0997,    0.0996]
        theta_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # generous limits for sim
        q_lim = rad(360)
        q_min = np.full(6, -q_lim)
        q_max = np.full(6,  q_lim)

        super().__init__(a, alpha, d, theta_offset, q_min, q_max, name="Simple6DOF_Corrected")

