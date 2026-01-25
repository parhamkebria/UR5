#!/usr/bin/env python3
"""
UR5/UR10 Inverse Kinematics - Parham Kebria

This module provides functions to compute the forward and inverse kinematics
of the Universal Robots UR5 and UR10 robotic arms using the Denavit-Hartenberg
convention.
"""

import cmath
import numpy as np
from numpy import linalg
from math import cos, sin, atan2, acos, asin, sqrt, pi

# Use numpy matrix for compatibility with existing code
MATRIX = np.matrix

# Robot model selection (set one to True, others to False)
UR5 = True
UR10 = False

# Denavit-Hartenberg parameters (in meters)
if UR10:
	d1 = 0.1273
	a2 = -0.612
	a3 = -0.5723
	a7 = 0.075
	d4 = 0.163941
	d5 = 0.1157
	d6 = 0.0922
	d = MATRIX([0.1273, 0, 0, 0.163941, 0.1157, 0.0922]) # mm
	a = MATRIX([0 ,-0.612 ,-0.5723 ,0 ,0 ,0]) # mm
	alph = MATRIX([pi/2, 0, 0, pi/2, -pi/2, 0 ])

elif UR5:
    # UR5 DH parameters
    d1 = 0.089159
    a2 = -0.42500
    a3 = -0.39225
    d4 = 0.10915
    d5 = 0.09465
    d6 = 0.0823
    d = MATRIX([0.089159, 0, 0, 0.10915, 0.09465, 0.0823])
    a = MATRIX([0, -0.425, -0.39225, 0, 0, 0])
    alph = MATRIX([pi/2, 0, 0, pi/2, -pi/2, 0])


# ============================================================================
# FORWARD KINEMATICS
# ============================================================================

def AH(n, th, c):
    """
    Compute the Denavit-Hartenberg transformation matrix for joint n.
    
    Args:
        n: Joint number (1-6)
        th: Joint angles matrix (6x8) containing all solution configurations
        c: Column index specifying which configuration to use
    
    Returns:
        4x4 homogeneous transformation matrix from joint n-1 to joint n
    """
    T_a = MATRIX(np.identity(4), copy=False)
    T_a[0, 3] = a[0, n-1]
    T_d = MATRIX(np.identity(4), copy=False)
    T_d[2, 3] = d[0, n-1]

    Rzt = MATRIX(
        [
            [cos(th[n-1, c]), -sin(th[n-1, c]), 0, 0],
            [sin(th[n-1, c]),  cos(th[n-1, c]), 0, 0],
            [0,                0,               1, 0],
            [0,                0,               0, 1]
        ],
        copy=False)

    Rxa = MATRIX(
        [
            [1, 0,                  0,                   0],
            [0, cos(alph[0, n-1]), -sin(alph[0, n-1]),   0],
            [0, sin(alph[0, n-1]),  cos(alph[0, n-1]),   0],
            [0, 0,                  0,                   1]
        ],
        copy=False)

    A_i = T_d * Rzt * T_a * Rxa

    return A_i


def HTrans(th, c):
    """
    Compute the homogeneous transformation matrix from base to end-effector.
    
    Args:
        th: Joint angles matrix (6x8) containing all solution configurations
        c: Column index specifying which configuration to use
    
    Returns:
        4x4 homogeneous transformation matrix from base to end-effector
    """
    A_1 = AH(1, th, c)
    A_2 = AH(2, th, c)
    A_3 = AH(3, th, c)
    A_4 = AH(4, th, c)
    A_5 = AH(5, th, c)
    A_6 = AH(6, th, c)
        
    T_06 = A_1 * A_2 * A_3 * A_4 * A_5 * A_6

    return T_06


# ============================================================================
# INVERSE KINEMATICS
# ============================================================================

def invKine(desired_pos):
    """
    Compute inverse kinematics for UR5/UR10 robot arm.
    
    This function calculates all possible joint angle solutions (up to 8)
    for a given desired end-effector pose.
    
    Args:
        desired_pos: 4x4 homogeneous transformation matrix representing
                    the desired end-effector pose (position and orientation)
    
    Returns:
        6x8 matrix where each column represents one of up to 8 possible
        joint angle solutions [theta1, theta2, theta3, theta4, theta5, theta6]
    """
    th = MATRIX(np.zeros((6, 8)))
    P_05 = (desired_pos * MATRIX([0, 0, -d6, 1]).T - MATRIX([0, 0, 0, 1]).T)
    
    # Joint 1 (base rotation)
    # Two solutions correspond to the shoulder being either left or right
    psi = atan2(P_05[1, 0], P_05[0, 0])
    phi = acos(d4 / sqrt(P_05[1, 0]**2 + P_05[0, 0]**2))
    th[0, 0:4] = pi/2 + psi + phi
    th[0, 4:8] = pi/2 + psi - phi
    th = th.real
    
    # Joint 5 (wrist 2) - wrist up or down configurations
    cl = [0, 4]
    for i in range(len(cl)):
        c = cl[i]
        T_10 = linalg.inv(AH(1, th, c))
        T_16 = T_10 * desired_pos
        th[4, c:c+2] = +acos((T_16[2, 3] - d4) / d6)
        th[4, c+2:c+4] = -acos((T_16[2, 3] - d4) / d6)

    th = th.real

    # Joint 6 (wrist 3)
    # Note: Solution is not well-defined when sin(theta5) = 0
    cl = [0, 2, 4, 6]
    for i in range(len(cl)):
        c = cl[i]
        T_10 = linalg.inv(AH(1, th, c))
        T_16 = linalg.inv(T_10 * desired_pos)
        th[5, c:c+2] = atan2((-T_16[1, 2] / sin(th[4, c])),
                            (T_16[0, 2] / sin(th[4, c])))
            
    th = th.real

    # Joint 3 (elbow) - elbow up or down configurations
    cl = [0, 2, 4, 6]
    for i in range(len(cl)):
        c = cl[i]
        T_10 = linalg.inv(AH(1, th, c))
        T_65 = AH(6, th, c)
        T_54 = AH(5, th, c)
        T_14 = (T_10 * desired_pos) * linalg.inv(T_54 * T_65)
        P_13 = T_14 * MATRIX([0, -d4, 0, 1]).T - MATRIX([0, 0, 0, 1]).T
        t3 = cmath.acos((linalg.norm(P_13)**2 - a2**2 - a3**2) / (2 * a2 * a3))
        th[2, c] = t3.real
        th[2, c+1] = -t3.real

    # Joints 2 and 4 (shoulder and wrist 1)
    cl = [0, 1, 2, 3, 4, 5, 6, 7]
    for i in range(len(cl)):
        c = cl[i]
        T_10 = linalg.inv(AH(1, th, c))
        T_65 = linalg.inv(AH(6, th, c))
        T_54 = linalg.inv(AH(5, th, c))
        T_14 = (T_10 * desired_pos) * T_65 * T_54
        P_13 = T_14 * MATRIX([0, -d4, 0, 1]).T - MATRIX([0, 0, 0, 1]).T
        
        # Joint 2 (shoulder)
        th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(a3 * sin(th[2, c]) / linalg.norm(P_13))
        
        # Joint 4 (wrist 1)
        T_32 = linalg.inv(AH(3, th, c))
        T_21 = linalg.inv(AH(2, th, c))
        T_34 = T_32 * T_21 * T_14
        th[3, c] = atan2(T_34[1, 0], T_34[0, 0])
    
    th = th.real

    return th