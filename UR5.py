"""
UR5 Robot Dynamics and Kinematics
This script computes the DH parameters, forward kinematics, Jacobian, 
mass matrix, Coriolis matrix, and gravity vector for the UR5 robot.
"""

import numpy as np
import sympy as sp
from sympy import symbols, cos, sin, pi, eye, diff, Matrix, zeros
import pickle

# DH and Inertia Parameters of UR5
alpha = [0, pi/2, 0, 0, -pi/2, pi/2]
a = [0, 0, 0.425, 0.39225, 0, 0]
d = [0.08916, 0, 0, 0.10915, 0.09456, 0.0823]
m = [3.7, 8.393, 2.275, 1.219, 1.219, 0.1879]
p_c1 = np.array([0, -1.93, -26.51]) * 1e-3
p_c2 = np.array([212.5, 0, 113.36]) * 1e-3
p_c3 = np.array([272.32, 0, 26.5]) * 1e-3
p_c4 = np.array([0, 16, 34.107, 35]) * 1e-3  # Note: MATLAB had 4 elements, using first 3 + average
p_c5 = np.array([0, -16.34, -1.8]) * 1e-3
p_c6 = np.array([0, 0, -1.159]) * 1e-3

In_1 = np.array([[84, 0, 0], [0, 64, 0], [0, 0, 84]]) * 1e-4
In_2 = np.array([[78, 0, 0], [0, 21, 0], [0, 0, 21]]) * 1e-4
In_3 = np.array([[16, 0, 0], [0, 462, 0], [0, 0, 462]]) * 1e-4
In_4 = np.array([[16, 0, 0], [0, 16, 0], [0, 0, 9]]) * 1e-4
In_5 = np.array([[16, 0, 0], [0, 16, 0], [0, 0, 9]]) * 1e-4
In_6 = np.eye(3) * 1e-4

# PARAMETERS AND SYMBOLS
g = symbols('g')
alpha_0, alpha_1, alpha_2, alpha_3, alpha_4, alpha_5 = alpha
a_0, a_1, a_2, a_3, a_4, a_5 = a
d_1, d_2, d_3, d_4, d_5, d_6 = d

p_cx1, p_cy1, p_cz1 = sp.nsimplify([float(p_c1[0]), float(p_c1[1]), float(p_c1[2])], rational=False)
p_cx2, p_cy2, p_cz2 = sp.nsimplify([float(p_c2[0]), float(p_c2[1]), float(p_c2[2])], rational=False)
p_cx3, p_cy3, p_cz3 = sp.nsimplify([float(p_c3[0]), float(p_c3[1]), float(p_c3[2])], rational=False)
p_cx4, p_cy4, p_cz4 = sp.nsimplify([float(p_c4[0]), float(p_c4[1]), float(p_c4[2])], rational=False)
p_cx5, p_cy5, p_cz5 = sp.nsimplify([float(p_c5[0]), float(p_c5[1]), float(p_c5[2])], rational=False)
p_cx6, p_cy6, p_cz6 = sp.nsimplify([float(p_c6[0]), float(p_c6[1]), float(p_c6[2])], rational=False)

m_1, m_2, m_3, m_4, m_5, m_6 = m
q_1, q_2, q_3, q_4, q_5, q_6 = symbols('q_1 q_2 q_3 q_4 q_5 q_6')
dq_1, dq_2, dq_3, dq_4, dq_5, dq_6 = symbols('dq_1 dq_2 dq_3 dq_4 dq_5 dq_6')

# Convert numpy inertia matrices to sympy
In_1_sym = Matrix(In_1)
In_2_sym = Matrix(In_2)
In_3_sym = Matrix(In_3)
In_4_sym = Matrix(In_4)
In_5_sym = Matrix(In_5)
In_6_sym = Matrix(In_6)

# ROTATION MATRICES
R_1 = Matrix([
    [cos(q_1), -sin(q_1), 0],
    [sin(q_1)*cos(alpha_0), cos(q_1)*cos(alpha_0), -sin(alpha_0)],
    [sin(q_1)*sin(alpha_0), cos(q_1)*sin(alpha_0), cos(alpha_0)]
])

R_2 = Matrix([
    [cos(q_2), -sin(q_2), 0],
    [sin(q_2)*cos(alpha_1), cos(q_2)*cos(alpha_1), -sin(alpha_1)],
    [sin(q_2)*sin(alpha_1), cos(q_2)*sin(alpha_1), cos(alpha_1)]
])

R_3 = Matrix([
    [cos(q_3), -sin(q_3), 0],
    [sin(q_3)*cos(alpha_2), cos(q_3)*cos(alpha_2), -sin(alpha_2)],
    [sin(q_3)*sin(alpha_2), cos(q_3)*sin(alpha_2), cos(alpha_2)]
])

R_4 = Matrix([
    [cos(q_4), -sin(q_4), 0],
    [sin(q_4)*cos(alpha_3), cos(q_4)*cos(alpha_3), -sin(alpha_3)],
    [sin(q_4)*sin(alpha_3), cos(q_4)*sin(alpha_3), cos(alpha_3)]
])

R_5 = Matrix([
    [cos(q_5), -sin(q_5), 0],
    [sin(q_5)*cos(alpha_4), cos(q_5)*cos(alpha_4), -sin(alpha_4)],
    [sin(q_5)*sin(alpha_4), cos(q_5)*sin(alpha_4), cos(alpha_4)]
])

R_6 = Matrix([
    [cos(q_6), -sin(q_6), 0],
    [sin(q_6)*cos(alpha_5), cos(q_6)*cos(alpha_5), -sin(alpha_5)],
    [sin(q_6)*sin(alpha_5), cos(q_6)*sin(alpha_5), cos(alpha_5)]
])

# POSITION VECTORS
p_1 = Matrix([a_0, -sin(alpha_0)*d_1, cos(alpha_0)*d_1])
p_2 = Matrix([a_1, -sin(alpha_1)*d_2, cos(alpha_1)*d_2])
p_3 = Matrix([a_2, -sin(alpha_2)*d_3, cos(alpha_2)*d_3])
p_4 = Matrix([a_3, -sin(alpha_3)*d_4, cos(alpha_3)*d_4])
p_5 = Matrix([a_4, -sin(alpha_4)*d_5, cos(alpha_4)*d_5])
p_6 = Matrix([a_5, -sin(alpha_5)*d_6, cos(alpha_5)*d_6])

# TRANSFORMATION MATRICES AND FORWARD KINEMATICS
T_1 = Matrix.vstack(Matrix.hstack(R_1, p_1), Matrix([[0, 0, 0, 1]]))
T_2 = Matrix.vstack(Matrix.hstack(R_2, p_2), Matrix([[0, 0, 0, 1]]))
T_3 = Matrix.vstack(Matrix.hstack(R_3, p_3), Matrix([[0, 0, 0, 1]]))
T_4 = Matrix.vstack(Matrix.hstack(R_4, p_4), Matrix([[0, 0, 0, 1]]))
T_5 = Matrix.vstack(Matrix.hstack(R_5, p_5), Matrix([[0, 0, 0, 1]]))
T_6 = Matrix.vstack(Matrix.hstack(R_6, p_6), Matrix([[0, 0, 0, 1]]))
T = T_1 * T_2 * T_3 * T_4 * T_5 * T_6

# COMs' POSITION VECTORS
p_c1_vec = p_1 + R_1 * Matrix([p_cx1, p_cy1, p_cz1])
p_c2_vec = p_1 + R_1 * (p_2 + R_2 * Matrix([p_cx2, p_cy2, p_cz2]))
p_c3_vec = p_1 + R_1 * (p_2 + R_2 * (p_3 + R_3 * Matrix([p_cx3, p_cy3, p_cz3])))
p_c4_vec = p_1 + R_1 * (p_2 + R_2 * (p_3 + R_3 * (p_4 + R_4 * Matrix([p_cx4, p_cy4, p_cz4]))))
p_c5_vec = p_1 + R_1 * (p_2 + R_2 * (p_3 + R_3 * (p_4 + R_4 * (p_5 + R_5 * Matrix([p_cx5, p_cy5, p_cz5])))))
p_c6_vec = p_1 + R_1 * (p_2 + R_2 * (p_3 + R_3 * (p_4 + R_4 * (p_5 + R_5 * (p_6 + R_6 * Matrix([p_cx6, p_cy6, p_cz6]))))))

# SYSTEM's STATES
q = Matrix([q_1, q_2, q_3, q_4, q_5, q_6])

# LINEAR PART of JACOBIANS
print("Computing Jacobians...")
J_v1 = p_c1_vec.jacobian(q)
J_v2 = p_c2_vec.jacobian(q)
J_v3 = p_c3_vec.jacobian(q)
J_v4 = p_c4_vec.jacobian(q)
J_v5 = p_c5_vec.jacobian(q)
J_v6 = p_c6_vec.jacobian(q)

# ROTATION MATRICES FROM BASE
R_20 = R_1 * R_2
R_30 = R_20 * R_3
R_40 = R_30 * R_4
R_50 = R_40 * R_5
R_60 = R_50 * R_6

# ANGULAR PART of JACOBIANS
z_zeros_5 = zeros(3, 5)
z_zeros_4 = zeros(3, 4)
z_zeros_3 = zeros(3, 3)
z_zeros_2 = zeros(3, 2)
z_zeros_1 = zeros(3, 1)

J_o1 = Matrix.hstack(R_1[:, 2], z_zeros_5)
J_o2 = Matrix.hstack(R_1[:, 2], R_20[:, 2], z_zeros_4)
J_o3 = Matrix.hstack(R_1[:, 2], R_20[:, 2], R_30[:, 2], z_zeros_3)
J_o4 = Matrix.hstack(R_1[:, 2], R_20[:, 2], R_30[:, 2], R_40[:, 2], z_zeros_2)
J_o5 = Matrix.hstack(R_1[:, 2], R_20[:, 2], R_30[:, 2], R_40[:, 2], R_50[:, 2], z_zeros_1)
J_o6 = Matrix.hstack(R_1[:, 2], R_20[:, 2], R_30[:, 2], R_40[:, 2], R_50[:, 2], R_60[:, 2])

# JACOBIAN MATRIX OF THE END-EFFECTOR
Jacobi = Matrix.vstack(J_v6, J_o6)

# ROBOT's INERTIA (MASS) MATRIX
print("Computing Mass Matrix...")
M = (J_v1.T * m_1 * eye(3) * J_v1 + J_o1.T * R_1 * In_1_sym * R_1.T * J_o1 +
     J_v2.T * m_2 * eye(3) * J_v2 + J_o2.T * R_20 * In_2_sym * R_20.T * J_o2 +
     J_v3.T * m_3 * eye(3) * J_v3 + J_o3.T * R_30 * In_3_sym * R_30.T * J_o3 +
     J_v4.T * m_4 * eye(3) * J_v4 + J_o4.T * R_40 * In_4_sym * R_40.T * J_o4 +
     J_v5.T * m_5 * eye(3) * J_v5 + J_o5.T * R_50 * In_5_sym * R_50.T * J_o5 +
     J_v6.T * m_6 * eye(3) * J_v6 + J_o6.T * R_60 * In_6_sym * R_60.T * J_o6)

# CORIOLIS and CENTRIFUGAL MATRIX
print("Computing Coriolis Matrix...")
C = zeros(6, 6)
dq = [dq_1, dq_2, dq_3, dq_4, dq_5, dq_6]
for k in range(6):
    for s in range(6):
        C_ks = 0
        for j in range(6):
            C_ks += (diff(M[k, s], q[j]) + diff(M[k, j], q[s]) - diff(M[j, s], q[k])) * dq[j]
        C[k, s] = 0.5 * C_ks

# POTENTIAL ENERGIES and GRAVITY VECTOR
print("Computing Gravity Vector...")
P1 = m_1 * Matrix([0, 0, g]).dot(p_c1_vec)
P2 = m_2 * Matrix([0, 0, g]).dot(p_c2_vec)
P3 = m_3 * Matrix([0, 0, g]).dot(p_c3_vec)
P4 = m_4 * Matrix([0, 0, g]).dot(p_c4_vec)
P5 = m_5 * Matrix([0, 0, g]).dot(p_c5_vec)
P6 = m_6 * Matrix([0, 0, g]).dot(p_c6_vec)
P = P1 + P2 + P3 + P4 + P5 + P6

g_1 = diff(P, q_1)
g_2 = diff(P, q_2)
g_3 = diff(P, q_3)
g_4 = diff(P, q_4)
g_5 = diff(P, q_5)
g_6 = diff(P, q_6)
G = Matrix([g_1, g_2, g_3, g_4, g_5, g_6])

# DYNAMICAL EQUATIONS of MOTION
# M(q)*ddq + C(q,dq)*dq + G(q) = u

# Save results
print("Saving results...")
results = {
    'T': T,
    'Jacobi': Jacobi,
    'M': M,
    'C': C,
    'G': G
}

with open('UR5.pkl', 'wb') as f:
    pickle.dump(results, f)

# Save to text files
with open('UR5T.txt', 'w') as f:
    f.write(str(T))

with open('UR5M.txt', 'w') as f:
    f.write(str(M))

with open('UR5C.txt', 'w') as f:
    f.write(str(C))

with open('UR5G.txt', 'w') as f:
    f.write(str(G))

with open('UR5J.txt', 'w') as f:
    f.write(str(Jacobi))

print("UR5 dynamics computation complete!")
print("Results saved to UR5.pkl and text files.")
