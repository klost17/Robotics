#! /usr/bin/env python3

"""
    # Alexandre Justo Miro
    # aljm@kth.se
"""

import numpy as np
import math

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    """
    Fill in your IK solution here and return the three joint values in q
    """
    l_0 = 0.07 # x distance between BASE FRAME and 1st JOINT (R)
    l_1 = 0.30 # x distance between 1st JOINT (R) and 2nd JOINT (R)
    l_2 = 0.35 # x distance between 2nd JOINT (R) and 3rd JOINT (P, END-EFFECTOR FRAME)

    x = x-l_0 # The origin is displaced a distance l_0 in the x coordinate

    # The inverse kinematics calculations were done by hand
    q_1 = math.atan2(y,x) - math.acos((x**2 + y**2 + l_1**2 - l_2**2)/(2*l_1*np.sqrt(x**2+y**2)))
    q_2 = math.acos((x**2 + y**2 - l_1**2 - l_2**2)/(2*l_1*l_2))
    q_3 = z

    q = [q_1, q_2, q_3]

    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """
    L = 0.40
    M = 0.39

    # Denavit-Hartenberg parameters
    th_1 = q[0]
    th_2 = q[1]
    th_3 = q[2]
    th_4 = q[3]
    th_5 = q[4]
    th_6 = q[5]
    th_7 = q[6]
    alpha_1 = math.pi/2
    alpha_2 = -math.pi/2
    alpha_3 = -math.pi/2
    alpha_4 = math.pi/2
    alpha_5 = math.pi/2
    alpha_6 = -math.pi/2
    alpha_7 = 0
    d_1 = 0.311
    d_2 = 0
    d_3 = L
    d_4 = 0
    d_5 = M
    d_6 = 0
    d_7 = 0.078
    a_1 = 0
    a_2 = 0
    a_3 = 0
    a_4 = 0
    a_5 = 0
    a_6 = 0
    a_7 = 0

    # First dummy value for initializing the while loop
    eps_X = np.array([ 1, 1, 1, 1, 1, 1 ])

    # Target value for X (orientation)
    n_e = np.array([ R[0][0], R[1][0], R[2][0] ])
    s_e = np.array([ R[0][1], R[1][1], R[2][1] ])
    a_e = np.array([ R[0][2], R[1][2], R[2][2] ])

    # Initial approximation for Theta
    Th_hat = q

    # Tolerance for X
    tol = 0.00001

    while np.linalg.norm(eps_X) > tol:

        # Transformation matrices
        T_0_1 = np.array([ [np.cos(th_1), -np.sin(th_1)*np.cos(alpha_1), np.sin(th_1)*np.sin(alpha_1), a_1*np.cos(th_1)], [np.sin(th_1), np.cos(th_1)*np.cos(alpha_1), -np.cos(th_1)*np.sin(alpha_1), a_1*np.sin(th_1)], [0, np.sin(alpha_1), np.cos(alpha_1), d_1], [0, 0, 0, 1] ])
        T_1_2 = np.array([ [np.cos(th_2), -np.sin(th_2)*np.cos(alpha_2), np.sin(th_2)*np.sin(alpha_2), a_2*np.cos(th_2)], [np.sin(th_2), np.cos(th_2)*np.cos(alpha_2), -np.cos(th_2)*np.sin(alpha_2), a_2*np.sin(th_2)], [0, np.sin(alpha_2), np.cos(alpha_2), d_2], [0, 0, 0, 1] ])
        T_2_3 = np.array([ [np.cos(th_3), -np.sin(th_3)*np.cos(alpha_3), np.sin(th_3)*np.sin(alpha_3), a_3*np.cos(th_3)], [np.sin(th_3), np.cos(th_3)*np.cos(alpha_3), -np.cos(th_3)*np.sin(alpha_3), a_3*np.sin(th_3)], [0, np.sin(alpha_3), np.cos(alpha_3), d_3], [0, 0, 0, 1] ])
        T_3_4 = np.array([ [np.cos(th_4), -np.sin(th_4)*np.cos(alpha_4), np.sin(th_4)*np.sin(alpha_4), a_4*np.cos(th_4)], [np.sin(th_4), np.cos(th_4)*np.cos(alpha_4), -np.cos(th_4)*np.sin(alpha_4), a_4*np.sin(th_4)], [0, np.sin(alpha_4), np.cos(alpha_4), d_4], [0, 0, 0, 1] ])
        T_4_5 = np.array([ [np.cos(th_5), -np.sin(th_5)*np.cos(alpha_5), np.sin(th_5)*np.sin(alpha_5), a_5*np.cos(th_5)], [np.sin(th_5), np.cos(th_5)*np.cos(alpha_5), -np.cos(th_5)*np.sin(alpha_5), a_5*np.sin(th_5)], [0, np.sin(alpha_5), np.cos(alpha_5), d_5], [0, 0, 0, 1] ])
        T_5_6 = np.array([ [np.cos(th_6), -np.sin(th_6)*np.cos(alpha_6), np.sin(th_6)*np.sin(alpha_6), a_6*np.cos(th_6)], [np.sin(th_6), np.cos(th_6)*np.cos(alpha_6), -np.cos(th_6)*np.sin(alpha_6), a_6*np.sin(th_6)], [0, np.sin(alpha_6), np.cos(alpha_6), d_6], [0, 0, 0, 1] ])
        T_6_7 = np.array([ [np.cos(th_7), -np.sin(th_7)*np.cos(alpha_7), np.sin(th_7)*np.sin(alpha_7), a_7*np.cos(th_7)], [np.sin(th_7), np.cos(th_7)*np.cos(alpha_7), -np.cos(th_7)*np.sin(alpha_7), a_7*np.sin(th_7)], [0, np.sin(alpha_7), np.cos(alpha_7), d_7], [0, 0, 0, 1] ])
        
        # Matrix products
        T_0_2 = np.dot(T_0_1, T_1_2)
        T_0_3 = np.dot(T_0_2, T_2_3)
        T_0_4 = np.dot(T_0_3, T_3_4)
        T_0_5 = np.dot(T_0_4, T_4_5)
        T_0_6 = np.dot(T_0_5, T_5_6)

        # Transformation matrix from end-effector to base
        T_0_E = np.dot(T_0_6, T_6_7)

        # Save auxiliary z vectors for Jacobian
        z_1 = np.array([0,0,1])
        z_2 = T_0_1[[0,1,2],2]
        z_3 = T_0_2[[0,1,2],2]
        z_4 = T_0_3[[0,1,2],2]
        z_5 = T_0_4[[0,1,2],2]
        z_6 = T_0_5[[0,1,2],2]
        z_7 = T_0_6[[0,1,2],2]
        # Save auxiliary p vectors for Jacobian
        p_1 = T_0_1[[0,1,2],3]
        p_2 = T_0_2[[0,1,2],3]
        p_3 = T_0_3[[0,1,2],3]
        p_4 = T_0_4[[0,1,2],3]
        p_5 = T_0_5[[0,1,2],3]
        p_6 = T_0_6[[0,1,2],3]
        p_7 = T_0_E[[0,1,2],3]
        # Save auxiliary C vectors for Jacobian
        C_1 = np.cross(z_1, p_7-p_1)
        C_2 = np.cross(z_2, p_7-p_2)
        C_3 = np.cross(z_3, p_7-p_3)
        C_4 = np.cross(z_4, p_7-p_4)
        C_5 = np.cross(z_5, p_7-p_5)
        C_6 = np.cross(z_6, p_7-p_6)
        C_7 = np.cross(z_7, p_7-p_7)
        # Build the Jacobian
        C_1 = np.concatenate((C_1,z_1))
        C_2 = np.concatenate((C_2,z_2))
        C_3 = np.concatenate((C_3,z_3))
        C_4 = np.concatenate((C_4,z_4))
        C_5 = np.concatenate((C_5,z_5))
        C_6 = np.concatenate((C_6,z_6))
        C_7 = np.concatenate((C_7,z_7))
        J = np.array([C_1, C_2, C_3, C_4, C_5, C_6, C_7])
        J = np.transpose(J)

        # Compute the pseudo-inverse (non-square matrix) of the Jacobian
        J_inv = np.linalg.pinv(J)

        # Iterative algorithm for finding inverse kinematics
        xyz1 = T_0_E[[0,1,2],3]
        eps_P = xyz1-point
        eps_O = 0.5*( np.cross(T_0_E[[0,1,2],0], n_e) + np.cross(T_0_E[[0,1,2],1], s_e) + np.cross(T_0_E[[0,1,2],2], a_e) )
        eps_X = np.array([eps_P[0], eps_P[1], eps_P[2], eps_O[0], eps_O[1], eps_O[2]])
        eps_Th = np.dot(J_inv, eps_X)

        # Upgrade approximated Theta vector
        Th_hat = Th_hat - eps_Th

        # Update values of components inside Theta vector
        th_1 = Th_hat[0]
        th_2 = Th_hat[1]
        th_3 = Th_hat[2]
        th_4 = Th_hat[3]
        th_5 = Th_hat[4]
        th_6 = Th_hat[5]
        th_7 = Th_hat[6]

    q = Th_hat

    return q