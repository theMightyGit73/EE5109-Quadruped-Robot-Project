#!/usr/bin/env python3
# Author: Claude

import numpy as np
import rospy

class LQR_controller(object):
    """
    LQR controller for stabilizing roll and pitch angles of a quadruped robot.
    The controller uses a discrete-time LQR approach with a state vector of
    [roll, pitch, roll_rate, pitch_rate].
    """
    def __init__(self, q_angle=0.8, q_rate=0.05, r_input=0.005, expected_dt=0.02, max_compensation=0.5):
        """
        Initialize the LQR controller.
        
        Args:
            q_angle: Weight for angle error in the Q matrix
            q_rate: Weight for angular rate error in the Q matrix
            r_input: Weight for control effort in the R matrix
            expected_dt: Expected time step for the discrete LQR controller
            max_compensation: Maximum compensation value to apply
        """
        # Controller tuning parameters
        self.q_angle = q_angle
        self.q_rate = q_rate
        self.r_input = r_input
        self.expected_dt = expected_dt
        self.max_compensation = max_compensation
        
        # State space matrices for roll-pitch dynamics
        # Based on simplified rigid body dynamics for small angles
        # State vector: [roll, pitch, roll_rate, pitch_rate]
        # Control input: [roll_torque, pitch_torque]
        
        # Initialize state variables
        self.last_roll = 0.0
        self.last_pitch = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.debug_counter = 0
        
        # Setup time tracking
        self.last_time = rospy.Time.now()
        
        # Generate initial LQR gain matrix
        self.calculate_lqr_gain(expected_dt)
    
    def calculate_lqr_gain(self, dt):
        """
        Calculate the LQR gain matrix based on the system dynamics and weights.
        
        Args:
            dt: Time step for discretization
        """
        # System matrices for simplified roll-pitch dynamics
        # Based on small angle approximation and decoupled roll/pitch
        
        # Get inertia values from URDF
        # For notspot robot, the relevant inertia values are:
        # ixx = 0.00058474 (roll)
        # iyy = 0.00029699 (pitch)
        
        # We model the system as:
        # x̄̇ = Ax̄ + Bu
        #
        # Where:
        # x̄ = [roll, pitch, roll_rate, pitch_rate]
        # u = [roll_torque, pitch_torque]
        
        # Inertia values from URDF
        Ixx = 0.00058474  # roll inertia
        Iyy = 0.00029699  # pitch inertia
        
        # Continuous time state-space model
        # For each angle (roll, pitch):
        # θ̈ = τ/I  (torque / inertia)
        
        # State matrix A (continuous time)
        Ac = np.array([
            [0, 0, 1, 0],      # roll_rate = roll_rate
            [0, 0, 0, 1],      # pitch_rate = pitch_rate
            [0, 0, 0, 0],      # roll_accel = f(torque)
            [0, 0, 0, 0]       # pitch_accel = f(torque)
        ])
        
        # Input matrix B (continuous time)
        Bc = np.array([
            [0, 0],            # roll is not directly affected by torque
            [0, 0],            # pitch is not directly affected by torque
            [1/Ixx, 0],        # roll_accel = roll_torque / Ixx
            [0, 1/Iyy]         # pitch_accel = pitch_torque / Iyy
        ])
        
        # Discretize the system: x[k+1] = Ad*x[k] + Bd*u[k]
        # Using first-order approximation for simplicity
        Ad = np.eye(4) + dt * Ac
        Bd = dt * Bc
        
        # Weight matrices for LQR cost function
        Q = np.array([
            [self.q_angle, 0, 0, 0],
            [0, self.q_angle, 0, 0],
            [0, 0, self.q_rate, 0],
            [0, 0, 0, self.q_rate]
        ])
        
        R = np.array([
            [self.r_input, 0],
            [0, self.r_input]
        ])
        
        # Solve the discrete-time Riccati equation
        # This is a simplified version that works for this problem
        # For more general cases, scipy.linalg.solve_discrete_are should be used
        P = self._solve_discrete_riccati(Ad, Bd, Q, R)
        
        # Calculate the LQR gain matrix K
        self.K = np.linalg.inv(R + Bd.T @ P @ Bd) @ Bd.T @ P @ Ad
        
        # Store the discretized matrices for future use
        self.Ad = Ad
        self.Bd = Bd
    
    def _solve_discrete_riccati(self, A, B, Q, R, max_iter=1000, tol=1e-8):
        """
        Solve the discrete-time algebraic Riccati equation (DARE).
        
        A simplified implementation of the iterative solution to DARE:
        P = A.T @ P @ A - A.T @ P @ B @ inv(R + B.T @ P @ B) @ B.T @ P @ A + Q
        
        Args:
            A, B: System matrices
            Q, R: Weight matrices
            max_iter: Maximum iterations
            tol: Convergence tolerance
            
        Returns:
            P: Solution to the DARE
        """
        # Initialize P matrix
        P = Q.copy()
        
        # Iterative solution
        for _ in range(max_iter):
            P_new = A.T @ P @ A - \
                   (A.T @ P @ B) @ np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A) + \
                   Q
            
            # Check convergence
            if np.max(np.abs(P - P_new)) < tol:
                return P_new
            
            P = P_new
        
        # Return the best estimate if not converged
        return P
    
    def run(self, roll, pitch):
        """
        Run one step of the LQR controller.
        """
        # Calculate time step
        t_now = rospy.Time.now()
        dt = (t_now - self.last_time).to_sec()
        
        # Use expected_dt if dt is too small (avoid division by zero or numerical issues)
        if dt < 0.001:
            dt = self.expected_dt
        
        # Update LQR gain if dt is significantly different from expected
        if abs(dt - self.expected_dt) > 0.01:
            self.calculate_lqr_gain(dt)
            rospy.loginfo("LQR: Updated gain matrix with dt=%f", dt)
        
        # Estimate angular rates (finite differences)
        self.roll_rate = (roll - self.last_roll) / dt
        self.pitch_rate = (pitch - self.last_pitch) / dt
        
        # Current state vector [roll, pitch, roll_rate, pitch_rate]
        x = np.array([roll, pitch, self.roll_rate, self.pitch_rate])
        
        # Calculate control input u = -K*x
        u = -self.K @ x
        
        # Debug prints
        if self.debug_counter % 30 == 0:  # Print every 30 iterations to avoid flooding
            rospy.loginfo("LQR: IMU Angles (rad): roll=%.4f, pitch=%.4f", roll, pitch)
            rospy.loginfo("LQR: Rates (rad/s): roll_rate=%.4f, pitch_rate=%.4f", 
                        self.roll_rate, self.pitch_rate)
            rospy.loginfo("LQR: Raw output: u_roll=%.4f, u_pitch=%.4f", u[0], u[1])
        self.debug_counter += 1
        
        # Limit maximum compensation
        for i in range(2):
            if u[i] > self.max_compensation:
                u[i] = self.max_compensation
                if self.debug_counter % 30 == 1:
                    rospy.logwarn("LQR: Compensation limiting applied (max) on axis %d", i)
            elif u[i] < -self.max_compensation:
                u[i] = -self.max_compensation
                if self.debug_counter % 30 == 1:
                    rospy.logwarn("LQR: Compensation limiting applied (min) on axis %d", i)
        
        # Update last values
        self.last_time = t_now
        self.last_roll = roll
        self.last_pitch = pitch
        
        # Final debug print
        if self.debug_counter % 30 == 2:
            rospy.loginfo("LQR: Final compensation: roll=%.4f, pitch=%.4f", u[0], u[1])
        
        # Return compensation values
        return u
    
    def reset(self):
        """
        Reset the controller state.
        """
        self.last_roll = 0.0
        self.last_pitch = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.last_time = rospy.Time.now()