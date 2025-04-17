#!/usr/bin/env python3
#Author: lnotspotl

import rospy
import numpy as np
from RoboticsUtilities.Transformations import rotxyz
from . PIDController import PID_controller
from . LQRController import LQR_controller  # Add this import

class RestController(object):
    def __init__(self, default_stance):
        self.def_stance = default_stance

        # Initialize PID controller
        self.pid_controller = PID_controller(0.75, 2.29, 0.0)
        
        # Initialize LQR controller with tuned parameters
        self.lqr_controller = LQR_controller(
            q_angle=1.2,        # Increased for better disturbance rejection
            q_rate=0.12,        # Increased for better damping
            r_input=0.003,      # Decreased slightly for more responsive control
            expected_dt=0.002,  # Expected control loop time step
            max_compensation=0.6 # Increased slightly for better recovery
        )
        
        self.use_imu = False
        self.use_button = True
        self.use_lqr = False  # Add this flag to track which controller to use
        
        # Reset both controllers initially
        self.pid_controller.reset()
        self.lqr_controller.reset()
        
    def updateStateCommand(self, msg, state, command):
        # local body position
        state.body_local_position[0] = msg.axes[7] * 0.03
        state.body_local_position[1] = msg.axes[6] * 0.03
        state.body_local_position[2] = msg.axes[1] * 0.03

        # local body orientation
        state.body_local_orientation[0] = msg.axes[0] * 0.4
        state.body_local_orientation[1] = msg.axes[4] * 0.5
        state.body_local_orientation[2] = msg.axes[3] * 0.4

        if self.use_button:
            if msg.buttons[7]:
                self.use_imu = not self.use_imu
                self.use_button = False
                rospy.loginfo(f"Rest Controller - Use roll/pitch compensation: {self.use_imu}")
                
            elif msg.buttons[4]:  # Use button 5 to toggle LQR
                self.use_lqr = not self.use_lqr
                self.use_button = False
                if self.use_lqr:
                    self.lqr_controller.reset()
                    rospy.loginfo("Rest: Switched to LQR controller")
                else:
                    self.pid_controller.reset()
                    rospy.loginfo("Rest: Switched to PID controller")

        if not self.use_button:
            if not (msg.buttons[4] or msg.buttons[7]):
                self.use_button = True

    @property
    def default_stance(self):
        return self.def_stance

    def step(self, state, command):
        temp = self.default_stance
        temp[2] = [command.robot_height] * 4

        # roll and pitch compensation
        if self.use_imu:
            if self.use_lqr:
                compensation = self.lqr_controller.run(state.imu_roll, state.imu_pitch)
            else:
                compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
                
            roll_compensation = -compensation[0]
            pitch_compensation = -compensation[1]

            rot = rotxyz(roll_compensation, pitch_compensation, 0)
            temp = np.matmul(rot, temp)

        return temp

    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        return state.foot_locations