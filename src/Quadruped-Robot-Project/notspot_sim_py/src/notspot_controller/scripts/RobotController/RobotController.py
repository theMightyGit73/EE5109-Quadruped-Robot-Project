#!/usr/bin/evn python3
#Author: lnotspotl
#Enhanced with improved logging and diagnostics

import numpy as np
import tf
import rospy
import time
import collections

from . StateCommand import State, Command, BehaviorState
from . RestController import RestController
from . TrotGaitController import TrotGaitController
from . CrawlGaitController import CrawlGaitController
from . StandController import StandController

class Robot(object):
    def __init__(self, body, legs, imu):
        self.body = body
        self.legs = legs

        self.delta_x = self.body[0] * 0.5
        self.delta_y = self.body[1] * 0.5 + self.legs[1]
        self.x_shift_front = 0.006
        self.x_shift_back = -0.03
        self.default_height = 0.15

        # Enhanced diagnostics
        self.controller_history = {
            "transitions": [],
            "active_durations": {
                "TrotGaitController": 0,
                "CrawlGaitController": 0,
                "StandController": 0,
                "RestController": 0
            },
            "last_transition": rospy.Time.now(),
            "current_controller": "RestController"
        }
        
        # Initialize IMU data collection
        self.imu_data_buffer = collections.deque(maxlen=100)  # Store last 100 readings
        self.imu_stats = {
            "roll_avg": 0,
            "pitch_avg": 0,
            "roll_std": 0,
            "pitch_std": 0,
            "roll_rate_avg": 0,
            "pitch_rate_avg": 0
        }

        # Initialize controllers
        self.trotGaitController = TrotGaitController(self.default_stance,
            stance_time = 0.18, swing_time = 0.24, time_step = 0.02,
            use_imu = imu)

        self.crawlGaitController = CrawlGaitController(self.default_stance,
            stance_time = 0.55, swing_time = 0.45, time_step = 0.02)
            
        self.standController = StandController(self.default_stance)

        self.restController = RestController(self.default_stance)

        self.currentController = self.restController
        self.state = State(self.default_height)
        self.state.foot_locations = self.default_stance
        self.command = Command(self.default_height)
        
        # Diagnostics for foot positions
        self.foot_position_history = {
            "FR": collections.deque(maxlen=100),
            "FL": collections.deque(maxlen=100),
            "RR": collections.deque(maxlen=100),
            "RL": collections.deque(maxlen=100)
        }
        
        rospy.loginfo("Robot controller initialized with enhanced diagnostics")

    def log_controller_change(self, new_controller_name):
        """Log controller transitions and durations"""
        now = rospy.Time.now()
        duration = (now - self.controller_history["last_transition"]).to_sec()
        
        # Log transition
        self.controller_history["transitions"].append({
            "from": self.controller_history["current_controller"],
            "to": new_controller_name,
            "time": now.to_sec(),
            "duration": duration
        })
        
        # Only keep last 10 transitions
        if len(self.controller_history["transitions"]) > 10:
            self.controller_history["transitions"].pop(0)
            
        # Update active durations
        self.controller_history["active_durations"][self.controller_history["current_controller"]] += duration
        
        # Update current controller and timestamp
        self.controller_history["current_controller"] = new_controller_name
        self.controller_history["last_transition"] = now
        
        # Log detailed transition info
        rospy.loginfo(f"Controller transition: {self.controller_history['transitions'][-1]['from']} -> {new_controller_name} (after {duration:.2f}s)")
        
        # Log controller usage stats periodically
        total_time = sum(self.controller_history["active_durations"].values())
        if total_time > 0:
            usage_percents = {ctrl: (time/total_time)*100 for ctrl, time in self.controller_history["active_durations"].items()}
            rospy.loginfo(f"Controller usage: " + 
                         f"Trot: {usage_percents['TrotGaitController']:.1f}%, " +
                         f"Crawl: {usage_percents['CrawlGaitController']:.1f}%, " +
                         f"Stand: {usage_percents['StandController']:.1f}%, " +
                         f"Rest: {usage_percents['RestController']:.1f}%")

    def change_controller(self):
        
        if self.command.trot_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.TROT
                self.currentController = self.trotGaitController
                self.log_controller_change("TrotGaitController")
                
                # Reset the appropriate controller based on current mode
                if hasattr(self.trotGaitController, 'use_lqr') and self.trotGaitController.use_lqr:
                    self.trotGaitController.lqr_controller.reset()
                    rospy.loginfo("Reset LQR controller state on mode change.")
                else:
                    self.trotGaitController.pid_controller.reset()
                    rospy.loginfo("Reset PID controller state on mode change.")
                self.state.ticks = 0
            self.command.trot_event = False

        elif self.command.crawl_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.CRAWL
                self.currentController = self.crawlGaitController
                self.log_controller_change("CrawlGaitController")
                self.currentController.first_cycle = True;
                self.state.ticks = 0
            self.command.crawl_event = False

        elif self.command.stand_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.STAND
                self.currentController = self.standController
                self.log_controller_change("StandController")
            self.command.stand_event = False

        elif self.command.rest_event:
            if self.state.behavior_state != BehaviorState.REST:
                self.log_controller_change("RestController")
                
            self.state.behavior_state = BehaviorState.REST
            self.currentController = self.restController
            
            # Reset the appropriate controller based on current mode
            if hasattr(self.restController, 'use_lqr') and self.restController.use_lqr:
                self.restController.lqr_controller.reset()
                rospy.loginfo("Reset LQR controller state on mode change.")
            else:
                self.restController.pid_controller.reset()
                rospy.loginfo("Reset PID controller state on mode change.")
            self.command.rest_event = False

    def joystick_command(self, msg):
        # Track if there was a change to log
        command_changed = False
        old_command = None
        
        if msg.buttons[0]: # rest
            if not self.command.rest_event:
                old_command = "Previous"
                command_changed = True
            self.command.trot_event = False
            self.command.crawl_event = False
            self.command.stand_event = False
            self.command.rest_event = True
            
        elif msg.buttons[1]: # trot
            if not self.command.trot_event:
                old_command = "Previous"
                command_changed = True
            self.command.trot_event = True
            self.command.crawl_event = False
            self.command.stand_event = False
            self.command.rest_event = False

        elif msg.buttons[2]: # crawl
            if not self.command.crawl_event:
                old_command = "Previous"
                command_changed = True
            self.command.trot_event = False
            self.command.crawl_event = True
            self.command.stand_event = False
            self.command.rest_event = False
       
        elif msg.buttons[3]: # stand
            if not self.command.stand_event:
                old_command = "Previous"
                command_changed = True
            self.command.trot_event = False
            self.command.crawl_event = False
            self.command.stand_event = True
            self.command.rest_event = False
      
        # Log command changes
        if command_changed:
            new_command = "Unknown"
            if self.command.rest_event:
                new_command = "Rest"
            elif self.command.trot_event:
                new_command = "Trot"
            elif self.command.crawl_event:
                new_command = "Crawl"
            elif self.command.stand_event:
                new_command = "Stand"
            
            rospy.loginfo(f"Command changed: {old_command} -> {new_command}")
        
        # 🆕 Add this:
        self.command.horizontal_velocity = msg.axes[1]
        self.command.vertical_velocity = msg.axes[0]
        self.command.yaw_rate = msg.axes[2]

        # Optional: log it for debugging
        import rospy
        rospy.loginfo(f"Velocity command: vx={self.command.horizontal_velocity:.2f}, vy={self.command.vertical_velocity:.2f}, yaw={self.command.yaw_rate:.2f}")
        # end Add this

        # Update state based on command
        self.currentController.updateStateCommand(msg, self.state, self.command)

    def update_imu_statistics(self):
        """Update IMU statistics based on collected data"""
        if len(self.imu_data_buffer) < 2:
            return
        
        # Calculate statistics
        roll_values = [data['roll'] for data in self.imu_data_buffer]
        pitch_values = [data['pitch'] for data in self.imu_data_buffer]
        roll_rates = [data['roll_rate'] for data in self.imu_data_buffer if 'roll_rate' in data]
        pitch_rates = [data['pitch_rate'] for data in self.imu_data_buffer if 'pitch_rate' in data]
        
        # Update statistics
        self.imu_stats["roll_avg"] = np.mean(roll_values)
        self.imu_stats["pitch_avg"] = np.mean(pitch_values)
        self.imu_stats["roll_std"] = np.std(roll_values)
        self.imu_stats["pitch_std"] = np.std(pitch_values)
        
        if roll_rates:
            self.imu_stats["roll_rate_avg"] = np.mean(roll_rates)
        if pitch_rates:
            self.imu_stats["pitch_rate_avg"] = np.mean(pitch_rates)

    def imu_orientation(self, msg):
        q = msg.orientation
        rpy_angles = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        
        # Store previous values for rate calculation
        prev_roll = self.state.imu_roll if hasattr(self.state, 'imu_roll') else 0
        prev_pitch = self.state.imu_pitch if hasattr(self.state, 'imu_pitch') else 0
        
        # Update state
        self.state.imu_roll = rpy_angles[0]
        self.state.imu_pitch = rpy_angles[1]
        
        # Calculate rates for debugging
        if not hasattr(self, 'last_imu_time'):
            self.last_imu_time = rospy.Time.now()
            self.imu_debug_counter = 0
        
        current_time = rospy.Time.now()
        dt = (current_time - self.last_imu_time).to_sec()
        if dt > 0.001:  # Avoid division by zero
            roll_rate = (self.state.imu_roll - prev_roll) / dt
            pitch_rate = (self.state.imu_pitch - prev_pitch) / dt
            
            # Store data for statistics
            self.imu_data_buffer.append({
                'timestamp': current_time.to_sec(),
                'roll': self.state.imu_roll,
                'pitch': self.state.imu_pitch,
                'roll_rate': roll_rate,
                'pitch_rate': pitch_rate,
                'accel_x': msg.linear_acceleration.x if hasattr(msg, 'linear_acceleration') else 0,
                'accel_y': msg.linear_acceleration.y if hasattr(msg, 'linear_acceleration') else 0,
                'accel_z': msg.linear_acceleration.z if hasattr(msg, 'linear_acceleration') else 0
            })
            
            # Update statistics periodically
            if self.imu_debug_counter % 10 == 0:
                self.update_imu_statistics()
            
            # Print debug info less frequently
            if self.imu_debug_counter % 100 == 0:  # Print less frequently
                rospy.loginfo("IMU: dt=%.4f, roll=%.4f, pitch=%.4f, roll_rate=%.4f, pitch_rate=%.4f", 
                            dt, self.state.imu_roll, self.state.imu_pitch, roll_rate, pitch_rate)
                # Also log quaternion values to check for any issues
                rospy.loginfo("IMU: quaternion=[%.4f, %.4f, %.4f, %.4f]", q.x, q.y, q.z, q.w)
                
                # Log statistics
                rospy.loginfo("IMU Stats: roll_avg=%.4f±%.4f, pitch_avg=%.4f±%.4f, roll_rate_avg=%.4f, pitch_rate_avg=%.4f",
                            self.imu_stats["roll_avg"], self.imu_stats["roll_std"], 
                            self.imu_stats["pitch_avg"], self.imu_stats["pitch_std"],
                            self.imu_stats["roll_rate_avg"], self.imu_stats["pitch_rate_avg"])
        
        self.last_imu_time = current_time
        self.imu_debug_counter += 1

    def update_foot_position_history(self, positions):
        """Track foot position history for diagnostics"""
        if positions.shape != (3, 4):
            return
        
        # Store current positions
        labels = ["FR", "FL", "RR", "RL"]
        for i, label in enumerate(labels):
            self.foot_position_history[label].append({
                'x': positions[0, i],
                'y': positions[1, i],
                'z': positions[2, i],
                'timestamp': time.time()
            })

    def run(self):
        foot_positions = self.currentController.run(self.state, self.command)
        
        # Track foot positions
        self.update_foot_position_history(foot_positions)
        
        return foot_positions

    @property
    def default_stance(self):
        # FR, FL, RR, RL
        return np.array([[self.delta_x + self.x_shift_front,self.delta_x + self.x_shift_front,-self.delta_x + self.x_shift_back,-self.delta_x + self.x_shift_back],
                         [-self.delta_y                    ,self.delta_y                     ,-self.delta_y                    , self.delta_y                    ],
                         [0                                ,0                                ,0                                ,0                                ]])
                         
    def get_diagnostic_info(self):
        """Return comprehensive diagnostic information"""
        info = {
            'controller': {
                'current': self.controller_history["current_controller"],
                'transitions': self.controller_history["transitions"],
                'usage': self.controller_history["active_durations"]
            },
            'imu': self.imu_stats,
            'state': {
                'behavior': self.state.behavior_state.name,
                'position': self.state.body_local_position.tolist(),
                'orientation': self.state.body_local_orientation.tolist(),
                'foot_positions': self.state.foot_locations.tolist() if hasattr(self.state, 'foot_locations') else None
            },
            'foot_history': {
                'FR': list(self.foot_position_history["FR"])[-1] if self.foot_position_history["FR"] else None,
                'FL': list(self.foot_position_history["FL"])[-1] if self.foot_position_history["FL"] else None,
                'RR': list(self.foot_position_history["RR"])[-1] if self.foot_position_history["RR"] else None,
                'RL': list(self.foot_position_history["RL"])[-1] if self.foot_position_history["RL"] else None
            }
        }
        return info