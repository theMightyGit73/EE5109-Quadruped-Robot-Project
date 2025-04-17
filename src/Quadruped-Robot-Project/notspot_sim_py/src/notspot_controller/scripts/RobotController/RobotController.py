#!/usr/bin/evn python3
#Author: lnotspotl
#Enhanced with improved logging, diagnostics, and controller comparison

import numpy as np
import tf # Make sure this is tf and not tf2_ros here if used for transformations
import rospy
import time
import collections
import os # Needed for path manipulation and directory creation
import csv # Needed for logging to CSV
import matplotlib.pyplot as plt # Needed for plotting
import pandas as pd # Needed for study graph generation
from datetime import datetime # Needed for timestamps

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

        # Data collection for controller comparison
        self.data_collection_enabled = True
        self.data_dir = os.path.expanduser("~/2425-EE5109/MiniProject/catkin_ws/data")
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
        
        self.performance_log = []
        self.controller_switches = []
        self.test_session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.data_collection_rate = 10  # Hz
        self.last_data_collection_time = rospy.Time.now()
        
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
        
        # Create a CSV file to store the comparison data
        self.init_data_collection()
        
        rospy.loginfo("Robot controller initialized with enhanced diagnostics and data collection")

    def init_data_collection(self):
        """Initialize data collection files for controller comparison"""
        self.performance_file = f"{self.data_dir}/controller_performance_{self.test_session_id}.csv"
        with open(self.performance_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'controller_type', 'controller_mode', 
                'roll', 'pitch', 'roll_rate', 'pitch_rate',
                'roll_setpoint', 'pitch_setpoint', 
                'roll_error', 'pitch_error',
                'control_effort_roll', 'control_effort_pitch',
                'robot_height', 'body_x', 'body_y',
                'fr_height', 'fl_height', 'rr_height', 'rl_height',
                'motion_command_x', 'motion_command_y', 'motion_command_yaw'
            ])
        
        # File for controller transition events
        self.transitions_file = f"{self.data_dir}/controller_transitions_{self.test_session_id}.csv"
        with open(self.transitions_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'from_controller', 'to_controller', 
                'pid_to_lqr', 'duration_in_previous'
            ])
        
        rospy.loginfo(f"Data collection initialized. Files will be saved to {self.data_dir}")

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
        
        # Record PID/LQR transitions specifically
        from_pid_lqr = "unknown"
        to_pid_lqr = "unknown"
        
        # Determine controller types - TrotGait, Rest, and Stand controllers all have LQR/PID modes
        if hasattr(self.currentController, 'use_lqr'):
            from_pid_lqr = "LQR" if self.currentController.use_lqr else "PID"
        
        # Check if the new controller has LQR capability
        if new_controller_name == "TrotGaitController":
            to_pid_lqr = "LQR" if self.trotGaitController.use_lqr else "PID"
        elif new_controller_name == "RestController": 
            to_pid_lqr = "LQR" if self.restController.use_lqr else "PID"
        elif new_controller_name == "StandController":
            to_pid_lqr = "LQR" if hasattr(self.standController, 'use_lqr') and self.standController.use_lqr else "PID"
        
        # Record the transition in CSV
        with open(self.transitions_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([
                now.to_sec(),
                self.controller_history['transitions'][-1]['from'], 
                new_controller_name,
                f"{from_pid_lqr}->{to_pid_lqr}",
                duration
            ])
        
        # Log controller usage stats periodically
        total_time = sum(self.controller_history["active_durations"].values())
        if total_time > 0:
            usage_percents = {ctrl: (time/total_time)*100 for ctrl, time in self.controller_history["active_durations"].items()}
            rospy.loginfo(f"Controller usage: " + 
                         f"Trot: {usage_percents['TrotGaitController']:.1f}%, " +
                         f"Crawl: {usage_percents['CrawlGaitController']:.1f}%, " +
                         f"Stand: {usage_percents['StandController']:.1f}%, " +
                         f"Rest: {usage_percents['RestController']:.1f}%")

    def collect_performance_data(self):
        """Collect performance data for controller comparison"""
        if not self.data_collection_enabled:
            return
            
        # Check if we should collect data based on rate
        current_time = rospy.Time.now()
        time_since_last = (current_time - self.last_data_collection_time).to_sec()
        if time_since_last < (1.0 / self.data_collection_rate):
            return
            
        self.last_data_collection_time = current_time
        
        # Determine controller type
        controller_type = self.controller_history["current_controller"]
        controller_mode = "unknown"
        
        # Determine controller mode (PID/LQR)
        if hasattr(self.currentController, 'use_lqr'):
            controller_mode = "LQR" if self.currentController.use_lqr else "PID"
        
        # Extract state data
        roll = self.state.imu_roll if hasattr(self.state, 'imu_roll') else 0.0
        pitch = self.state.imu_pitch if hasattr(self.state, 'imu_pitch') else 0.0
        
        # Calculate roll and pitch rates (if available)
        roll_rate = 0.0
        pitch_rate = 0.0
        if hasattr(self, 'last_roll') and hasattr(self, 'last_pitch') and hasattr(self, 'last_imu_time'):
            dt = (current_time - self.last_imu_time).to_sec()
            if dt > 0.001:  # Avoid division by zero
                roll_rate = (roll - self.last_roll) / dt
                pitch_rate = (pitch - self.last_pitch) / dt
        
        # Save current values for next calculation
        self.last_roll = roll
        self.last_pitch = pitch
        self.last_imu_time = current_time
        
        # Get setpoints and errors
        roll_setpoint = self.state.body_local_orientation[0] if hasattr(self.state, 'body_local_orientation') else 0.0
        pitch_setpoint = self.state.body_local_orientation[1] if hasattr(self.state, 'body_local_orientation') else 0.0
        
        roll_error = roll_setpoint - roll
        pitch_error = pitch_setpoint - pitch
        
        # Get control efforts
        control_effort_roll = 0.0
        control_effort_pitch = 0.0
        if hasattr(self.currentController, 'pid_controller') and hasattr(self.currentController.pid_controller, 'last_error'):
            control_effort_roll = self.currentController.pid_controller.last_error[0]
            control_effort_pitch = self.currentController.pid_controller.last_error[1]
        elif hasattr(self.currentController, 'lqr_controller'):
            # LQR control effort would be here if accessible
            # This would require modification to LQRController.py to expose these values
            pass
        
        # Get foot heights
        foot_heights = [0.0, 0.0, 0.0, 0.0]
        if hasattr(self.state, 'foot_locations') and self.state.foot_locations.shape == (3, 4):
            foot_heights = self.state.foot_locations[2, :].tolist()
        
        # Get motion commands
        motion_x = 0.0
        motion_y = 0.0
        motion_yaw = 0.0
        if hasattr(self.command, 'velocity'):
            if len(self.command.velocity) >= 1:
                motion_x = self.command.velocity[0]
            if len(self.command.velocity) >= 2:
                motion_y = self.command.velocity[1]
        if hasattr(self.command, 'yaw_rate'):
            motion_yaw = self.command.yaw_rate
        
        # Record data to CSV
        with open(self.performance_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([
                current_time.to_sec(),
                controller_type,
                controller_mode,
                roll,
                pitch,
                roll_rate,
                pitch_rate,
                roll_setpoint,
                pitch_setpoint,
                roll_error,
                pitch_error,
                control_effort_roll,
                control_effort_pitch,
                self.command.robot_height,
                self.state.body_local_position[0] if hasattr(self.state, 'body_local_position') else 0.0,
                self.state.body_local_position[1] if hasattr(self.state, 'body_local_position') else 0.0,
                foot_heights[0],
                foot_heights[1],
                foot_heights[2],
                foot_heights[3],
                motion_x,
                motion_y,
                motion_yaw
            ])
        
        # Add to in-memory buffer for analysis
        self.performance_log.append({
            'timestamp': current_time.to_sec(),
            'controller_type': controller_type,
            'controller_mode': controller_mode,
            'roll': roll,
            'pitch': pitch,
            'roll_error': roll_error,
            'pitch_error': pitch_error
        })
        
        # Keep the in-memory log limited
        if len(self.performance_log) > 5000:  # Keep last ~10 minutes at 10Hz
            self.performance_log.pop(0)

    def generate_controller_comparison_graphs(self):
        """Generate comparison graphs between PID and LQR controllers"""
        try:
            # Check if we have enough data
            if len(self.performance_log) < 100:
                rospy.logwarn("Not enough data collected for meaningful comparison graphs")
                return
            
            # Filter data by controller mode
            pid_data = [entry for entry in self.performance_log if entry['controller_mode'] == 'PID']
            lqr_data = [entry for entry in self.performance_log if entry['controller_mode'] == 'LQR']
            
            if len(pid_data) < 50 or len(lqr_data) < 50:
                rospy.logwarn(f"Need more data for both controllers. PID: {len(pid_data)}, LQR: {len(lqr_data)}")
                return
            
            # Create figure with multiple subplots
            fig, axes = plt.subplots(2, 2, figsize=(12, 8))
            fig.suptitle('PID vs LQR Controller Performance Comparison', fontsize=16)
            
            # Prepare data for plotting
            pid_timestamps = [entry['timestamp'] for entry in pid_data]
            pid_roll_errors = [entry['roll_error'] for entry in pid_data]
            pid_pitch_errors = [entry['pitch_error'] for entry in pid_data]
            
            lqr_timestamps = [entry['timestamp'] for entry in lqr_data]
            lqr_roll_errors = [entry['roll_error'] for entry in lqr_data]
            lqr_pitch_errors = [entry['pitch_error'] for entry in lqr_data]
            
            # Normalize timestamps to start from 0
            if pid_timestamps:
                min_pid_time = min(pid_timestamps)
                pid_timestamps = [t - min_pid_time for t in pid_timestamps]
            
            if lqr_timestamps:
                min_lqr_time = min(lqr_timestamps)
                lqr_timestamps = [t - min_lqr_time for t in lqr_timestamps]
            
            # Plot 1: Roll Error Comparison
            axes[0, 0].plot(pid_timestamps, pid_roll_errors, 'r-', alpha=0.7, label='PID')
            axes[0, 0].plot(lqr_timestamps, lqr_roll_errors, 'b-', alpha=0.7, label='LQR')
            axes[0, 0].set_title('Roll Error Comparison')
            axes[0, 0].set_xlabel('Time (s)')
            axes[0, 0].set_ylabel('Error (rad)')
            axes[0, 0].legend()
            axes[0, 0].grid(True)
            
            # Plot 2: Pitch Error Comparison
            axes[0, 1].plot(pid_timestamps, pid_pitch_errors, 'r-', alpha=0.7, label='PID')
            axes[0, 1].plot(lqr_timestamps, lqr_pitch_errors, 'b-', alpha=0.7, label='LQR')
            axes[0, 1].set_title('Pitch Error Comparison')
            axes[0, 1].set_xlabel('Time (s)')
            axes[0, 1].set_ylabel('Error (rad)')
            axes[0, 1].legend()
            axes[0, 1].grid(True)
            
            # Plot 3: Error Histograms
            # Roll errors
            axes[1, 0].hist(pid_roll_errors, bins=30, alpha=0.5, color='r', label='PID', density=True)
            axes[1, 0].hist(lqr_roll_errors, bins=30, alpha=0.5, color='b', label='LQR', density=True)
            axes[1, 0].set_title('Roll Error Distribution')
            axes[1, 0].set_xlabel('Error (rad)')
            axes[1, 0].set_ylabel('Density')
            axes[1, 0].legend()
            axes[1, 0].grid(True)
            
            # Plot 4: Error Statistics Comparison
            # Calculate statistics
            pid_roll_std = np.std(pid_roll_errors)
            pid_pitch_std = np.std(pid_pitch_errors)
            pid_roll_mean = np.mean(np.abs(pid_roll_errors))
            pid_pitch_mean = np.mean(np.abs(pid_pitch_errors))
            
            lqr_roll_std = np.std(lqr_roll_errors)
            lqr_pitch_std = np.std(lqr_pitch_errors)
            lqr_roll_mean = np.mean(np.abs(lqr_roll_errors))
            lqr_pitch_mean = np.mean(np.abs(lqr_pitch_errors))
            
            # Bar plot for standard deviations
            bar_width = 0.3
            index = np.array([0, 1])
            
            axes[1, 1].bar(index, [pid_roll_std, pid_pitch_std], bar_width, 
                         alpha=0.7, color='r', label='PID')
            axes[1, 1].bar(index + bar_width, [lqr_roll_std, lqr_pitch_std], 
                         bar_width, alpha=0.7, color='b', label='LQR')
            
            axes[1, 1].set_title('Error Standard Deviation (lower is better)')
            axes[1, 1].set_ylabel('Standard Deviation (rad)')
            axes[1, 1].set_xticks(index + bar_width / 2)
            axes[1, 1].set_xticklabels(['Roll', 'Pitch'])
            axes[1, 1].legend()
            axes[1, 1].grid(True)
            
            # Add a text box with detailed statistics
            fig.text(0.5, 0.01, 
                    f"PID - Roll: mean={pid_roll_mean:.4f}, std={pid_roll_std:.4f} | Pitch: mean={pid_pitch_mean:.4f}, std={pid_pitch_std:.4f}\n"
                    f"LQR - Roll: mean={lqr_roll_mean:.4f}, std={lqr_roll_std:.4f} | Pitch: mean={lqr_pitch_mean:.4f}, std={lqr_pitch_std:.4f}",
                    ha='center', fontsize=10, bbox=dict(facecolor='white', alpha=0.5))
            
            # Adjust layout and save figure
            plt.tight_layout(rect=[0, 0.03, 1, 0.95])
            output_file = f"{self.data_dir}/controller_comparison_{self.test_session_id}.png"
            plt.savefig(output_file, dpi=300)
            rospy.loginfo(f"Controller comparison graph saved to {output_file}")
            
            # Save another version with current timestamp for multiple graphs
            current_time = datetime.now().strftime("%H%M%S")
            plt.savefig(f"{self.data_dir}/controller_comparison_{self.test_session_id}_{current_time}.png", dpi=300)
            
            plt.close(fig)
            
        except Exception as e:
            rospy.logerr(f"Error generating controller comparison graphs: {str(e)}")

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
        
        # Check for controller toggle (PID/LQR)
        if msg.buttons[4] and not hasattr(self, 'last_lqr_toggle_time'):
            # Initialize toggle tracking
            self.last_lqr_toggle_time = rospy.Time.now()
            self.last_lqr_toggle_state = False
        
        if msg.buttons[4] and hasattr(self, 'last_lqr_toggle_time'):
            current_time = rospy.Time.now()
            if (current_time - self.last_lqr_toggle_time).to_sec() > 0.5:  # Debounce
                # Toggle LQR on current controller
                if hasattr(self.currentController, 'use_lqr'):
                    self.currentController.use_lqr = not self.currentController.use_lqr
                    controller_name = self.controller_history["current_controller"]
                    controller_mode = "LQR" if self.currentController.use_lqr else "PID"
                    rospy.loginfo(f"Toggled {controller_name} to {controller_mode} mode")
                    
                    # Also toggle on other controllers to maintain consistency
                    if hasattr(self.trotGaitController, 'use_lqr'):
                        self.trotGaitController.use_lqr = self.currentController.use_lqr
                    if hasattr(self.restController, 'use_lqr'):
                        self.restController.use_lqr = self.currentController.use_lqr
                    if hasattr(self.standController, 'use_lqr'):
                        self.standController.use_lqr = self.currentController.use_lqr
                    
                    # Reset controllers
                    if self.currentController.use_lqr:
                        if hasattr(self.currentController, 'lqr_controller'):
                            self.currentController.lqr_controller.reset()
                    else:
                        if hasattr(self.currentController, 'pid_controller'):
                            self.currentController.pid_controller.reset()
                
                self.last_lqr_toggle_time = current_time
                self.last_lqr_toggle_state = not self.last_lqr_toggle_state
      
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
        
        # Update mode from command horizontal and vertical speeds
        if hasattr(msg, 'axes') and len(msg.axes) >= 5:
            self.command.horizontal_velocity = msg.axes[4]  # Forward/backward
            self.command.vertical_velocity = msg.axes[3]    # Left/right
            self.command.yaw_rate = msg.axes[0]            # Rotation
        
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
        
        # Initialize IMU debug counter if it doesn't exist
        if not hasattr(self, 'imu_debug_counter'):
            self.imu_debug_counter = 0
            self.last_imu_time = rospy.Time.now()
        
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
        # Collect performance data
        self.collect_performance_data()
        
        # Run the controller
        foot_positions = self.currentController.run(self.state, self.command)
        
        # Track foot positions
        self.update_foot_position_history(foot_positions)
        
        # Generate comparison graphs periodically
        if hasattr(self, 'last_graph_time'):
            current_time = rospy.Time.now()
            if (current_time - self.last_graph_time).to_sec() > 60.0:  # Every minute
                if rospy.get_param('~enable_controller_graphs', True):  # Parameter to enable/disable
                    self.generate_controller_comparison_graphs()
                self.last_graph_time = current_time
        else:
            self.last_graph_time = rospy.Time.now()
        
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
                'usage': self.controller_history["active_durations"],
                'mode': "LQR" if hasattr(self.currentController, 'use_lqr') and self.currentController.use_lqr else "PID"
            },
            'imu': self.imu_stats,
            'state': {
                'behavior': self.state.behavior_state.name,
                'position': self.state.body_local_position.tolist() if hasattr(self.state, 'body_local_position') else None,
                'orientation': self.state.body_local_orientation.tolist() if hasattr(self.state, 'body_local_orientation') else None,
                'foot_positions': self.state.foot_locations.tolist() if hasattr(self.state, 'foot_locations') else None
            },
            'foot_history': {
                'FR': list(self.foot_position_history["FR"])[-1] if self.foot_position_history["FR"] else None,
                'FL': list(self.foot_position_history["FL"])[-1] if self.foot_position_history["FL"] else None,
                'RR': list(self.foot_position_history["RR"])[-1] if self.foot_position_history["RR"] else None,
                'RL': list(self.foot_position_history["RL"])[-1] if self.foot_position_history["RL"] else None
            },
            'performance': {
                'data_points_collected': len(self.performance_log),
                'last_update': rospy.Time.now().to_sec()
            }
        }
        return info
    
    def run_controller_study(self, configs=None, test_duration=10.0):
        """Run an automated study comparing PID and LQR performance 
        
        Args:
            configs: List of controller parameter dictionaries to test
            test_duration: Duration in seconds to test each configuration
        """
        if configs is None:
            # Default configurations to test
            configs = [
                # PID configurations
                {'mode': 'PID', 'kp': 0.5, 'ki': 0.1, 'kd': 0.05, 'description': 'PID_Conservative'},
                {'mode': 'PID', 'kp': 1.0, 'ki': 0.2, 'kd': 0.1, 'description': 'PID_Moderate'},
                {'mode': 'PID', 'kp': 1.5, 'ki': 0.3, 'kd': 0.15, 'description': 'PID_Aggressive'},
                
                # LQR configurations
                {'mode': 'LQR', 'q_angle': 0.5, 'q_rate': 0.05, 'r_input': 0.01, 'description': 'LQR_Conservative'},
                {'mode': 'LQR', 'q_angle': 1.0, 'q_rate': 0.05, 'r_input': 0.005, 'description': 'LQR_Moderate'},
                {'mode': 'LQR', 'q_angle': 2.0, 'q_rate': 0.1, 'r_input': 0.003, 'description': 'LQR_Aggressive'}
            ]
        
        # Create a results file
        results_file = f"{self.data_dir}/controller_study_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(results_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow([
                'Config', 'Mode', 'Parameters', 'Roll_Error_Mean', 'Roll_Error_Std', 
                'Pitch_Error_Mean', 'Pitch_Error_Std', 'Test_Duration'
            ])
        
        rospy.loginfo("Starting automated controller study...")
        
        for config in configs:
            # Setup controller based on configuration
            rospy.loginfo(f"Testing configuration: {config}")
            
            # Configure controller
            if config['mode'] == 'PID':
                # Configure PID parameters
                if hasattr(self.currentController, 'pid_controller'):
                    self.currentController.use_lqr = False
                    self.currentController.pid_controller.kp = config['kp']
                    self.currentController.pid_controller.ki = config['ki']
                    self.currentController.pid_controller.kd = config['kd']
                    self.currentController.pid_controller.reset()
                    
                    # Also update the other controllers for consistency
                    if hasattr(self.trotGaitController, 'pid_controller'):
                        self.trotGaitController.use_lqr = False
                        self.trotGaitController.pid_controller.kp = config['kp']
                        self.trotGaitController.pid_controller.ki = config['ki']
                        self.trotGaitController.pid_controller.kd = config['kd']
                    
                    if hasattr(self.restController, 'pid_controller'):
                        self.restController.use_lqr = False
                        self.restController.pid_controller.kp = config['kp']
                        self.restController.pid_controller.ki = config['ki']
                        self.restController.pid_controller.kd = config['kd']
                
            elif config['mode'] == 'LQR':
                # Configure LQR parameters
                if hasattr(self.currentController, 'lqr_controller'):
                    self.currentController.use_lqr = True
                    self.currentController.lqr_controller.q_angle = config['q_angle']
                    self.currentController.lqr_controller.q_rate = config['q_rate']
                    self.currentController.lqr_controller.r_input = config['r_input']
                    self.currentController.lqr_controller.reset()
                    
                    # Also update the other controllers for consistency
                    if hasattr(self.trotGaitController, 'lqr_controller'):
                        self.trotGaitController.use_lqr = True
                        self.trotGaitController.lqr_controller.q_angle = config['q_angle']
                        self.trotGaitController.lqr_controller.q_rate = config['q_rate']
                        self.trotGaitController.lqr_controller.r_input = config['r_input']
                    
                    if hasattr(self.restController, 'lqr_controller'):
                        self.restController.use_lqr = True
                        self.restController.lqr_controller.q_angle = config['q_angle']
                        self.restController.lqr_controller.q_rate = config['q_rate']
                        self.restController.lqr_controller.r_input = config['r_input']
            
            # Clear previous performance data
            self.performance_log = []
            
            # Run the test for specified duration
            rospy.loginfo(f"Running test for {test_duration} seconds...")
            start_time = rospy.Time.now()
            
            while (rospy.Time.now() - start_time).to_sec() < test_duration:
                # Just let the regular control loop run
                rospy.sleep(0.1)
            
            # Calculate performance metrics
            if len(self.performance_log) > 0:
                roll_errors = [abs(entry['roll_error']) for entry in self.performance_log]
                pitch_errors = [abs(entry['pitch_error']) for entry in self.performance_log]
                
                roll_error_mean = np.mean(roll_errors)
                roll_error_std = np.std(roll_errors)
                pitch_error_mean = np.mean(pitch_errors)
                pitch_error_std = np.std(pitch_errors)
                
                # Format parameters for CSV
                if config['mode'] == 'PID':
                    params_str = f"kp={config['kp']},ki={config['ki']},kd={config['kd']}"
                else:
                    params_str = f"q_angle={config['q_angle']},q_rate={config['q_rate']},r_input={config['r_input']}"
                
                # Save results
                with open(results_file, 'a') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        config['description'],
                        config['mode'],
                        params_str,
                        roll_error_mean,
                        roll_error_std,
                        pitch_error_mean,
                        pitch_error_std,
                        test_duration
                    ])
                
                rospy.loginfo(f"Results for {config['description']} - Roll Error: {roll_error_mean:.4f}±{roll_error_std:.4f}, Pitch Error: {pitch_error_mean:.4f}±{pitch_error_std:.4f}")
            else:
                rospy.logwarn(f"No performance data collected for {config['description']}")
        
        rospy.loginfo(f"Controller study complete. Results saved to {results_file}")
        
        # Generate summary graph
        self.generate_controller_study_graphs(results_file)
        
        return results_file

    def update_foot_position_history(self, positions):
        """Track foot position history for diagnostics"""
        if not hasattr(self, 'foot_position_history'):
            self.foot_position_history = {
                "FR": collections.deque(maxlen=100),
                "FL": collections.deque(maxlen=100),
                "RR": collections.deque(maxlen=100),
                "RL": collections.deque(maxlen=100)
            }
            
        if positions is None or not hasattr(positions, 'shape') or positions.shape != (3, 4):
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

    def generate_controller_study_graphs(self, results_file):
        """Generate summary graphs for the controller study"""
        try:
            
            df = pd.read_csv(results_file)
            
            # Create figure
            fig, axes = plt.subplots(2, 1, figsize=(10, 12))
            fig.suptitle('Controller Performance Comparison Study', fontsize=16)
            
            # Split by controller mode
            pid_results = df[df['Mode'] == 'PID']
            lqr_results = df[df['Mode'] == 'LQR']
            
            # Plot 1: Roll Error Comparison
            bar_width = 0.35
            index = np.arange(max(len(pid_results), len(lqr_results)))
            
            if not pid_results.empty:
                axes[0].bar(index, pid_results['Roll_Error_Mean'], bar_width, 
                           yerr=pid_results['Roll_Error_Std'], label='PID', color='r', alpha=0.7)
            
            if not lqr_results.empty:
                axes[0].bar(index + bar_width, lqr_results['Roll_Error_Mean'], bar_width,
                           yerr=lqr_results['Roll_Error_Std'], label='LQR', color='b', alpha=0.7)
            
            axes[0].set_title('Roll Error by Controller Configuration')
            axes[0].set_ylabel('Mean Absolute Error (rad)')
            axes[0].set_xticks(index + bar_width / 2)
            
            if not pid_results.empty and not lqr_results.empty:
                # If we have both types, use combined descriptions for labeling
                if len(pid_results) == len(lqr_results):
                    # Assuming the order matches (conservative, moderate, aggressive)
                    axes[0].set_xticklabels(['Conservative', 'Moderate', 'Aggressive'])
                else:
                    # Just use generic labels
                    axes[0].set_xticklabels([f'Config {i+1}' for i in range(max(len(pid_results), len(lqr_results)))])
            elif not pid_results.empty:
                axes[0].set_xticklabels(pid_results['Config'])
            elif not lqr_results.empty:
                axes[0].set_xticklabels(lqr_results['Config'])
            
            axes[0].legend()
            axes[0].grid(True)
            
            # Plot 2: Pitch Error Comparison
            if not pid_results.empty:
                axes[1].bar(index, pid_results['Pitch_Error_Mean'], bar_width, 
                           yerr=pid_results['Pitch_Error_Std'], label='PID', color='r', alpha=0.7)
            
            if not lqr_results.empty:
                axes[1].bar(index + bar_width, lqr_results['Pitch_Error_Mean'], bar_width,
                           yerr=lqr_results['Pitch_Error_Std'], label='LQR', color='b', alpha=0.7)
            
            axes[1].set_title('Pitch Error by Controller Configuration')
            axes[1].set_ylabel('Mean Absolute Error (rad)')
            axes[1].set_xticks(index + bar_width / 2)
            
            if not pid_results.empty and not lqr_results.empty:
                if len(pid_results) == len(lqr_results):
                    axes[1].set_xticklabels(['Conservative', 'Moderate', 'Aggressive'])
                else:
                    axes[1].set_xticklabels([f'Config {i+1}' for i in range(max(len(pid_results), len(lqr_results)))])
            elif not pid_results.empty:
                axes[1].set_xticklabels(pid_results['Config'])
            elif not lqr_results.empty:
                axes[1].set_xticklabels(lqr_results['Config'])
            
            axes[1].legend()
            axes[1].grid(True)
            
            # Add overall comparison
            if not pid_results.empty and not lqr_results.empty:
                pid_avg_roll = pid_results['Roll_Error_Mean'].mean()
                lqr_avg_roll = lqr_results['Roll_Error_Mean'].mean()
                pid_avg_pitch = pid_results['Pitch_Error_Mean'].mean()
                lqr_avg_pitch = lqr_results['Pitch_Error_Mean'].mean()
                
                improvement_roll = (pid_avg_roll - lqr_avg_roll) / pid_avg_roll * 100
                improvement_pitch = (pid_avg_pitch - lqr_avg_pitch) / pid_avg_pitch * 100
                
                overall_text = (
                    f"Overall Comparison:\n"
                    f"Roll Error: PID={pid_avg_roll:.4f}, LQR={lqr_avg_roll:.4f} "
                    f"({improvement_roll:.1f}% {'improvement' if improvement_roll > 0 else 'worse'} with LQR)\n"
                    f"Pitch Error: PID={pid_avg_pitch:.4f}, LQR={lqr_avg_pitch:.4f} "
                    f"({improvement_pitch:.1f}% {'improvement' if improvement_pitch > 0 else 'worse'} with LQR)"
                )
                
                fig.text(0.5, 0.01, overall_text, ha='center', fontsize=12, 
                        bbox=dict(facecolor='white', alpha=0.8))
            
            plt.tight_layout(rect=[0, 0.05, 1, 0.95])
            
            # Save figure
            graph_file = results_file.replace('.csv', '_summary.png')
            plt.savefig(graph_file, dpi=300)
            rospy.loginfo(f"Summary graph saved to {graph_file}")
            
            plt.close(fig)
            
        except Exception as e:
            rospy.logerr(f"Error generating controller study graphs: {str(e)}")