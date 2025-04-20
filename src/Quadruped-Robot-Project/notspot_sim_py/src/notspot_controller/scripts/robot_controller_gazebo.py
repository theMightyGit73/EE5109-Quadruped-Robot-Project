#!/usr/bin/env python3
#Author: lnotspotl
#Modified with enhanced debugging, logging, odometry, and PID tuning support

# Add script directory to Python path
import sys
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

import rospy
import time
import numpy as np
import psutil
import threading
import pandas as pd  
import matplotlib.pyplot as plt  
import tf2_ros
from datetime import datetime

from sensor_msgs.msg import Joy, Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from RobotController import RobotController
from InverseKinematics import robot_IK
from std_msgs.msg import Float64, String, Float64MultiArray

# Configuration
USE_IMU = True
RATE = 60
DEBUG_PRINT_INTERVAL = 30  # Print debug info every X iterations
TARGET_LOOP_TIME = 1.0/RATE  # Store this for later use
LOG_TO_FILE = True  # Set to True to log to file
LOG_FILE_PATH = os.path.expanduser("~/2425-EE5109/MiniProject/catkin_ws/logs/")
PID_ANALYSIS_INTERVAL = 60  # Generate PID analysis every 60 seconds when in PID mode

# System stats tracking
system_stats = {
    'cpu_percent': 0,
    'memory_percent': 0,
    'loop_time_stats': {
        'last_100': []
    }
}

# Initialize ROS node
rospy.init_node("Robot_Controller")
rospy.loginfo("Starting NotSpot Robot Controller with enhanced debugging, odometry, and PID tuning")

# Create log directory if it doesn't exist
if LOG_TO_FILE:
    try:
        if not os.path.exists(LOG_FILE_PATH):
            os.makedirs(LOG_FILE_PATH)
        log_file_name = f"{LOG_FILE_PATH}notspot_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        log_file = open(log_file_name, 'w')
        log_file.write("timestamp,controller,dx,dy,dz,roll,pitch,yaw,imu_roll,imu_pitch,roll_error,pitch_error,")
        log_file.write("fr_height,fl_height,rr_height,rl_height,loop_time,cpu_percent,memory_percent,")
        log_file.write("odom_x,odom_y,odom_yaw,vel_x,vel_y,yaw_rate\n")  # Added odometry data to log
        rospy.loginfo(f"Logging data to {log_file_name}")
    except Exception as e:
        rospy.logwarn(f"Failed to create log file: {str(e)}")
        LOG_TO_FILE = False

# Robot geometry
body = [0.1908, 0.080]
legs = [0.0, 0.04, 0.100, 0.094333] 

# Initialize robot and inverse kinematics
notspot_robot = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics = robot_IK.InverseKinematics(body, legs)

# Setup debug publisher
debug_pub = rospy.Publisher("/notspot_debug", String, queue_size=10)

# Setup performance monitoring publisher
perf_pub = rospy.Publisher("/notspot_controller/performance", Float64MultiArray, queue_size=10)

# Add odometry publisher and tf broadcaster
odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
tf_broadcaster = tf2_ros.TransformBroadcaster()

# Initialize odometry message
odom = Odometry()
odom.header.frame_id = "odom"
odom.child_frame_id = "base_link"

# Odometry estimation variables
x_pos = 0.0
y_pos = 0.0
yaw = 0.0
last_time = rospy.Time.now()
prev_foot_positions = None

# Additional position tracking in the map frame
map_x = 0.0
map_y = 0.0
map_yaw = 0.0

last_pid_analysis_time = rospy.Time.now()

# Setup joint command publishers
command_topics = ["/notspot_controller/FR1_joint/command",
                  "/notspot_controller/FR2_joint/command",
                  "/notspot_controller/FR3_joint/command",
                  "/notspot_controller/FL1_joint/command",
                  "/notspot_controller/FL2_joint/command",
                  "/notspot_controller/FL3_joint/command",
                  "/notspot_controller/RR1_joint/command",
                  "/notspot_controller/RR2_joint/command",
                  "/notspot_controller/RR3_joint/command",
                  "/notspot_controller/RL1_joint/command",
                  "/notspot_controller/RL2_joint/command",
                  "/notspot_controller/RL3_joint/command"]

publishers = []
for i in range(len(command_topics)):
    publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size=0))

# Set up subscribers
if USE_IMU:
    rospy.loginfo("Subscribing to IMU data on topic: notspot_imu/base_link_orientation")
    rospy.Subscriber("notspot_imu/base_link_orientation", Imu, notspot_robot.imu_orientation)
else:
    rospy.logwarn("IMU is disabled - robot will not use orientation feedback")

rospy.Subscriber("notspot_joy/joy_ramped", Joy, notspot_robot.joystick_command)

# Add PID tuning subscriber
rospy.Subscriber("/notspot_controller/pid_tuning", Float64MultiArray, notspot_robot.handle_pid_tuning)
rospy.loginfo("Subscribed to PID tuning commands")

# Global joint state data
joint_states = None
def joint_state_callback(msg):
    global joint_states
    joint_states = msg
rospy.Subscriber("/joint_states", JointState, joint_state_callback)

# Setup control loop rate
rate = rospy.Rate(RATE)
rospy.loginfo(f"Control loop will run at {RATE}Hz")

# Initialize debug variables
debug_loop_counter = 0
start_time = time.time()
total_iterations = 0
min_loop_time = float('inf')
max_loop_time = 0
total_loop_time = 0

# Function to convert Euler angles to quaternion
def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

# Function to publish debug data
def publish_debug_info(info_str):
    msg = String()
    msg.data = info_str
    debug_pub.publish(msg)

# Function to publish performance data
def publish_performance_data(commanded_angles, actual_angles=None, control_effort=None):
    msg = Float64MultiArray()
    data = list(commanded_angles)
    
    if actual_angles is not None:
        data += list(actual_angles)
    
    if control_effort is not None:
        data += list(control_effort)
    
    msg.data = data
    perf_pub.publish(msg)

# Function to monitor system resources in a separate thread
def monitor_system_resources():
    process = psutil.Process(os.getpid())
    while not rospy.is_shutdown():
        try:
            system_stats['cpu_percent'] = process.cpu_percent(interval=1.0)
            system_stats['memory_percent'] = process.memory_percent()
        except Exception as e:
            rospy.logwarn(f"Error monitoring system resources: {str(e)}")
        
        # Sleep to avoid excessive CPU usage
        time.sleep(5)  # Update every 5 seconds

# Start resource monitoring thread
resource_thread = threading.Thread(target=monitor_system_resources)
resource_thread.daemon = True
resource_thread.start()

# Function to create a visual state representation
def publish_state_visualization(state, command, odom_x=0, odom_y=0, odom_yaw=0):
    # Create a more visual state representation
    state_str = [
        "╔══════════════ NOTSPOT STATE ══════════════╗",
        f"║ Mode: {state.behavior_state.name:<36} ║",
        f"║ Controller: {notspot_robot.currentController.__class__.__name__:<30} ║",
        "╠══════════ BODY POSITION/ORIENTATION ══════╣",
        f"║ Position: x={state.body_local_position[0]:+7.4f}, y={state.body_local_position[1]:+7.4f}, z={state.body_local_position[2]:+7.4f} ║",
        f"║ Orientation: r={state.body_local_orientation[0]:+7.4f}, p={state.body_local_orientation[1]:+7.4f}, y={state.body_local_orientation[2]:+7.4f} ║"
    ]
    
    # Add odometry data
    state_str.append("╠═════════════════ ODOMETRY ═════════════════╣")
    state_str.append(f"║ Position: x={odom_x:+7.4f}, y={odom_y:+7.4f}, yaw={odom_yaw:+7.4f}        ║")
    
    # Add IMU data if available
    if hasattr(state, 'imu_roll') and hasattr(state, 'imu_pitch'):
        state_str.append("╠═════════════════ IMU DATA ═════════════════╣")
        state_str.append(f"║ IMU Angles: r={state.imu_roll:+7.4f}, p={state.imu_pitch:+7.4f}             ║")
        
        if hasattr(notspot_robot, 'currentController') and hasattr(notspot_robot.currentController, 'use_imu'):
            imu_status = "ENABLED" if notspot_robot.currentController.use_imu else "DISABLED"
            controller = "LQR" if (hasattr(notspot_robot.currentController, 'use_lqr') and 
                                 notspot_robot.currentController.use_lqr) else "PID"
            state_str.append(f"║ Stabilization: {imu_status}, Controller: {controller}       ║")
    
    # Command data
    state_str.append("╠═════════════════ COMMANDS ═════════════════╣")
    state_str.append(f"║ Velocity: x={command.velocity[0]:+7.4f}, y={command.velocity[1]:+7.4f}, yaw={command.yaw_rate:+7.4f} ║")
    
    # Add foot position data
    if hasattr(state, 'foot_locations'):
        state_str.append("╠════════════════ FOOT STATUS ════════════════╣")
        labels = ["FR", "FL", "RR", "RL"]
        for i in range(4):
            x, y, z = state.foot_locations[0, i], state.foot_locations[1, i], state.foot_locations[2, i]
            state_str.append(f"║ {labels[i]}: x={x:+7.4f}, y={y:+7.4f}, z={z:+7.4f}          ║")
    
    # Add controller information
    pid_info = ""
    lqr_info = ""
    
    if hasattr(notspot_robot.currentController, 'pid_controller'):
        pid = notspot_robot.currentController.pid_controller
        pid_info = f"║ PID: kp={pid.kp:.4f}, ki={pid.ki:.4f}, kd={pid.kd:.4f}        ║"
    
    if hasattr(notspot_robot.currentController, 'lqr_controller'):
        lqr = notspot_robot.currentController.lqr_controller
        lqr_info = f"║ LQR: q_angle={lqr.q_angle:.4f}, q_rate={lqr.q_rate:.4f}, r_input={lqr.r_input:.4f} ║"
    
    if pid_info or lqr_info:
        state_str.append("╠════════════ CONTROLLER PARAMETERS ════════════╣")
        if pid_info:
            state_str.append(pid_info)
        if lqr_info:
            state_str.append(lqr_info)
    
    # System stats
    state_str.append("╠═════════════════ SYSTEM STATS ══════════════╣")
    avg_loop_time = np.mean(system_stats['loop_time_stats']['last_100']) if system_stats['loop_time_stats']['last_100'] else 0
    max_loop_time = max(system_stats['loop_time_stats']['last_100']) if system_stats['loop_time_stats']['last_100'] else 0
    state_str.append(f"║ CPU: {system_stats['cpu_percent']:5.1f}%, Loop: {avg_loop_time*1000:5.2f}ms (max: {max_loop_time*1000:5.2f}ms) ║")
    
    state_str.append("╚═══════════════════════════════════════════════╝")
    
    viz_msg = String()
    viz_msg.data = "\n".join(state_str)
    debug_pub.publish(viz_msg)

# Function to log data to CSV file
def log_data_to_file(timestamp, controller, dx, dy, dz, roll, pitch, yaw, 
                    imu_roll, imu_pitch, roll_error, pitch_error,
                    fr_height, fl_height, rr_height, rl_height, loop_time,
                    odom_x=0, odom_y=0, odom_yaw=0, vel_x=0, vel_y=0, yaw_rate=0):
    if not LOG_TO_FILE:
        return
    
    try:
        log_file.write(f"{timestamp},{controller},{dx},{dy},{dz},{roll},{pitch},{yaw},")
        log_file.write(f"{imu_roll},{imu_pitch},{roll_error},{pitch_error},")
        log_file.write(f"{fr_height},{fl_height},{rr_height},{rl_height},{loop_time},")
        log_file.write(f"{system_stats['cpu_percent']},{system_stats['memory_percent']},")
        log_file.write(f"{odom_x},{odom_y},{odom_yaw},{vel_x},{vel_y},{yaw_rate}\n")
        log_file.flush()  # Ensure data is written immediately
    except Exception as e:
        rospy.logwarn(f"Failed to write to log file: {str(e)}")

# Cleanup to save memory
del command_topics
# Don't delete RATE since we need it later
del USE_IMU

# Main control loop
rospy.loginfo("Entering main control loop")
while not rospy.is_shutdown():
    loop_start_time = time.time()
    
    # Run robot controller
    leg_positions = notspot_robot.run()
    notspot_robot.change_controller()

    # Get robot state
    dx = notspot_robot.state.body_local_position[0]
    dy = notspot_robot.state.body_local_position[1]
    dz = notspot_robot.state.body_local_position[2]
    
    roll = notspot_robot.state.body_local_orientation[0]
    pitch = notspot_robot.state.body_local_orientation[1]
    yaw_local = notspot_robot.state.body_local_orientation[2]
    
    # Odometry update
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()
    last_time = current_time
    
    # Estimate velocity from command inputs
    vel_x = 0.0
    vel_y = 0.0
    yaw_rate = 0.0
    
    # Get velocity from controller if available
    if hasattr(notspot_robot.command, 'horizontal_velocity') and hasattr(notspot_robot.command, 'vertical_velocity'):
        # Use velocity directly from command
        # Access max velocities from the trot controller
        if hasattr(notspot_robot, 'trotGaitController'):
            max_x_velocity = notspot_robot.trotGaitController.max_x_velocity
            max_y_velocity = notspot_robot.trotGaitController.max_y_velocity
            vel_x = notspot_robot.command.horizontal_velocity * max_x_velocity  # Scale by max velocity
            vel_y = notspot_robot.command.vertical_velocity * max_y_velocity    # Scale by max velocity
        else:
            # Fallback if trotGaitController is not available
            vel_x = notspot_robot.command.horizontal_velocity
            vel_y = notspot_robot.command.vertical_velocity
            
        yaw_rate = notspot_robot.command.yaw_rate

        # Update map position based on these velocities
        map_x += (vel_x * np.cos(map_yaw) - vel_y * np.sin(map_yaw)) * dt
        map_y += (vel_x * np.sin(map_yaw) + vel_y * np.cos(map_yaw)) * dt
        map_yaw += yaw_rate * dt
    else:
        # Fallback to estimating velocity from foot positions change
        if prev_foot_positions is not None and hasattr(notspot_robot.state, 'behavior_state') and notspot_robot.state.behavior_state == 2:  # Check if in trot mode
            # Calculate average foot movement
            delta_pos = np.mean(leg_positions - prev_foot_positions, axis=1)
            vel_x = delta_pos[0] / dt
            vel_y = delta_pos[1] / dt
        else:
            vel_x = 0.0
            vel_y = 0.0
    
    # Estimate yaw rate from command
    if hasattr(notspot_robot.command, 'yaw_rate'):
        yaw_rate = notspot_robot.command.yaw_rate
    else:
        # Could improve this with IMU data if available
        yaw_rate = 0.0
    
    # Integrate velocity to get position
    # Convert to world frame using the current yaw angle
    x_pos += (vel_x * np.cos(yaw) - vel_y * np.sin(yaw)) * dt
    y_pos += (vel_x * np.sin(yaw) + vel_y * np.cos(yaw)) * dt
    yaw += yaw_rate * dt
    
    # Save foot positions for next iteration
    prev_foot_positions = leg_positions.copy() if leg_positions is not None else None
    
    # Use IMU for orientation if available, otherwise use integrated yaw
    imu_roll = 0
    imu_pitch = 0
    imu_roll_error = 0
    imu_pitch_error = 0
    
    if hasattr(notspot_robot.state, 'imu_roll') and hasattr(notspot_robot.state, 'imu_pitch'):
        imu_roll = notspot_robot.state.imu_roll
        imu_pitch = notspot_robot.state.imu_pitch
        
        # Calculate difference between commanded and measured orientation
        imu_roll_error = roll - imu_roll
        imu_pitch_error = pitch - imu_pitch
        
        # Use IMU values for odometry orientation
        odom_roll = imu_roll
        odom_pitch = imu_pitch
        # Note: We don't have IMU yaw, so we use the integrated yaw
        odom_yaw = yaw
    else:
        odom_roll = roll
        odom_pitch = pitch
        odom_yaw = yaw
    
    # Get quaternion from roll, pitch, yaw
    quaternion = get_quaternion_from_euler(odom_roll, odom_pitch, odom_yaw)
    
    # Update odometry message
    odom.header.stamp = current_time
    odom.pose.pose.position.x = x_pos
    odom.pose.pose.position.y = y_pos
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation.x = quaternion[0]
    odom.pose.pose.orientation.y = quaternion[1]
    odom.pose.pose.orientation.z = quaternion[2]
    odom.pose.pose.orientation.w = quaternion[3]
    odom.twist.twist.linear.x = vel_x
    odom.twist.twist.linear.y = vel_y
    odom.twist.twist.angular.z = yaw_rate
    
    # Publish odometry
    odom_pub.publish(odom)
    
    # Publish transform
    transform = TransformStamped()
    transform.header.stamp = current_time
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_link"
    transform.transform.translation.x = x_pos
    transform.transform.translation.y = y_pos
    transform.transform.translation.z = 0.0
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    tf_broadcaster.sendTransform(transform)
    
    # Debug prints
    debug_loop_counter += 1
    if debug_loop_counter % DEBUG_PRINT_INTERVAL == 0:
        controller_type = notspot_robot.currentController.__class__.__name__
        controller_type_str = f"{controller_type}"
        if hasattr(notspot_robot.currentController, 'use_lqr'):
            controller_type_str += f" ({'LQR' if notspot_robot.currentController.use_lqr else 'PID'})"
        
        rospy.loginfo(f"Controller: {controller_type_str}")
        rospy.loginfo(f"Body position: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}")
        rospy.loginfo(f"Body orientation: roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw_local:.4f}")
        
        # Log IMU data if available
        if hasattr(notspot_robot.state, 'imu_roll') and hasattr(notspot_robot.state, 'imu_pitch'):
            rospy.loginfo(f"IMU orientation: roll={imu_roll:.4f}, pitch={imu_pitch:.4f}")
            rospy.loginfo(f"Orientation error: roll_error={imu_roll_error:.4f}, pitch_error={imu_pitch_error:.4f}")
        
        # Log odometry data
        rospy.loginfo(f"Odometry: x={x_pos:.4f}, y={y_pos:.4f}, yaw={yaw:.4f}")
        rospy.loginfo(f"Velocity: vx={vel_x:.4f}, vy={vel_y:.4f}, yaw_rate={yaw_rate:.4f}")
        
        # Debug output for foot positions
        foot_heights = leg_positions[2, :]
        rospy.loginfo(f"Foot heights: FR={foot_heights[0]:.4f}, FL={foot_heights[1]:.4f}, RR={foot_heights[2]:.4f}, RL={foot_heights[3]:.4f}")
        
        # Add controller information
        if hasattr(notspot_robot.currentController, 'pid_controller'):
            pid = notspot_robot.currentController.pid_controller
            rospy.loginfo(f"PID: kp={pid.kp:.4f}, ki={pid.ki:.4f}, kd={pid.kd:.4f}")
        
        if hasattr(notspot_robot.currentController, 'lqr_controller'):
            lqr = notspot_robot.currentController.lqr_controller
            rospy.loginfo(f"LQR: q_angle={lqr.q_angle:.4f}, q_rate={lqr.q_rate:.4f}, r_input={lqr.r_input:.4f}")
        
        # Log to file if enabled
        current_timestamp = time.time() - start_time
        log_data_to_file(
            current_timestamp, controller_type_str, 
            dx, dy, dz, roll, pitch, yaw_local,
            imu_roll, imu_pitch, imu_roll_error, imu_pitch_error,
            foot_heights[0], foot_heights[1], foot_heights[2], foot_heights[3],
            loop_end_time - loop_start_time if 'loop_end_time' in locals() else 0,
            x_pos, y_pos, yaw, vel_x, vel_y, yaw_rate  # Added odometry data
        )
        
        # Publish comprehensive debug info
        debug_str = (
            f"Time: {time.time() - start_time:.2f}s\n"
            f"Controller: {controller_type_str}\n"
            f"Body: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}\n"
            f"Angles: r={roll:.4f}, p={pitch:.4f}, y={yaw_local:.4f}\n"
        )
        
        if hasattr(notspot_robot.state, 'imu_roll') and hasattr(notspot_robot.state, 'imu_pitch'):
            debug_str += f"IMU: r={notspot_robot.state.imu_roll:.4f}, p={notspot_robot.state.imu_pitch:.4f}\n"
            debug_str += f"Error: r_err={imu_roll_error:.4f}, p_err={imu_pitch_error:.4f}\n"
        
        debug_str += f"Odom: x={x_pos:.4f}, y={y_pos:.4f}, yaw={yaw:.4f}\n"
        debug_str += f"Vel: vx={vel_x:.4f}, vy={vel_y:.4f}, yaw_rate={yaw_rate:.4f}\n"
        
        debug_str += f"Foot heights: FR={foot_heights[0]:.4f}, FL={foot_heights[1]:.4f}, "
        debug_str += f"RR={foot_heights[2]:.4f}, RL={foot_heights[3]:.4f}\n"
        
        # System stats
        if system_stats['loop_time_stats']['last_100']:
            avg_loop = np.mean(system_stats['loop_time_stats']['last_100']) * 1000  # Convert to ms
            debug_str += f"System: CPU={system_stats['cpu_percent']:.1f}%, Mem={system_stats['memory_percent']:.1f}%, "
            debug_str += f"Loop={avg_loop:.2f}ms\n"
        
        publish_debug_info(debug_str)
    
    # Publish visual state representation every X iterations
    if debug_loop_counter % (DEBUG_PRINT_INTERVAL * 2) == 0:
        publish_state_visualization(notspot_robot.state, notspot_robot.command, x_pos, y_pos, yaw)
    
    # Auto-generate PID analysis when in PID mode (if enough time has passed)
    if ((current_time - last_pid_analysis_time).to_sec() > PID_ANALYSIS_INTERVAL and 
            hasattr(notspot_robot.currentController, 'use_lqr') and 
            not notspot_robot.currentController.use_lqr):  # Only when in PID mode
        
        # Try to generate PID analysis
        if hasattr(notspot_robot, 'auto_generate_pid_analysis'):
            success = notspot_robot.auto_generate_pid_analysis()
            if success:
                last_pid_analysis_time = current_time
                rospy.loginfo("Auto-generated PID analysis graphs")

    # Compute inverse kinematics
    try:
        joint_angles = inverseKinematics.inverse_kinematics(leg_positions,
                              dx, dy, dz, roll, pitch, yaw_local)

        # Get actual joint positions if available
        actual_joint_positions = None
        if joint_states is not None:
            actual_joint_positions = list(joint_states.position)
            
            # Calculate tracking error metrics if data is available
            if debug_loop_counter % (DEBUG_PRINT_INTERVAL * 5) == 0 and len(actual_joint_positions) == len(joint_angles):
                errors = [abs(joint_angles[i] - actual_joint_positions[i]) for i in range(len(joint_angles))]
                max_error = max(errors)
                avg_error = sum(errors) / len(errors)
                rospy.loginfo(f"Joint tracking - Max error: {max_error:.4f} rad, Avg error: {avg_error:.4f} rad")
        
        # Publish performance data
        publish_performance_data(joint_angles, actual_joint_positions)

        # Publish joint commands
        for i in range(len(joint_angles)):
            publishers[i].publish(joint_angles[i])
            
        # Debug joint angles occasionally
        if debug_loop_counter % (DEBUG_PRINT_INTERVAL * 10) == 0:  # Less frequent
            rospy.loginfo(f"Joint angles sample: {joint_angles[0:3]}")
            
    except Exception as e:
        rospy.logwarn(f"Inverse kinematics error: {str(e)}")
        pass
    
    # Calculate loop statistics
    loop_end_time = time.time()
    loop_duration = loop_end_time - loop_start_time
    
    # Update loop time stats
    total_iterations += 1
    min_loop_time = min(min_loop_time, loop_duration)
    max_loop_time = max(max_loop_time, loop_duration)
    total_loop_time += loop_duration
    
    # Store loop time in rolling window
    system_stats['loop_time_stats']['last_100'].append(loop_duration)
    if len(system_stats['loop_time_stats']['last_100']) > 100:
        system_stats['loop_time_stats']['last_100'].pop(0)
    
    # Print loop timing statistics periodically
    if debug_loop_counter % (DEBUG_PRINT_INTERVAL * 5) == 0:
        avg_loop_time = total_loop_time / total_iterations
        rospy.loginfo(f"Control loop timing - Current: {loop_duration*1000:.2f}ms, Avg: {avg_loop_time*1000:.2f}ms, Min: {min_loop_time*1000:.2f}ms, Max: {max_loop_time*1000:.2f}ms")
        
        # Check if we're meeting timing requirements
        if loop_duration > TARGET_LOOP_TIME:
            rospy.logwarn(f"Control loop took {loop_duration*1000:.2f}ms (target: {TARGET_LOOP_TIME*1000:.2f}ms) - May not be meeting real-time requirements")
    
    # Maintain loop rate
    rate.sleep()

# Clean up resources
if LOG_TO_FILE:
    try:
        log_file.close()
        rospy.loginfo(f"Log file closed: {log_file_name}")
    except:
        pass