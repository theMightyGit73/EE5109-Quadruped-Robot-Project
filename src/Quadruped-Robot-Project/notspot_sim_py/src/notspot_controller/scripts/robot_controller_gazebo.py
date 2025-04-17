#!/usr/bin/env python3
#Author: lnotspotl

import rospy
import math
import tf2_ros
import numpy as np
from sensor_msgs.msg import Joy, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from RobotController import RobotController
from InverseKinematics import robot_IK
from std_msgs.msg import Float64

USE_IMU = True
RATE = 60

rospy.init_node("Robot_Controller")

# Robot geometry
body = [0.1908, 0.080]
legs = [0.0, 0.04, 0.100, 0.094333] 

notspot_robot = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics = robot_IK.InverseKinematics(body, legs)

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
    publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 0))

# Add odometry publisher and tf broadcaster
odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
tf_broadcaster = tf2_ros.TransformBroadcaster()

# Initialize odometry message
odom = Odometry()
odom.header.frame_id = "odom"
odom.child_frame_id = "base_link"

# Set covariance matrices - important for SLAM
# Diagonal values represent uncertainty in x, y, z, roll, pitch, yaw
position_covariance_diagonal = [0.01, 0.01, 0.01, 0.03, 0.03, 0.01]  # Lower values = higher confidence
twist_covariance_diagonal = [0.01, 0.01, 0.01, 0.05, 0.05, 0.01]

# Fill position covariance (6x6 matrix)
for i in range(6):
    odom.pose.covariance[i*6+i] = position_covariance_diagonal[i]
    odom.twist.covariance[i*6+i] = twist_covariance_diagonal[i]

# Odometry estimation variables
x_pos = 0.0
y_pos = 0.0
yaw = 0.0
last_time = rospy.Time.now()
prev_leg_positions = None
prev_time = rospy.Time.now()

# Velocity filtering variables
vel_x_filter = 0.0
vel_y_filter = 0.0
yaw_rate_filter = 0.0
filter_alpha = 0.7  # Filtering coefficient (higher = more responsive)

# Initialize velocity measurement
last_vel_x = 0.0
last_vel_y = 0.0
last_yaw_rate = 0.0

if USE_IMU:
    rospy.Subscriber("notspot_imu/base_link_orientation", Imu, notspot_robot.imu_orientation)
rospy.Subscriber("notspot_joy/joy_ramped", Joy, notspot_robot.joystick_command)

rate = rospy.Rate(RATE)

del body
del legs
del command_topics
del USE_IMU
del RATE

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

# Function to normalize angle to [-pi, pi]
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

# Function to estimate velocity from leg movements (leg odometry)
def estimate_velocity_from_legs(current_leg_positions, prev_leg_positions, dt):
    if prev_leg_positions is None or dt <= 0:
        return 0.0, 0.0
        
    # Calculate delta position for each leg that's in stance phase
    # (This is a simplified approach - a more sophisticated approach would track
    # which legs are in stance vs. swing phase)
    delta_positions = []
    
    # Calculate average movement of all legs
    for i in range(len(current_leg_positions)):
        # Calculate delta for this leg
        delta_x = current_leg_positions[i][0] - prev_leg_positions[i][0]
        delta_y = current_leg_positions[i][1] - prev_leg_positions[i][1]
        
        # Only include significant movements (filter out noise)
        if abs(delta_x) + abs(delta_y) < 0.001:
            delta_positions.append((-delta_x, -delta_y))  # Negative because leg movement is opposite to body
    
    if len(delta_positions) > 0:
        avg_delta_x = sum(d[0] for d in delta_positions) / len(delta_positions)
        avg_delta_y = sum(d[1] for d in delta_positions) / len(delta_positions)
        
        # Convert to velocity
        vel_x_legs = avg_delta_x / dt
        vel_y_legs = avg_delta_y / dt
        
        return vel_x_legs, vel_y_legs
    else:
        return 0.0, 0.0

while not rospy.is_shutdown():
    leg_positions = notspot_robot.run()
    notspot_robot.change_controller()

    dx = notspot_robot.state.body_local_position[0]
    dy = notspot_robot.state.body_local_position[1]
    dz = notspot_robot.state.body_local_position[2]
    
    roll = notspot_robot.state.body_local_orientation[0]
    pitch = notspot_robot.state.body_local_orientation[1]
    yaw_local = notspot_robot.state.body_local_orientation[2]  # Local yaw

    try:
        joint_angles = inverseKinematics.inverse_kinematics(leg_positions,
                               dx, dy, dz, roll, pitch, yaw_local)

        for i in range(len(joint_angles)):
            publishers[i].publish(joint_angles[i])
    except Exception as e:
        rospy.logwarn(f"IK calculation error: {e}")
    
    # Publish odometry data
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()
    
    if dt <= 0:
        rate.sleep()
        continue
        
    last_time = current_time
    
    # Multi-source velocity estimation with sensor fusion
    vel_x = 0.0
    vel_y = 0.0
    yaw_rate = 0.0
    
    # 1. Get velocity from command (with scaling by max velocity)
    if hasattr(notspot_robot.command, 'horizontal_velocity') and hasattr(notspot_robot.command, 'vertical_velocity'):
        max_x_velocity = notspot_robot.trotGaitController.max_x_velocity if hasattr(notspot_robot, 'trotGaitController') else 0.5
        max_y_velocity = notspot_robot.trotGaitController.max_y_velocity if hasattr(notspot_robot, 'trotGaitController') else 0.5
        
        vel_x_cmd = notspot_robot.command.horizontal_velocity * max_x_velocity
        vel_y_cmd = notspot_robot.command.vertical_velocity * max_y_velocity
        yaw_rate_cmd = notspot_robot.command.yaw_rate if hasattr(notspot_robot.command, 'yaw_rate') else 0.0
    else:
        vel_x_cmd = 0.0
        vel_y_cmd = 0.0
        yaw_rate_cmd = 0.0
    
    # 2. Get velocity from leg odometry (actual leg movements)
    vel_x_legs, vel_y_legs = 0.0, 0.0
    if prev_leg_positions is not None and leg_positions is not None:
        vel_x_legs, vel_y_legs = estimate_velocity_from_legs(leg_positions, prev_leg_positions, dt)
    
    # 3. Fuse velocity estimates with weight factors
    # In trot mode, use a mix of command and leg odometry
    if hasattr(notspot_robot.state, 'behavior_state') and notspot_robot.state.behavior_state == 2:  # Trot mode
        # Give more weight to leg odometry when we have it
        if abs(vel_x_legs) > 0.001 or abs(vel_y_legs) > 0.001:
            vel_x = 0.6 * vel_x_legs + 0.4 * vel_x_cmd
            vel_y = 0.6 * vel_y_legs + 0.4 * vel_y_cmd
        else:
            # Fall back to command velocity if leg odometry is not reliable
            vel_x = vel_x_cmd
            vel_y = vel_y_cmd
    else:
        # In other modes (like rest), rely mostly on command
        vel_x = vel_x_cmd
        vel_y = vel_y_cmd
    
    yaw_rate = yaw_rate_cmd  # Use command yaw rate (could enhance with IMU)
    
    # Apply low-pass filter to velocities for smoother estimation
    vel_x_filter = filter_alpha * vel_x + (1 - filter_alpha) * last_vel_x
    vel_y_filter = filter_alpha * vel_y + (1 - filter_alpha) * last_vel_y
    yaw_rate_filter = filter_alpha * yaw_rate + (1 - filter_alpha) * last_yaw_rate
    
    last_vel_x = vel_x_filter
    last_vel_y = vel_y_filter
    last_yaw_rate = yaw_rate_filter
    
    # Update robot orientation - using IMU when available
    if hasattr(notspot_robot.state, 'imu_roll') and hasattr(notspot_robot.state, 'imu_pitch'):
        roll = notspot_robot.state.imu_roll
        pitch = notspot_robot.state.imu_pitch
        
        # If we have IMU yaw, use it for better accuracy
        if hasattr(notspot_robot.state, 'imu_yaw'):
            # IMU yaw might need an offset to align with world frame
            yaw = notspot_robot.state.imu_yaw
        else:
            # Integrate yaw rate to get yaw angle
            yaw += yaw_rate_filter * dt
            yaw = normalize_angle(yaw)
    else:
        # No IMU, use simple integration
        roll = 0.0
        pitch = 0.0
        yaw += yaw_rate_filter * dt
        yaw = normalize_angle(yaw)
    
    # Integrate velocity (in robot frame) to position (in world frame)
    # Convert robot-frame velocity to world-frame using current yaw
    world_vel_x = vel_x_filter * math.cos(yaw) - vel_y_filter * math.sin(yaw)
    world_vel_y = vel_x_filter * math.sin(yaw) + vel_y_filter * math.cos(yaw)
    
    # Update position
    x_pos += world_vel_x * dt
    y_pos += world_vel_y * dt
    
    # Save leg positions for next iteration
    prev_leg_positions = leg_positions.copy() if leg_positions is not None else None
    
    # Get quaternion from roll, pitch, yaw
    quaternion = get_quaternion_from_euler(roll, pitch, yaw)
    
    # Update odometry message
    odom.header.stamp = current_time
    odom.pose.pose.position.x = x_pos
    odom.pose.pose.position.y = y_pos
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation.x = quaternion[0]
    odom.pose.pose.orientation.y = quaternion[1]
    odom.pose.pose.orientation.z = quaternion[2]
    odom.pose.pose.orientation.w = quaternion[3]
    
    # Set velocities in the robot's frame (not the world frame)
    odom.twist.twist.linear.x = vel_x_filter
    odom.twist.twist.linear.y = vel_y_filter
    odom.twist.twist.angular.z = yaw_rate_filter
    
    # Adjust covariance based on motion state
    # Higher uncertainty when robot is moving faster
    motion_factor = math.sqrt(vel_x_filter**2 + vel_y_filter**2 + yaw_rate_filter**2) + 1.0
    for i in range(6):
        odom.pose.covariance[i*6+i] = position_covariance_diagonal[i] * motion_factor
        odom.twist.covariance[i*6+i] = twist_covariance_diagonal[i] * motion_factor
    
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

    rate.sleep()