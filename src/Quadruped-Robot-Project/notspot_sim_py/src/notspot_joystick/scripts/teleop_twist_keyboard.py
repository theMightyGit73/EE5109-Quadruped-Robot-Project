#!/usr/bin/env python3

from __future__ import print_function
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import sys, select, termios, tty
import os  # For terminal color support

# ANSI color codes for terminal output
class Colors:
    RESET = '\033[0m'
    BOLD = '\033[1m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    BG_RED = '\033[41m'
    BG_GREEN = '\033[42m'
    BG_YELLOW = '\033[43m'
    BG_BLUE = '\033[44m'
    BG_MAGENTA = '\033[45m'
    BG_CYAN = '\033[46m'
    BG_WHITE = '\033[47m'

msg = """
Reading from the keyboard and publishing to Joy!
---------------------------
Movement:
   w : Forward (Axis 1)
   x : Backward (Axis 1)
   a : Strafe Left (Axis 0)
   d : Strafe Right (Axis 0)
   e : Forward-right diagonal (Axis 0, Axis 1)
   q : Forward-left Diagonal (Axis 0, Axis 1)
   c : Backward-right diagonal (Axis 0, Axis 1)
   z : Backward-left diagonal (Axis 0, Axis 1)
   j : Rotate left (Axis 2)
   l : Rotate right (Axis 2)
   s : Stop

Rotation and Height:
   u : Lean left
   o : Lean right
   n : Lean forward
   m : Lean back
   t : Increase height (was 'i') (Axis 4)
   g : Decrease height (was 'k') (Axis 4)

Buttons (Number Keys):
   1 : (Rest)
   2 : (Trot)
   3 : (Crawl)
   4 : (Stand)
   5 : (Toggle LQR/PID controller)
   6 : (Tuning Mode)
   7 : (Autorest (Trot only))
   8 : (Roll/Pitch Compensation)
   9 : (PID Analysis - Generate graphs)

Controller Tuning Mode (after pressing 6):
   p : Select PID mode
   l : Select LQR mode
   
   When in PID mode:
     k : Select kp parameter
     i : Select ki parameter  
     d : Select kd parameter
   
   When in LQR mode:
     a : Select q_angle parameter
     r : Select q_rate parameter
     n : Select r_input parameter
     t : Select expected_dt parameter
     c : Select max_compensation parameter
   
   Shared parameter tuning:
     + : Increase selected parameter
     - : Decrease selected parameter
     [ : Decrease step size
     ] : Increase step size
     0 : Reset to default values
     v : Generate performance comparison graph
     h : Print help message

Controller Study:
   S : Start automated controller parameter study
   
CTRL-C to quit
"""

# Quick reference guide for normal mode
QUICK_REFERENCE_NORMAL = """
QUICK REFERENCE - NORMAL MODE
----------------------------
1-8: Robot modes   WASD: Movement
j/l: Rotate left/right   t/g: Height up/down
6: Enter Tuning Mode   h: Help
"""

# Quick reference guide for tuning mode
QUICK_REFERENCE_TUNING = """
QUICK REFERENCE - TUNING MODE
----------------------------
p: PID Mode   l: LQR Mode
+/-: Change value   [/]: Change step size
0: Reset defaults   v: Generate graph
Esc: Exit Tuning Mode   h: Help
"""

# Quick reference guide for each controller mode
QUICK_REFERENCE_PID = """
QUICK REFERENCE - PID PARAMETERS
-----------------------------
k: kp parameter   i: ki parameter   d: kd parameter
+/-: Change value   [/]: Change step size
0: Reset defaults   v: Generate graph
Esc: Exit Tuning Mode   h: Help
"""

QUICK_REFERENCE_LQR = """
QUICK REFERENCE - LQR PARAMETERS
-----------------------------
a: q_angle   r: q_rate   n: r_input
t: expected_dt   c: max_compensation
+/-: Change value   [/]: Change step size
0: Reset defaults   v: Generate graph
Esc: Exit Tuning Mode   h: Help
"""

# Robot mode descriptions (for clear feedback)
MODE_DESCRIPTIONS = {
    '1': "REST MODE - Robot at rest position",
    '2': "TROT MODE - Normal walking gait",
    '3': "CRAWL MODE - Slow, stable movement",
    '4': "STAND MODE - Standing still",
    '5': "CONTROLLER TOGGLE - Switch PID/LQR",
    '6': "TUNING MODE - Adjust control parameters",
    '7': "AUTOREST - Trot mode auto reset",
    '8': "IMU COMPENSATION - Roll/pitch stabilization" ,
    '9': "PID ANALYSIS - Generate PID performance graphs"
}

# Key bindings for movement, rotation, and height
moveBindings = {
    'w': (1, 0, 0, 0, 0, 0, 0, 0),   # Forward (Axis 1)
    'x': (-1, 0, 0, 0, 0, 0, 0, 0),  # Backward (Axis 1)
    'a': (0, 1, 0, 0, 0, 0, 0, 0),   # Left strafe (Axis 0)
    'd': (0, -1, 0, 0, 0, 0, 0, 0),  # Right strafe (Axis 0)
    'j': (0, 0, 1, 0, 0, 0, 0, 0),   # Rotate left (Axis 2)
    'l': (0, 0, -1, 0, 0, 0, 0, 0),  # Rotate right (Axis 2)
    't': (0, 0, 0, 1, 0, 0, 0, 0),   # Increase height (was 'i') (Axis 4)
    'g': (0, 0, 0, -1, 0, 0, 0, 0),  # Decrease height (was 'k') (Axis 4)
    'e': (1, -1, 0, 0, 0, 0, 0, 0),  # Forward-right diagonal (Axis 0, Axis 1)
    'q': (1, 1, 0, 0, 0, 0, 0, 0),   # Forward-left diagonal (Axis 0, Axis 1)
    'c': (-1, -1, 0, 0, 0, 0, 0, 0), # Backward-right diagonal (Axis 0, Axis 1)
    'z': (-1, 1, 0, 0, 0, 0, 0, 0),  # Backward-left diagonal (Axis 0, Axis 1)
    'u': (0, 0, 0, 0, 0, 1, 0, 0),   # Lean left
    'o': (0, 0, 0, 0, 0, -1, 0, 0),  # Lean right
    'n': (0, 0, 0, 0, 0, 0, 0, 1),   # Lean forward
    'm': (0, 0, 0, 0, 0, 0, 0, -1),  # Lean back
    's': (0, 0, 0, 0, 0, 0, 0, 0),   # Stop
}

# Button bindings for number keys
buttonBindings = {
    '1': 0,  # Button 0 (Rest)
    '2': 1,  # Button 1 (Trot)
    '3': 2,  # Button 2 (Crawl)
    '4': 3,  # Button 3 (Stand)
    '5': 4,  # Button 4 (Toggle LQR/PID)
    '6': 5,  # Button 5 (Tuning Mode)
    '7': 6,  # Button 6 (Autorest)
    '8': 7,  # Button 7 (Roll/Pitch Compensation)
    '9': 8,  # Button 8 (PID Analysis)
}

# Special command bindings
commandBindings = {
    'v': 'generate_graph',    # Generate performance comparison graph
    'S': 'start_study',       # Start automated controller parameter study
    '0': 'reset_params',      # Reset parameters to default values
    '\x1b': 'exit_tuning',    # ESC key to exit tuning mode
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params):
    """Print current mode and selected parameter with enhanced visual feedback"""
    
    if not tuning_mode:
        # Normal mode
        print("\n" + "="*60)
        print(f"CURRENT MODE: {Colors.BOLD}{Colors.CYAN}NORMAL ROBOT CONTROL{Colors.RESET}")
        print("="*60)
        print(QUICK_REFERENCE_NORMAL)
        return
    
    # We're in tuning mode
    if pid_target == "PID":
        color_prefix = Colors.GREEN
        mode_header = f"{Colors.BOLD}{color_prefix}PID TUNING MODE{Colors.RESET}"
        quick_ref = QUICK_REFERENCE_PID
    else:
        color_prefix = Colors.BLUE
        mode_header = f"{Colors.BOLD}{color_prefix}LQR TUNING MODE{Colors.RESET}"
        quick_ref = QUICK_REFERENCE_LQR
    
    print("\n" + "="*60)
    print(f"CURRENT MODE: {mode_header} - Step size: {Colors.BOLD}{param_step:.4f}{Colors.RESET}")
    
    # Print parameters with the selected one highlighted
    if pid_target == "PID":
        # Print all parameter values with selected one highlighted
        for param, display_name in [('kp', 'kp'), ('ki', 'ki'), ('kd', 'kd')]:
            if param == current_param:
                print(f"{Colors.BG_YELLOW}{Colors.BOLD}► {display_name}: {pid_params[param]:.4f} ◄{Colors.RESET}")
            else:
                print(f"  {display_name}: {pid_params[param]:.4f}")
    else:
        # Print all parameter values with selected one highlighted
        for param, display_name in [
            ('q_angle', 'q_angle'), 
            ('q_rate', 'q_rate'), 
            ('r_input', 'r_input'), 
            ('expected_dt', 'expected_dt'), 
            ('max_compensation', 'max_comp')
        ]:
            if param == current_lqr_param:
                print(f"{Colors.BG_YELLOW}{Colors.BOLD}► {display_name}: {lqr_params[param]:.4f} ◄{Colors.RESET}")
            else:
                print(f"  {display_name}: {lqr_params[param]:.4f}")
    
    print("="*60)
    print(quick_ref)

def notify_mode_change(mode_name):
    """Print a highly visible mode change notification"""
    print(f"\n{Colors.BOLD}{Colors.BG_BLUE}" + "!"*50 + f"{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BG_BLUE}!!  ROBOT MODE CHANGED: {mode_name}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BG_BLUE}" + "!"*50 + f"{Colors.RESET}\n")
    
def notify_controller_change(controller_type):
    """Print a highly visible controller change notification"""
    color = Colors.BG_GREEN if controller_type == "PID" else Colors.BG_BLUE
    print(f"\n{Colors.BOLD}{color}" + "#"*50 + f"{Colors.RESET}")
    print(f"{Colors.BOLD}{color}##  CONTROLLER SWITCHED TO: {controller_type}{Colors.RESET}")
    print(f"{Colors.BOLD}{color}" + "#"*50 + f"{Colors.RESET}\n")

def notify_tuning_mode_change(enabled):
    """Print a highly visible tuning mode change notification"""
    if enabled:
        print(f"\n{Colors.BOLD}{Colors.BG_YELLOW}" + "="*50 + f"{Colors.RESET}")
        print(f"{Colors.BOLD}{Colors.BG_YELLOW}==  ENTERING TUNING MODE - Press 'p' for PID or 'l' for LQR  =={Colors.RESET}")
        print(f"{Colors.BOLD}{Colors.BG_YELLOW}==  Press ESC to exit tuning mode at any time             =={Colors.RESET}")
        print(f"{Colors.BOLD}{Colors.BG_YELLOW}" + "="*50 + f"{Colors.RESET}\n")
        print(QUICK_REFERENCE_TUNING)
    else:
        print(f"\n{Colors.BOLD}{Colors.BG_CYAN}" + "="*50 + f"{Colors.RESET}")
        print(f"{Colors.BOLD}{Colors.BG_CYAN}==  EXITING TUNING MODE - Returning to normal control  =={Colors.RESET}")
        print(f"{Colors.BOLD}{Colors.BG_CYAN}" + "="*50 + f"{Colors.RESET}\n")

def notify_parameter_change(param_name, value, direction):
    """Print a visible parameter change notification"""
    arrow = "↑" if direction > 0 else "↓"
    print(f"{Colors.YELLOW}" + "-"*40 + f"{Colors.RESET}")
    print(f"{Colors.YELLOW}Parameter {arrow} {param_name}: {value:.4f}{Colors.RESET}")
    print(f"{Colors.YELLOW}" + "-"*40 + f"{Colors.RESET}")

def notify_study_started():
    """Print notification for study start"""
    print(f"\n{Colors.MAGENTA}" + "*"*50 + f"{Colors.RESET}")
    print(f"{Colors.MAGENTA}* STARTING CONTROLLER PARAMETER STUDY *{Colors.RESET}")
    print(f"{Colors.MAGENTA}* This will take several minutes...        *{Colors.RESET}")
    print(f"{Colors.MAGENTA}" + "*"*50 + f"{Colors.RESET}\n")

def print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step):
    """Print a concise status header showing the most important information"""
    if tuning_mode:
        active_controller = pid_target
        active_param = current_param if pid_target == "PID" else current_lqr_param
        status = f"{Colors.BOLD}STATUS:{Colors.RESET} "
        status += f"Mode [{Colors.MAGENTA}TUNING{Colors.RESET}] | "
        status += f"Robot [{Colors.GREEN}{robot_mode}{Colors.RESET}] | "
        status += f"Controller [{Colors.BLUE}{active_controller}{Colors.RESET}] | "
        status += f"Parameter [{Colors.YELLOW}{active_param}{Colors.RESET}] | "
        status += f"Step [{Colors.CYAN}{param_step:.4f}{Colors.RESET}]"
    else:
        status = f"{Colors.BOLD}STATUS:{Colors.RESET} "
        status += f"Mode [{Colors.CYAN}NORMAL{Colors.RESET}] | "
        status += f"Robot [{Colors.GREEN}{robot_mode}{Colors.RESET}]"
    
    print("\n" + status + "\n")

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_keyboard')
    joy_pub = rospy.Publisher('/notspot_joy/joy_ramped', Joy, queue_size=1)
    
    # New publisher for PID tuning commands
    pid_tuning_pub = rospy.Publisher('/notspot_controller/pid_tuning', Float64MultiArray, queue_size=5)
    
    # PID tuning parameters
    pid_params = {
        'kp': 0.75,  # Default PID values from the code
        'ki': 2.29,
        'kd': 0.0
    }
    
    # LQR parameters
    lqr_params = {
        'q_angle': 1.2,         # Default LQR values
        'q_rate': 0.12,
        'r_input': 0.003,
        'expected_dt': 0.02,
        'max_compensation': 0.5
    }
    
    current_param = 'kp'     # Currently selected parameter (PID mode)
    current_lqr_param = 'q_angle'  # Currently selected parameter (LQR mode)
    param_step = 0.05       # Default step value for parameter changes
    pid_target = "PID"      # Target controller to tune (PID or LQR)
    robot_mode = "REST"     # Current robot mode
    tuning_mode = False     # Whether we're in tuning mode
    
    print(msg)
    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
    rospy.loginfo("Teleop keyboard running with enhanced tuning mode")
    
    try:
        while not rospy.is_shutdown():
            key = getKey()
            
            joy = Joy()
            joy.axes = [0.0] * 8  # Initialize with 8 axes
            joy.buttons = [0] * 11  # Initialize with 11 buttons
            
            # Special case: ESC key to exit tuning mode
            if key == '\x1b':  # ESC key
                if tuning_mode:
                    tuning_mode = False
                    notify_tuning_mode_change(False)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                continue
                
            # Special case for button 6 (tuning mode)
            if key == '6':
                tuning_mode = True
                notify_tuning_mode_change(True)
                print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                continue
                
            # Handle normal mode keys
            if not tuning_mode:
                # Handle movement keys (case-sensitive)
                if key in moveBindings.keys():
                    x, y, z, th, b, c, d, e = moveBindings[key]

                    # Map key bindings to axes
                    joy.axes[4] = x  # Left/Right (Axis 0)
                    joy.axes[3] = y  # Forward/Backward (Axis 1)
                    joy.axes[0] = z  # Rotation (Axis 2)
                    joy.axes[1] = th  # Height (Axis 4)
                    joy.axes[2] = b
                    joy.axes[6] = c
                    joy.axes[5] = d
                    joy.axes[7] = e
                    
                    # Show what movement command was sent
                    move_name = key
                    if key == 'w': move_name = "Forward"
                    elif key == 'x': move_name = "Backward"
                    elif key == 'a': move_name = "Strafe Left"
                    elif key == 'd': move_name = "Strafe Right"
                    elif key == 'j': move_name = "Rotate Left"
                    elif key == 'l': move_name = "Rotate Right"
                    elif key == 't': move_name = "Increase Height"
                    elif key == 'g': move_name = "Decrease Height"
                    
                    print(f"Movement: {move_name}")
                    
                    # Publish the Joy message
                    joy_pub.publish(joy)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)

                # Handle button keys
                elif key in buttonBindings.keys():
                    # Map number keys to buttons
                    button_index = buttonBindings[key]
                    joy.buttons[button_index] = 1  # Press the button
                    
                    # Show what mode was selected with a prominent notification
                    if key in MODE_DESCRIPTIONS:
                        notify_mode_change(MODE_DESCRIPTIONS[key])
                        if key == '1':
                            robot_mode = "REST"
                        elif key == '2':
                            robot_mode = "TROT"
                        elif key == '3':
                            robot_mode = "CRAWL"
                        elif key == '4':
                            robot_mode = "STAND"

                        # Special handling for PID Analysis
                        if key == '9':
                            print(f"\n{Colors.MAGENTA}Generating automatic PID analysis...{Colors.RESET}\n")

                    # Publish the Joy message
                    joy_pub.publish(joy)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                    
                # Print help
                elif key == 'h':
                    print(msg)
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    
                # Otherwise ignore the key in normal mode
                
            # Handle tuning mode keys
            else:
                # First, check for controller selection keys
                if key == 'p':  # Select PID controller
                    pid_target = "PID"
                    notify_controller_change("PID")
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                
                elif key == 'l':  # Select LQR controller
                    pid_target = "LQR"
                    notify_controller_change("LQR")
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                
                # Check for PID parameter selection keys
                elif pid_target == "PID" and key == 'k':  # Changed from 'p' to 'k' for kp
                    current_param = 'kp'
                    rospy.loginfo(f"Selected PID kp parameter: {pid_params['kp']:.4f}")
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                    
                elif pid_target == "PID" and key == 'i':
                    current_param = 'ki'
                    rospy.loginfo(f"Selected PID ki parameter: {pid_params['ki']:.4f}")
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                    
                elif pid_target == "PID" and key == 'd':
                    current_param = 'kd'
                    rospy.loginfo(f"Selected PID kd parameter: {pid_params['kd']:.4f}")
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                
                # Check for LQR parameter selection keys    
                elif pid_target == "LQR" and key == 'a':
                    current_lqr_param = 'q_angle'
                    rospy.loginfo(f"Selected LQR q_angle parameter: {lqr_params['q_angle']:.4f}")
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                    
                elif pid_target == "LQR" and key == 'r':
                    current_lqr_param = 'q_rate'
                    rospy.loginfo(f"Selected LQR q_rate parameter: {lqr_params['q_rate']:.4f}")
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                    
                elif pid_target == "LQR" and key == 'n':  # Changed from 'i' to 'n' for r_input
                    current_lqr_param = 'r_input'
                    rospy.loginfo(f"Selected LQR r_input parameter: {lqr_params['r_input']:.4f}")
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                    
                elif pid_target == "LQR" and key == 't':
                    current_lqr_param = 'expected_dt'
                    rospy.loginfo(f"Selected LQR expected_dt parameter: {lqr_params['expected_dt']:.4f}")
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                    
                elif pid_target == "LQR" and key == 'c':
                    current_lqr_param = 'max_compensation'
                    rospy.loginfo(f"Selected LQR max_compensation parameter: {lqr_params['max_compensation']:.4f}")
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                    
                # Handle parameter value changes
                elif key == '+' or key == '=':  # '=' is on the same key as '+' on most keyboards
                    tuning_msg = Float64MultiArray()
                    
                    if pid_target == "PID":
                        # Store previous value for display
                        old_value = pid_params[current_param]
                        
                        # Update PID parameter
                        pid_params[current_param] += param_step
                        rospy.loginfo(f"Increased PID {current_param} to {pid_params[current_param]:.4f}")
                        
                        # Show parameter change visually
                        notify_parameter_change(current_param, pid_params[current_param], 1)
                        
                        # Send updated PID parameters
                        tuning_msg.data = [
                            0,  # 0 for PID
                            ord(current_param[0].upper()),  # First letter capitalized
                            pid_params['kp'],
                            pid_params['ki'],
                            pid_params['kd'],
                            param_step
                        ]
                    else:
                        # Store previous value for display
                        old_value = lqr_params[current_lqr_param]
                        
                        # Update LQR parameter
                        lqr_params[current_lqr_param] += param_step
                        rospy.loginfo(f"Increased LQR {current_lqr_param} to {lqr_params[current_lqr_param]:.4f}")
                        
                        # Show parameter change visually
                        notify_parameter_change(current_lqr_param, lqr_params[current_lqr_param], 1)
                        
                        # Send updated LQR parameters with proper parameter code
                        param_code = {
                            'q_angle': ord('A'),
                            'q_rate': ord('R'),
                            'r_input': ord('I'),
                            'expected_dt': ord('T'),
                            'max_compensation': ord('C')
                        }[current_lqr_param]
                        
                        tuning_msg.data = [
                            1,  # 1 for LQR
                            param_code,
                            lqr_params['q_angle'],
                            lqr_params['q_rate'],
                            lqr_params['r_input'],
                            lqr_params['expected_dt'],
                            lqr_params['max_compensation'],
                            param_step
                        ]
                    
                    pid_tuning_pub.publish(tuning_msg)
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                    
                elif key == '-':
                    tuning_msg = Float64MultiArray()
                    
                    if pid_target == "PID":
                        # Store previous value for display
                        old_value = pid_params[current_param]
                        
                        # Update PID parameter (ensure it doesn't go below 0)
                        pid_params[current_param] = max(0, pid_params[current_param] - param_step)
                        rospy.loginfo(f"Decreased PID {current_param} to {pid_params[current_param]:.4f}")
                        
                        # Show parameter change visually
                        notify_parameter_change(current_param, pid_params[current_param], -1)
                        
                        # Send updated PID parameters
                        tuning_msg.data = [
                            0,  # 0 for PID
                            ord(current_param[0].upper()),  # First letter capitalized 
                            pid_params['kp'],
                            pid_params['ki'],
                            pid_params['kd'],
                            -param_step
                        ]
                    else:
                        # Store previous value for display
                        old_value = lqr_params[current_lqr_param]
                        
                        # Update LQR parameter (ensure it doesn't go below 0)
                        lqr_params[current_lqr_param] = max(0, lqr_params[current_lqr_param] - param_step)
                        rospy.loginfo(f"Decreased LQR {current_lqr_param} to {lqr_params[current_lqr_param]:.4f}")
                        
                        # Show parameter change visually
                        notify_parameter_change(current_lqr_param, lqr_params[current_lqr_param], -1)
                        
                        # Send updated LQR parameters with proper parameter code
                        param_code = {
                            'q_angle': ord('A'),
                            'q_rate': ord('R'),
                            'r_input': ord('I'),
                            'expected_dt': ord('T'),
                            'max_compensation': ord('C')
                        }[current_lqr_param]
                        
                        tuning_msg.data = [
                            1,  # 1 for LQR
                            param_code,
                            lqr_params['q_angle'],
                            lqr_params['q_rate'],
                            lqr_params['r_input'],
                            lqr_params['expected_dt'],
                            lqr_params['max_compensation'],
                            -param_step
                        ]
                    
                    pid_tuning_pub.publish(tuning_msg)
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                
                # Handle special commands
                elif key in commandBindings:
                    command = commandBindings[key]
                    
                    # Create command message
                    tuning_msg = Float64MultiArray()
                    
                    if command == 'reset_params':
                        # Reset to default values based on the code
                        if pid_target == "PID":
                            pid_params = {'kp': 0.75, 'ki': 2.29, 'kd': 0.0}
                            rospy.loginfo(f"Reset PID parameters to defaults")
                            print(f"\n{Colors.CYAN}Parameters reset to defaults{Colors.RESET}\n")
                            tuning_msg.data = [2, 0, 0, 0, 0, 0]  # 2 = reset command for PID
                        else:
                            lqr_params = {
                                'q_angle': 1.2,
                                'q_rate': 0.12,
                                'r_input': 0.003,
                                'expected_dt': 0.02,
                                'max_compensation': 0.5
                            }
                            rospy.loginfo(f"Reset LQR parameters to defaults")
                            print(f"\n{Colors.CYAN}Parameters reset to defaults{Colors.RESET}\n")
                            tuning_msg.data = [2, 1, 0, 0, 0, 0, 0, 0]  # 2 = reset command for LQR
                        
                    elif command == 'generate_graph':
                        if tuning_mode and pid_target == "PID":
                            print(f"\n{Colors.MAGENTA}Generating enhanced PID tuning graphs...{Colors.RESET}\n")
                            # Use command code 6 for enhanced PID graphs
                            tuning_msg.data = [6, 0, 0, 0, 0, 0]  # 6 = generate enhanced PID graphs
                        else:
                            print(f"\n{Colors.MAGENTA}Generating performance comparison graph...{Colors.RESET}\n")
                            # Use the original command code 4 for standard graphs
                            tuning_msg.data = [4, 0, 0, 0, 0, 0]  # 4 = generate standard graph
                        
                    elif command == 'start_study':
                        rospy.loginfo("Starting automated controller parameter study")
                        
                        # Show study start notification
                        notify_study_started()
                        
                        tuning_msg.data = [5, 0, 0, 0, 0, 0]  # 5 = start study
                    
                    if command != 'exit_tuning':  # Skip publishing if just exiting tuning mode
                        pid_tuning_pub.publish(tuning_msg)
                        printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                        print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                    
                # Change parameter step size
                elif key == '[':
                    param_step = max(0.001, param_step/2)
                    rospy.loginfo(f"Decreased step size to {param_step:.4f}")
                    print(f"{Colors.CYAN}Step size decreased to {param_step:.4f}{Colors.RESET}")
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                    
                elif key == ']':
                    param_step = param_step*2
                    rospy.loginfo(f"Increased step size to {param_step:.4f}")
                    print(f"{Colors.CYAN}Step size increased to {param_step:.4f}{Colors.RESET}")
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                    
                # Print help
                elif key == 'h':
                    print(msg)
                    printCurrentMode(tuning_mode, pid_target, current_param, current_lqr_param, param_step, pid_params, lqr_params)
                    print_status_header(tuning_mode, pid_target, current_param, current_lqr_param, robot_mode, param_step)
                
            # Check for CTRL-C to quit
            if key == '\x03':  # CTRL-C
                break
                
    except Exception as e:
        print(f"{Colors.RED}Error: {e}{Colors.RESET}")

    finally:
        # Reset the Joy message
        joy = Joy()
        joy.axes = [0.0] * 8
        joy.buttons = [0] * 11
        joy_pub.publish(joy)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)