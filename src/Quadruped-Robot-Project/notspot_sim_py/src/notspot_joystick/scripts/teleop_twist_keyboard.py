#!/usr/bin/env python3

from __future__ import print_function
import rospy
from sensor_msgs.msg import Joy
import sys, select, termios, tty

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
   a : Twist left
   d : Twist right
   u : Lean left
   o : Lean right
   j : pivot left
   l : pivot right
   n : Lean forward
   m : Lean back
   i : Increase height (Axis 4)
   k : Decrease height (Axis 4)

Buttons (Number Keys):
   1 : (Rest)
   2 : (Trot)
   3 : (Crawl)
   4 : (Stand)
   5 : (Toggle LQR/PID controller)
   6 : (Unused)
   7 : (Autorest (Trot only))
   8 : (Roll/Pitch Compensation)

CTRL-C to quit
"""

# Key bindings for movement, rotation, and height
moveBindings = {
    'w': (1, 0, 0, 0, 0, 0, 0, 0),   # Forward (Axis 1)
    'x': (-1, 0, 0, 0, 0, 0, 0, 0),  # Backward (Axis 1)
    'a': (0, 1, 0, 0, 0, 0, 0, 0),   # Left strafe (Axis 0)
    'd': (0, -1, 0, 0, 0, 0, 0, 0),  # Right strafe (Axis 0)
    'j': (0, 0, 1, 0, 0, 0, 0, 0),   # Rotate left (Axis 2)
    'l': (0, 0, -1, 0, 0, 0, 0, 0),  # Rotate right (Axis 2)
    'i': (0, 0, 0, 1, 0, 0, 0, 0),   # Increase height (Axis 4)
    'k': (0, 0, 0, -1, 0, 0, 0, 0),  # Decrease height (Axis 4)
    'e': (1, -1, 0, 0, 0, 0, 0, 0),  # Forward-right diagonal (Axis 0, Axis 1)
    'q': (1, 1, 0, 0, 0, 0, 0, 0),   # Forward-left diagonal (Axis 0, Axis 1)
    'c': (-1, -1, 0, 0, 0, 0, 0, 0), # Backward-right diagonal (Axis 0, Axis 1)
    'z': (-1, 1, 0, 0, 0, 0, 0, 0),  # Backward-left diagonal (Axis 0, Axis 1)
    'u': (0, 0, 0, 0, 0, 1, 0, 0),   # Lean left
    'o': (0, 0, 0, 0, 0, -1, 0, 0),  # Lean right
    'p': (0, 0, 0, 0, 0, 0, 1, 0),
    'b': (0, 0, 0, 0, 1, 0, 0, 0),
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
    '6': 5,  # Button 5
    '7': 6,  # Button 6 (Autorest)
    '8': 7,  # Button 7 (Roll/Pitch Compensation)
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')
    joy_pub = rospy.Publisher('/notspot_joy/joy_ramped', Joy, queue_size=1)

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            joy = Joy()
            joy.axes = [0.0] * 8  # Initialize with 8 axes
            joy.buttons = [0] * 11  # Initialize with 11 buttons

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
                
                #rospy.loginfo(f"[DEBUG] Key '{key}' → Axes: {joy.axes}")


                # Debug output to check the axes values
                #print(f"Key pressed: {key}")
                #print(f"Axes Values: {joy.axes}")  # This will print out the updated axes values

            elif key in buttonBindings.keys():
                # Map number keys to buttons
                button_index = buttonBindings[key]
                joy.buttons[button_index] = 1  # Press the button

            elif key == '\x03':  # CTRL-C
                break

            # Publish the Joy message
            joy_pub.publish(joy)

    except Exception as e:
        print(e)

    finally:
        # Reset the Joy message
        joy = Joy()
        joy.axes = [0.0] * 8
        joy.buttons = [0] * 11
        joy_pub.publish(joy)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)