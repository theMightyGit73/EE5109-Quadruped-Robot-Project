#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

class GroundTruthOdom:
    def __init__(self):
        rospy.init_node('ground_truth_odom_publisher')

        self.robot_name = "notspot_gazebo"  # change to match your robot's name in Gazebo
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_callback)

    def model_state_callback(self, msg):
        try:
            index = msg.name.index(self.robot_name)
        except ValueError:
            rospy.logwarn_throttle(5, f"Robot '{self.robot_name}' not found in /gazebo/model_states.")
            return

        pose = msg.pose[index]
        twist = msg.twist[index]

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose = pose
        odom.twist.twist = twist

        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        node = GroundTruthOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
