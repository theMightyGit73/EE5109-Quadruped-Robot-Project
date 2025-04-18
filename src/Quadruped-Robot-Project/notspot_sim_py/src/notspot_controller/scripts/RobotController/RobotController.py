#!/usr/bin/evn python3
#Author: lnotspotl

import numpy as np
import tf

from . StateCommand import State, Command, BehaviorState
from . RestController import RestController
from . TrotGaitController import TrotGaitController
from . CrawlGaitController import CrawlGaitController
from . StandController import StandController
import rospy

class Robot(object):
    def __init__(self, body, legs, imu):
        self.body = body
        self.legs = legs

        self.delta_x = self.body[0] * 0.5
        self.delta_y = self.body[1] * 0.5 + self.legs[1]
        self.x_shift_front = 0.006
        self.x_shift_back = -0.03
        self.default_height = 0.15

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

    def change_controller(self):
        
        if self.command.trot_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.TROT
                self.currentController = self.trotGaitController
                self.currentController.pid_controller.reset()
                self.state.ticks = 0
            self.command.trot_event = False

        elif self.command.crawl_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.CRAWL
                self.currentController = self.crawlGaitController
                self.currentController.first_cycle = True
                self.state.ticks = 0
            self.command.crawl_event = False

        elif self.command.stand_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.STAND
                self.currentController = self.standController
            self.command.stand_event = False

        elif self.command.rest_event:
            self.state.behavior_state = BehaviorState.REST
            self.currentController = self.restController
            self.currentController.pid_controller.reset()
            self.command.rest_event = False

    def joystick_command(self,msg):
        if msg.buttons[0]: # rest
            self.command.trot_event = False
            self.command.crawl_event = False
            self.command.stand_event = False
            self.command.rest_event = True
            
        elif msg.buttons[1]: # trot
            self.command.trot_event = True
            self.command.crawl_event = False
            self.command.stand_event = False
            self.command.rest_event = False

        elif msg.buttons[2]: # crawl
            self.command.trot_event = False
            self.command.crawl_event = True
            self.command.stand_event = False
            self.command.rest_event = False
       
        elif msg.buttons[3]: # stand
            self.command.trot_event = False
            self.command.crawl_event = False
            self.command.stand_event = True
            self.command.rest_event = False

        # 🆕 Add this:
        self.command.horizontal_velocity = msg.axes[1]
        self.command.vertical_velocity = msg.axes[0]
        self.command.yaw_rate = msg.axes[2]

        # Optional: log it for debugging
        import rospy
        rospy.loginfo(f"Velocity command: vx={self.command.horizontal_velocity:.2f}, vy={self.command.vertical_velocity:.2f}, yaw={self.command.yaw_rate:.2f}")
        # end Add this

        self.currentController.updateStateCommand(msg, self.state, self.command)

    def imu_orientation(self,msg):
        q = msg.orientation
        rpy_angles = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        self.state.imu_roll = rpy_angles[0]
        self.state.imu_pitch = rpy_angles[1]

        # Optional yaw (if needed, just for debug — might drift)
        yaw = rpy_angles[2]

        # Print IMU data once per second
        rospy.loginfo_throttle(1.0, f"IMU RPY → Roll: {rpy_angles[0]:.3f}, Pitch: {rpy_angles[1]:.3f}, Yaw: {yaw:.3f}")

    def run(self):
        return self.currentController.run(self.state, self.command)

    @property
    def default_stance(self):
        # FR, FL, RR, RL
        return np.array([[self.delta_x + self.x_shift_front,self.delta_x + self.x_shift_front,-self.delta_x + self.x_shift_back,-self.delta_x + self.x_shift_back],
                         [-self.delta_y                    ,self.delta_y                     ,-self.delta_y                    , self.delta_y                    ],
                         [0                                ,0                                ,0                                ,0                                ]])
