#! /usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import JointState

from yumi import YuMi
from gello import GELLO

class YuMiTeleop:
    def __init__(self):
        rospy.init_node('gumi_ros_node', anonymous=True)
        # register publisher for the teleoperation commands
        self.teleop_pub = rospy.Publisher('/yumi/teleop/action', JointState, queue_size=10)
        # Initialize the YuMi class
        self.yumi = YuMi()
        # Initialize the GELLO class
        self.gello = GELLO(left_init_states=self.yumi.left_grasp_states, right_init_states=self.yumi.right_grasp_states)

        # Initialize the JointState message
        self.joint_state = JointState()
        self.joint_state.name = self.yumi.joint_names

        # slow down the speed of the robot
        self.yumi.change_max_speed(self.yumi.left_rapid_task, 10) # 5 degrees per second
        self.yumi.change_max_speed(self.yumi.right_rapid_task, 10) # 5 degrees per second

        # yumi.stop_egm()
        self.yumi.start_egm()
        self.yumi.start_controllers()

        # move yumi to the initial position
        self.yumi.command_joint_state(self.yumi.left_grasp_states + self.yumi.right_grasp_states)
        key = input("Press Enter when the movement is done")
        if key == 'q':
            exit()
        gello_states = self.gello.get_left_states()[0] + self.gello.get_right_states()[0]
        self.yumi.command_joint_state(gello_states)
        key = input("Press Enter when the movement is done")
        if key == 'q':
            exit()

        self.yumi.stop_egm()
        # slow down the speed of the robot
        self.yumi.change_max_speed(self.yumi.left_rapid_task, 180) # 5 degrees per second
        self.yumi.change_max_speed(self.yumi.right_rapid_task, 180) # 5 degrees per second

        self.yumi.start_egm()
        self.yumi.start_controllers()
        self.yumi.keep_egm_alive = True

    def get_gripper_section(self, gripper):
        if gripper < 0.1:
            return 0
        elif gripper > 0.9:
            return 2
        else:
            return 1

    def run(self):
        # Start the teleoperation
        control_rate = 30
        last_gripper_section = [0, 0]
        while not rospy.is_shutdown():
            left_states, left_gripper = self.gello.get_left_states()
            right_states, right_gripper = self.gello.get_right_states()
            self.yumi.command_joint_state(left_states + right_states)
            gripper_section = [self.get_gripper_section(left_gripper), self.get_gripper_section(right_gripper)]
            if gripper_section != last_gripper_section:
                last_gripper_section = gripper_section
                if gripper_section[0] == 0:
                    self.yumi.sendLeftGripperCmd(19, wait=False)
                elif gripper_section[0] == 2:
                    self.yumi.setLeftGripperPos(25, wait=False)
                else:
                    self.yumi.setLeftGripperPos(10, wait=False)
                if gripper_section[1] == 0:
                    self.yumi.sendRightGripperCmd(19, wait=False)
                elif gripper_section[1] == 2:
                    self.yumi.setRightGripperPos(25, wait=False)
                else:
                    self.yumi.setRightGripperPos(10, wait=False)

            # publish the joint states
            self.joint_state.position = right_states + left_states + \
                [right_gripper, right_gripper] + [left_gripper, left_gripper]
            self.joint_state.header.stamp = rospy.Time.now()
            self.teleop_pub.publish(self.joint_state)
            rospy.sleep(1/control_rate)

if __name__ == "__main__":
    teleop = YuMiTeleop()
    teleop.run()