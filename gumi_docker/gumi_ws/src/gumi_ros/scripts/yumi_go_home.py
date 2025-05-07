#! /usr/bin/env python3

import rospy
from yumi import YuMi

if __name__ == "__main__":
    rospy.init_node('yumi_wrapper', anonymous=True)
    # Initialize the YumiRobot class
    yumi = YuMi()

    yumi.stop_egm()
    # slow down the speed of the robot
    yumi.change_max_speed(yumi.left_rapid_task, 10) # 5 degrees per second

    # yumi.stop_egm()
    # yumi.start_egm()
    # yumi.start_controllers()

    yumi.change_max_speed(yumi.right_rapid_task, 10) # 5 degrees per second

    # yumi.stop_egm()
    yumi.start_egm()
    yumi.start_controllers()

    # move yumi to the initial position
    yumi.command_joint_state(yumi.left_calc_states + yumi.right_calc_states)
    input("Press Enter when the movement is done")
