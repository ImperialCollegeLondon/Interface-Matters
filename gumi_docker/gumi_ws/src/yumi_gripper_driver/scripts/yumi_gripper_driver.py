#! /usr/bin/env python3

import rospy
import threading
import numpy as np
from math import sqrt
from time import sleep
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from abb_robot_msgs.srv import TriggerWithResultCode
from abb_robot_msgs.msg import SystemState
from yumi_gripper_socket_client import YumiGripperSocketClient

class YumiGripperInterface:
    # gripper_joint_names = ['gripper_r_joint', 'gripper_r_joint_m','gripper_l_joint', 'gripper_l_joint_m']
    gripper_joint_names = ['gripper_l_joint', 'gripper_r_joint']

    def __init__(self, base_link='yumi_base_link'):
        rospy.init_node('yumi_gripper_driver', anonymous=True)
        rospy.wait_for_service('/yumi/rws/pp_to_main')
        pp_to_main_client = rospy.ServiceProxy('/yumi/rws/pp_to_main', TriggerWithResultCode)
        rospy.wait_for_service('/yumi/rws/start_rapid')
        start_rapid_client = rospy.ServiceProxy('/yumi/rws/start_rapid', TriggerWithResultCode)
        rospy.wait_for_service('/yumi/rws/set_motors_on')
        set_motors_on_client = rospy.ServiceProxy('/yumi/rws/set_motors_on', TriggerWithResultCode)
        rospy.sleep(0.5)

        # check if RAPID codes are running, call the service to start them if not
        rapid_active = False
        while not rapid_active:
            # check if the RAPID programs running conditions are met
            system_states = rospy.wait_for_message('/yumi/rws/system_states', SystemState)
            if not system_states.motors_on:
                print('Motors off, turning on...')
                set_motors_on_client()
                rospy.sleep(1)
                continue
            if not system_states.auto_mode:
                print('Mate, you need to switch to auto mode.')
                rospy.sleep(1)
                continue
            # set the task pointers to the main
            pp_to_main_ok = pp_to_main_client()
            if pp_to_main_ok.result_code == 3004:
                print('RAPID programs are already running.')
                rapid_active = True
                break
            # start the RAPID programs
            start_rapid_ok = start_rapid_client()
            if pp_to_main_ok.result_code == 1 and start_rapid_ok.result_code == 1:
                rapid_active = True
                print('RAPID programs started.')
                rospy.sleep(0.5)
            else:
                if pp_to_main_ok.result_code != 1:
                    print('pp_to_main failed (code: {}, message: {})'.format(
                        pp_to_main_ok.result_code, pp_to_main_ok.message))
                if start_rapid_ok.result_code != 1:
                    print('start_rapid failed (code: {}, message: {})'.format(
                        start_rapid_ok.result_code, start_rapid_ok.message))
                rospy.sleep(0.5)

        # rospy.on_shutdown(self.on_exit)
        self.base_link = base_link
        ip = rospy.get_param('~yumi_ip', 'localhost')
        self.yumi_gripper_socket_client = YumiGripperSocketClient(ip=ip)

        # initialise the state and target buffers
        self.gripper_l, self.gripper_r = self.yumi_gripper_socket_client.receive_states()
        self.gripper_l_tar = self.gripper_l
        self.gripper_r_tar = self.gripper_r
        self.gripper_l_eff_tar = 0.0
        self.gripper_r_eff_tar = 0.0
        self.gripper_mode = 0
        self.yumi_gripper_socket_client.send_motions_l(0, 0)
        self.yumi_gripper_socket_client.send_motions_r(0, 0)

        self.yumi_gripper_states_pub = rospy.Publisher("/yumi/gripper_states",
                                                       JointState, queue_size=1)
        self.yumi_gripper_effort_sub_l = rospy.Subscriber("/yumi/gripper_l_effort_cmd", 
                                                          Float64, self.gripper_effort_cb_l)
        self.yumi_gripper_effort_sub_r = rospy.Subscriber("/yumi/gripper_r_effort_cmd", 
                                                          Float64, self.gripper_effort_cb_r)
        self.yumi_gripper_position_sub_l = rospy.Subscriber("/yumi/gripper_l_position_cmd", 
                                                            Float64, self.gripper_position_cb_l)
        self.yumi_gripper_position_sub_r = rospy.Subscriber("/yumi/gripper_r_position_cmd", 
                                                            Float64, self.gripper_position_cb_r)

        self.run_gripper()

    def pub_gripper_states(self):
        """ transform joint state lists to jointstate messages """
        gripper_states = JointState()
        gripper_states.header.stamp = rospy.Time.now()
        gripper_states.header.frame_id = self.base_link
        gripper_states.name = self.gripper_joint_names
        gripper_states.position = [self.gripper_l/1000, self.gripper_r/1000]
        gripper_states.velocity = []
        gripper_states.effort = []
        self.yumi_gripper_states_pub.publish(gripper_states)

    def run_gripper(self):
        """ gripper thread: reading and writing through websocket constantly (run in background) """
        rospy.loginfo('Gripper interface online.')
        while not rospy.is_shutdown():
            # print(self.gripper_l, self.gripper_r)
            state = self.yumi_gripper_socket_client.receive_states()
            if state is not None:
                self.gripper_l, self.gripper_r = state
                self.pub_gripper_states()
            sleep(0.01)

    def gripper_effort_cb_l(self, msg):
        self.gripper_l_eff_tar = msg.data
        self.yumi_gripper_socket_client.send_motions_l(self.gripper_l_eff_tar, 0)

    def gripper_effort_cb_r(self, msg):
        self.gripper_r_eff_tar = msg.data
        self.yumi_gripper_socket_client.send_motions_r(self.gripper_r_eff_tar, 0)

    def gripper_position_cb_l(self, msg):
        self.gripper_l_tar = msg.data
        self.yumi_gripper_socket_client.send_motions_l(self.gripper_l_tar, 1)

    def gripper_position_cb_r(self, msg):
        self.gripper_r_tar = msg.data
        self.yumi_gripper_socket_client.send_motions_r(self.gripper_r_tar, 1)

    def set_gripper_effort(self, effort_l, effort_r):
        """ sets the desired effort for the gripper's fingers. Effort should be in [-20, 20] """
        self.gripper_l_eff_tar = effort_l
        self.gripper_r_eff_tar = effort_r
        self.yumi_gripper_socket_client.send_motions_l(self.gripper_l_eff_tar, 0)
        self.yumi_gripper_socket_client.send_motions_r(self.gripper_r_eff_tar, 0)

    def set_gripper_position(self, pos_l, pos_r):
        """ sets the desired position for the gripper's fingers. Position should be in [0, 25] """
        self.gripper_l_tar = pos_l
        self.gripper_r_tar = pos_r
        self.yumi_gripper_socket_client.send_motions_l(self.gripper_l_tar, 1)
        self.yumi_gripper_socket_client.send_motions_r(self.gripper_r_tar, 1)

if __name__ == '__main__':
    yumi_gripper = YumiGripperInterface()
