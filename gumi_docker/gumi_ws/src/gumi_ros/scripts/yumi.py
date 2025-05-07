from typing import Dict

import numpy as np
from math import sqrt
from random import random

import rospy
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState

from abb_robot_msgs.srv import TriggerWithResultCode
from abb_egm_msgs.msg import EGMState
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from abb_rapid_sm_addin_msgs.srv import GetEGMSettings, SetEGMSettings

# from base_robot import Robot


class YuMi():
    GRIPPER_TOLERANCE = 0.5 # mm
    TIME_TOLERANCE = 50
    GRIPPER_TIME_TOLERANCE = 5

    keep_egm_alive = False

    left_rapid_task = 'T_ROB_L'
    right_rapid_task = 'T_ROB_R'
    joint_names = ['yumi_robr_joint_1', 'yumi_robr_joint_2', 'yumi_robr_joint_3', 'yumi_robr_joint_4', 
                   'yumi_robr_joint_5', 'yumi_robr_joint_6', 'yumi_robr_joint_7', 'yumi_robl_joint_1', 
                   'yumi_robl_joint_2', 'yumi_robl_joint_3', 'yumi_robl_joint_4', 'yumi_robl_joint_5',
                   'yumi_robl_joint_6', 'yumi_robl_joint_7', 'gripper_r_joint', 'gripper_r_joint_m',
                   'gripper_l_joint', 'gripper_l_joint_m']
    
    # fix the left arm and gripper joints
    left_calc_states = [0, -2.2689, 2.3562, 0.5236, 0, 0.6981, 0]
    right_calc_states = [0, -2.2689, -2.3562, 0.5236, 0, 0.6981, 0]
    right_obs_states_old = [1.3764188289642334, -0.5266491770744324, -2.042018175125122, -0.25265806913375854, 0.39488810300827026, 1.015584111213684, 2.192312002182007]
    left_obs_states_old = [-1.2257874011993408, -0.4985324442386627, 1.980921745300293, -0.058894701302051544, -0.47677695751190186, 0.9440299868583679, -2.054021120071411]
    left_obs_states = [-0.720012903213501, -0.3579239249229431, 1.772327184677124, 
                       0.13136780261993408, -0.648350715637207, 1.1232486963272095, 
                       -1.8344221115112305]
    right_obs_states = [0.8814099431037903, -0.36250463128089905, -1.85231351852417, 
                        -0.028060272336006165, 0.606028139591217, 1.1636393070220947, 
                        1.9661468267440796]
    left_grasp_states = [-0.6087591438289535, -1.082181434487124, 1.8366249441652813, 
                         0.5307080841282654, 2.619130781460849, 0.30367070501132487, 
                         2.0107517372984773] 
    right_grasp_states = [0.6849933284549622, -1.0505161162309455, -1.9075465218907455, 
                          0.48255300503453435, 0.3831120013800615, -0.28755738199385494, 
                          1.3071448436156516]
    left_grasp_states2 = [-1.0771714448928833, -0.7815413475036621, 1.3724327087402344,
                            0.3715556263923645, 1.7982889413833618, 0.8510035872459412,
                            2.2453041076660156]
    right_grasp_states2 = [1.0602233409881592, -0.6867497563362122, -1.4237779378890991, 
                           0.3548184633255005, 4.543345928192139, 0.7604496479034424, 
                           -2.2329251766204834]

    command_pub = rospy.Publisher('/yumi/egm/joint_group_position_controller/command', Float64MultiArray, queue_size=10)
    left_gripper_pos_pub = rospy.Publisher('/yumi/gripper_l_position_cmd', Float64, queue_size=10)
    right_gripper_pos_pub = rospy.Publisher('/yumi/gripper_r_position_cmd', Float64, queue_size=10)
    left_gripper_cmd_pub = rospy.Publisher('/yumi/gripper_l_effort_cmd', Float64, queue_size=10)
    right_gripper_cmd_pub = rospy.Publisher('/yumi/gripper_r_effort_cmd', Float64, queue_size=10)

    def __init__(self):
        ### Prepare the robot for teleoperation
        self.start_egm_joint_client = rospy.ServiceProxy('/yumi/rws/sm_addin/start_egm_joint', TriggerWithResultCode)
        self.stop_egm_joint_client = rospy.ServiceProxy('/yumi/rws/sm_addin/stop_egm', TriggerWithResultCode)
        self.switch_controller_client = rospy.ServiceProxy('/yumi/egm/controller_manager/switch_controller', SwitchController)
        self.get_egm_settings = rospy.ServiceProxy("/yumi/rws/sm_addin/get_egm_settings", GetEGMSettings)
        self.set_egm_settings = rospy.ServiceProxy("/yumi/rws/sm_addin/set_egm_settings", SetEGMSettings)
        rospy.Subscriber('/yumi/egm/joint_states', JointState, self.yumi_states_sb, queue_size=1)
        rospy.Subscriber('/yumi/egm/egm_states', EGMState, self.egm_state_cb)
        rospy.Subscriber('/yumi/gripper_states', JointState, self.yumi_gripper_states_sb, queue_size=1)

        # get initial states
        states = rospy.wait_for_message('/yumi/egm/joint_states', JointState)
        self.left_arm_state = states.position[:7]
        self.right_arm_state = states.position[7:14]
        self.left_velocities = states.velocity[:7]
        self.right_velocities = states.velocity[7:14]
        gripper_states = rospy.wait_for_message('/yumi/gripper_states', JointState)
        self.left_gripper_state = gripper_states.position[0]*1000
        self.right_gripper_state = gripper_states.position[1]*1000
        rospy.sleep(0.5)
        print("YuMi wrapper initialized")

    def start_egm(self):
        response = self.start_egm_joint_client()
        if response.result_code == 1:
            print("EGM started")
        else:
            print("Failed to start EGM (code: {}, message: {})".format(response.result_code, response.message))
        rospy.sleep(0.5)
    
    def stop_egm(self):
        response = self.stop_egm_joint_client()
        if response.result_code == 1:
            print("EGM stopped")
        else:
            print("Failed to stop EGM (code: {}, message: {})".format(response.result_code, response.message))
        rospy.sleep(0.5)


    def egm_state_cb(self,msg):
        if not self.keep_egm_alive:
            return
        # check if EGM is off
        if msg.egm_channels[0].active == False:
            self.restarting = True
            rospy.loginfo('EGM is off. Restarting...')
            # call the service to start the EGM
            response = self.start_egm_joint_client()
            if response.result_code == 1:
                rospy.loginfo('EGM restarted successfully.')
            else:
                rospy.logerr('Failed to restart EGM (code: {}, message: {})'.format(
                    response.result_code, response.message))
            self.restarting = False

    def start_controllers(self):
        request = SwitchControllerRequest(start_controllers=['joint_group_position_controller'], 
                                        stop_controllers=[''], 
                                        strictness=1, 
                                        start_asap=False, 
                                        timeout=0.0)
        response = self.switch_controller_client(request)
        if response.ok:
            print("Controller started")
        else:
            print("Failed to restart Controllers (code: {}, message: {})".format(response.result_code, response.message))
        rospy.sleep(0.5)

    def change_max_speed(self, task, max_speed_deviation):
        # reset max velocity
        current_settings = self.get_egm_settings(task=task)
        settings = current_settings.settings
        settings.activate.max_speed_deviation = max_speed_deviation # degs
        settings.run.pos_corr_gain = 1 # 0 for pure velocity control
        response = self.set_egm_settings(task=task, settings=settings)
        if response.result_code == 1:
            print("Max speed deviation set to: ", max_speed_deviation)
        else:
            print("Failed to set max speed deviation (code: {}, message: {})".format(response.result_code, response.message))
        rospy.sleep(0.5)
    
    def _get_eef_pos(self) -> np.ndarray:
        return self.left_gripper_state + self.right_gripper_state

    def _get_gripper_pos(self) -> float:
        return self.left_gripper_state + self.right_gripper_state

    def get_joint_state(self):
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        return list(self.left_arm_state) + list(self.right_arm_state)
    
    def get_joint_vel(self):
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        return self.left_velocities + self.right_velocities

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        controller_cmd = Float64MultiArray()
        controller_cmd.data = joint_state
        self.command_pub.publish(controller_cmd)

    def get_observations(self) -> Dict[str, np.ndarray]:
        joint_pos = self.get_joint_state()
        joint_vel = self.get_joint_vel()
        ee_force = self._get_eef_force()
        ee_pos_quat = self._get_eef_pos()
        ee_vel = self._get_eef_speed()
        gripper_pos = np.array([joint_pos[-1]])
        return {
            "joint_positions": joint_pos,
            "joint_velocities": joint_vel,
            "ee_pos_quat": ee_pos_quat,
            "ee_vel": ee_vel,
            "ee_force": ee_force,
            "gripper_position": gripper_pos,
        }

    
    def yumi_states_sb(self, msg):
        """ saves yumi states into this class """
        self.left_arm_state = msg.position[:7]
        self.right_arm_state = msg.position[7:14]
        self.left_velocities = msg.velocity[:7]
        self.right_velocities = msg.velocity[7:14]

    def yumi_gripper_states_sb(self, msg):
        """ saves yumi gripper states into this class """
        self.left_gripper_state = msg.position[0]*1000
        self.right_gripper_state = msg.position[1]*1000

    ###############
    ### Gripper ###
    ###############

    def wait_for_completion_gripper_l(self, log=True):
        """ pause until left gripper state within tolerance """
        time_start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now()-time_start).to_sec()<self.GRIPPER_TIME_TOLERANCE:
            if self.check_completion_gripper_l():
                return True
            else:
                rospy.sleep(0.01)
        if log:
            rospy.logwarn("Gripper action timed-out. Not able to complete in "+str(self.GRIPPER_TIME_TOLERANCE)+'s')
        return False

    def wait_for_completion_gripper_r(self, log=True):
        """ pause until right gripper state within tolerance """
        time_start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now()-time_start).to_sec()<self.GRIPPER_TIME_TOLERANCE:
            if self.check_completion_gripper_r():
                return True
            else:
                rospy.sleep(0.01)
        if log:
            rospy.logwarn("Gripper action timed-out. Not able to complete in "+str(self.GRIPPER_TIME_TOLERANCE)+'s')
        return False

    def check_completion_gripper_l(self):
        """ check if left gripper commands are within tolerance """
        error = sqrt((self.left_gripper_state-self.left_gripper_target)**2)
        return error<=self.GRIPPER_TOLERANCE

    def check_completion_gripper_r(self):
        """ check if right gripper commands are within tolerance """
        error = sqrt((self.right_gripper_state-self.right_gripper_target)**2)
        return error<=self.GRIPPER_TOLERANCE

    def closeRightGripper(self, wait=True, log=False):
        """ Close the right gripper """
        self.sendRightGripperCmd(10, wait, log)

    def openRightGripper(self, wait=True, log=False):
        """ Open the right gripper """
        self.sendRightGripperCmd(-10, wait, log)

    def closeLeftGripper(self, wait=True, log=False): 
        """ Close the left gripper """
        self.sendLeftGripperCmd(10, wait, log)

    def openLeftGripper(self, wait=True, log=False):
        """ Open the right gripper """
        self.sendLeftGripperCmd(-10, wait, log)

    def sendRightGripperCmd(self, cmd, wait=True, log=False):
        """ Send an effort command to the right gripper """
        noise = random()/100
        msg = Float64()
        msg.data = cmd+noise
        self.right_gripper_target = 25 if cmd<0 else 0
        self.right_gripper_cmd_pub.publish(msg)
        if wait and cmd!=0:
            self.wait_for_completion_gripper_r(log=log)

    def sendLeftGripperCmd(self, cmd, wait=True, log=False):
        """ Send an effort command to the left gripper """
        noise = random()/100
        msg = Float64()
        msg.data = cmd+noise
        self.left_gripper_target = 25 if cmd<0 else 0
        self.left_gripper_cmd_pub.publish(msg)
        if wait and cmd!=0:
            self.wait_for_completion_gripper_l(log=log)

    def setRightGripperPos(self, pos, wait=True, log=False): # pos in mm, opening of one side
        """ Send a position command to the right gripper """
        noise = random()/100
        msg = Float64()
        msg.data = self.right_gripper_target = pos+noise
        self.right_gripper_pos_pub.publish(msg)
        if wait:
            self.wait_for_completion_gripper_r(log=log)

    def setLeftGripperPos(self, pos, wait=True, log=False):
        """ Send a position command to the left gripper """
        noise = random()/100
        msg = Float64()
        msg.data = self.left_gripper_target = pos+noise
        self.left_gripper_pos_pub.publish(msg)
        if wait:
            self.wait_for_completion_gripper_l(log=log)


def main():
    rospy.init_node('yumi_wrapper', anonymous=True)
    yumi = YuMi()
    yumi.stop_egm()

    # slow down the speed of the robot
    yumi.change_max_speed(yumi.left_rapid_task, 10) # 5 degrees per second
    yumi.change_max_speed(yumi.right_rapid_task, 10) # 5 degrees per second

    yumi.start_egm()
    yumi.start_controllers()
    
    # move yumi to the initial position
    yumi.command_joint_state(yumi.left_grasp_states + yumi.right_grasp_states)
    key = input("Press Enter when the movement is done")
    if key == 'q':
        exit()
        
    yumi.stop_egm()
    # slow down the speed of the robot
    yumi.change_max_speed(yumi.left_rapid_task, 30) # 5 degrees per second
    yumi.change_max_speed(yumi.right_rapid_task, 30) # 5 degrees per second

    yumi.start_egm()
    yumi.start_controllers()
    print("YuMi ready for teleoperation")


if __name__ == "__main__":
    main()
