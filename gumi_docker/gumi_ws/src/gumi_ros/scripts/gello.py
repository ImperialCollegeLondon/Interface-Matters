import numpy as np
from dynamixel_robot import DynamixelRobot

class GELLO:
    left_port = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISMQ8-if00-port0"
    right_port = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0RL1-if00-port0"

    def __init__(self, left_init_states, right_init_states):
        self.yumi_left = DynamixelRobot(
            joint_ids=(1, 2, 3, 4, 5, 6, 7),
            joint_offsets=(2*np.pi/2, -1*np.pi/2, 0*np.pi/2, 1*np.pi/2, 6*np.pi/2, 3*np.pi/2, 0*np.pi/2),
            joint_signs=(1, -1, 1, -1, 1, -1, 1),
            real=True,
            port=self.left_port,
            baudrate=57600,
            gripper_config=(8, 199.751171875, 157.951171875),
            start_joints=np.array(left_init_states+[0])
        )
        self.yumi_right = DynamixelRobot(
            joint_ids=(1, 2, 3, 4, 5, 6, 7),
            joint_offsets=(4*np.pi/2, 2*np.pi/2, 2*np.pi/2, 1*np.pi/2, 5*np.pi/2, -1*np.pi/2, 1*np.pi/2),
            joint_signs=(1, -1, 1, -1, 1, -1, 1),
            real=True,
            port=self.right_port,
            baudrate=57600,
            gripper_config=(8, 14.477734375, -27.322265625),
            start_joints=np.array(right_init_states+[0])
        )
    print("Dynamixel connected")
    
    def get_left_states(self):
        return self.yumi_left.get_observations()["joint_state"][:7].tolist(), self.yumi_left.get_observations()["joint_state"][7]
    
    def get_right_states(self):
        return self.yumi_right.get_observations()["joint_state"][:7].tolist(), self.yumi_right.get_observations()["joint_state"][7]
    
if __name__ == "__main__":
    left_states = [0, -2.2689, 2.3562, 0.5236, 0, 0.6981, 0]
    right_states = [0, -2.2689, -2.3562, 0.5236, 0, 0.6981, 0]
    gello = GELLO(left_states, right_states)
    print(gello.get_left_states())