#! /usr/bin python3
import socket
import struct
from time import sleep

from actionlib_msgs import msg

class YumiGripperSocketClient:
    
    def __init__(self, ip = '192.168.125.1', 
                 motion_port_l = 13000, 
                 motion_port_r = 13001, 
                 state_port = 13002, 
                 timeout = 5):
        # setup the socket connection
        self._motion_socket_l = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._motion_socket_l.settimeout(timeout)
        self._motion_socket_l.connect((ip, motion_port_l))
        self._motion_socket_r = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._motion_socket_r.settimeout(timeout)
        self._motion_socket_r.connect((ip, motion_port_r))

        self._state_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._state_socket.settimeout(timeout)
        self._state_socket.connect((ip, state_port))

        # self.port_lock = False

    def send_motions_l(self, value, mode=0):
        """ send gripper effort/position commands through the websocket """
        # while self.port_lock:
        #     sleep(0.01)
        # self.port_lock = True
        # send the message length first
        msg_length = 20
        self._motion_socket_l.send(struct.pack("I", msg_length))
        sleep(0.001)
        # int msg_type 10; int comm_type 1; int reply_code 0; int sequence_id 0; float * 7;
        # int 4 bytes; float 4 bytes  
        msg_type = 8008
        comm_type = 1
        reply_code = 0
        byte_array = []
        byte_array.append(struct.pack("i", msg_type))
        byte_array.append(struct.pack("i", comm_type))
        byte_array.append(struct.pack("i", reply_code))
        byte_array.append(struct.pack("f", value))
        byte_array.append(struct.pack("f", mode)) # EFFORT := 0; POSITION := 1
        req = b"".join(byte_array)
        self._motion_socket_l.send(req)
        # self.port_lock = False

    def send_motions_r(self, value, mode=0):
        """ send gripper effort/position commands through the websocket """
        # while self.port_lock:
        #     sleep(0.01)
        # self.port_lock = True
        # send the message length first
        msg_length = 20
        self._motion_socket_r.send(struct.pack("I", msg_length))
        sleep(0.001)
        # int msg_type 10; int comm_type 1; int reply_code 0; int sequence_id 0; float * 7;
        # int 4 bytes; float 4 bytes  
        msg_type = 8008
        comm_type = 1
        reply_code = 0
        byte_array = []
        byte_array.append(struct.pack("i", msg_type))
        byte_array.append(struct.pack("i", comm_type))
        byte_array.append(struct.pack("i", reply_code))
        byte_array.append(struct.pack("f", value))
        byte_array.append(struct.pack("f", mode)) # EFFORT := 0; POSITION := 1
        req = b"".join(byte_array)
        self._motion_socket_r.send(req)
        # self.port_lock = False

    def receive_states(self):
        """ receive gripper states from the websocket """
        # bytes = self._state_socket.recv(1024)
        # if bytes != 28:
        #     print('Empty state message received.')
        #     return
        try:
            msg_len = 0 
            while msg_len != 28:
                bytes = self._state_socket.recv(1024)
                msg_len = len(bytes)
            return self.unpack_bytes(bytes)
        except:
            print('Time out occured (grippers).')
            return None

    @staticmethod
    def unpack_bytes(bytes):
        """ transform bytes into lists """ 
        # packet_length = struct.unpack_from("I", bytes, 0) # this byte not included
        # msg_type = struct.unpack_from("i", bytes, 4)
        # comm_type = struct.unpack_from("i", bytes, 8)
        # reply_code = struct.unpack_from("i", bytes, 12)
        # sequence_id = struct.unpack_from("i", bytes, 16)
        gripper_l = struct.unpack_from("f", bytes, 20)[0]
        gripper_r = struct.unpack_from("f", bytes, 24)[0]
        return gripper_l, gripper_r

if __name__ == '__main__':
    # yumi_socket_client = YumiGripperSocketClient(ip='localhost')
    yumi_socket_client = YumiGripperSocketClient()
    # yumi_socket_client.send_motions(5, 5, mode=0)
    # yumi_socket_client._motion_socket.close()
    # yumi_socket_client._state_socket.close()
    try:
        while True:
            gripper_l, gripper_r = yumi_socket_client.receive_states()
            yumi_socket_client.send_motions_l(5, mode=0)
            yumi_socket_client.send_motions_r(5, mode=0)
    except KeyboardInterrupt:
        print('Interruption received. Terminating program.')
        yumi_socket_client._motion_socket_l.close()
        yumi_socket_client._motion_socket_r.close()
        yumi_socket_client._state_socket.close()
    