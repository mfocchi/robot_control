# Description
# File contains some necessary control algorithms for HyQ
# Author: Michele Focchi
# Date: 23-10-2022
import rospkg
import socket
import numpy as np
from base_controllers.components.filter import SecondOrderFilter
import rospy as ros
from std_srvs.srv import Trigger, TriggerRequest


class GripperManager():
    def __init__(self, real_robot_flag = False, dt = 0.001, gripping_duration = 5.):
        self.q_des_gripper = np.array([1.8, 1.8,1.8])
        self.gripping_duration = gripping_duration
        self.real_robot = real_robot_flag
        self.SO_filter = SecondOrderFilter(3)
        self.SO_filter.initFilter(self.q_des_gripper,dt)

    def resend_robot_program(self):
        ros.sleep(1.5)
        ros.wait_for_service("/ur5/ur_hardware_interface/resend_robot_program")
        sos_service = ros.ServiceProxy('/ur5/ur_hardware_interface/resend_robot_program', Trigger)
        sos = TriggerRequest()
        result = sos_service(sos)
        # print(result)
        ros.sleep(0.1)

    def mapToGripperJoints(self, diameter):
        return (diameter - 22) / (130 - 22) * (-np.pi) + np.pi  # D = 130-> q = 0, D = 22 -> q = 3.14

    def getDesGripperJoints(self):
        return self.SO_filter.filter(self.q_des_gripper, self.gripping_duration)

    def move_gripper(self, diameter):
        if not self.real_robot:
            q_finger = self.mapToGripperJoints(diameter)
            self.q_des_gripper = np.array([q_finger, q_finger, q_finger])
            return

        import socket

        HOST = "192.168.0.100"  # The UR IP address
        PORT = 30002  # UR secondary client
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        sock.settimeout(0.5)
        try:
            sock.connect((HOST, PORT))
        except:
            raise Exception("Cannot connect to end-effector socket") from None
        sock.settimeout(None)
        scripts_path = rospkg.RosPack().get_path('ur_description') + '/gripper/scripts'

        onrobot_script = scripts_path + "/onrobot_superminimal.script"
        file = open(onrobot_script, "rb")  # Robotiq Gripper
        lines = file.readlines()
        file.close()

        tool_index = 0
        blocking = True
        cmd_string = f"tfg_release({diameter},  tool_index={tool_index}, blocking={blocking})"

        line_number_to_add = 446

        new_lines = lines[0:line_number_to_add]
        new_lines.insert(line_number_to_add + 1, str.encode(cmd_string))
        new_lines += lines[line_number_to_add::]

        offset = 0
        buffer = 2024
        file_to_send = b''.join(new_lines)

        if len(file_to_send) < buffer:
            buffer = len(file_to_send)
        data = file_to_send[0:buffer]
        while data:
            sock.send(data)
            offset += buffer
            if len(file_to_send) < offset + buffer:
                buffer = len(file_to_send) - offset
            data = file_to_send[offset:offset + buffer]
        sock.close()

        print("Gripper moved, now resend robot program")
        self.resend_robot_program()
        return

