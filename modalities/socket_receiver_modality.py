from modalities.modality import  Modality

import numpy as np
from common import *

class Socket_receiver (Modality):
    def __init__ (self, port_, logger_ = 0):
        self.logger = logger_
        self.port = port_
        # self.commands = []

        context = zmq.Context ()
        self.socket_receiver = context.socket (zmq.PAIR)
        self.socket_receiver.bind ("tcp://127.0.0.1:%s" % self.port)

    def __del__ (self):
        self.socket_receiver.close ()

    def name(self):
        return "Socket_receiver"

    def _read_data (self):
        self.read_data = self.socket_receiver.recv ()

    def _process_data(self):
        raw_commands = self.read_data
        angleList = []
        namesList = []
        animations = []
        raw_lines = raw_commands.replace("]","").replace("[","").replace(", (", "").replace("(","").split(")")
        for line in raw_lines:
            if line[1:17] == "/set_joint_angle":
                name = line[21: 21+(line[21:].index("'"))]
                if name != 'mode':
                    robot_joint = synchronized_joints [name]
                    namesList.append(robot_joint)
                    angle = float(line[25+(line[21:].index("'")):-1])
                    angleList.append([angle])
        return [angleList, namesList]

    def _interpret_data(self):
        pass

    def _get_command(self):
        return self._process_data()

    def get_command (self, skip_reading_data=False):
        self._read_data()
        self._process_data()
        self._interpret_data()

        return self._get_command()
