# -*- coding: utf-8 -*-

from modalities.modality import  Modality
import numpy as np
import common

import io
import pydub
from pydub import AudioSegment
from pydub.playback import play
import scipy.fftpack
import cv2

class Computer_keyboard (Modality):
    def __init__ (self, phrases_path = "", key_to_command_ = {"z" : "empty"}, logger_ = 0):
        self.logger = logger_

        self.read_data        = 0x00
        self.processed_data   = 0x00
        self.interpreted_data = 0x00

        self.curr_mode = 0

        self.all_keys = ["w", "e", "r", "t", "y", "u", "i", "o", "p", "a", "s", "d", "f", "g", "h", "j", "k", "l", "v", "b", "m", "z", "x", "c", "n"]

        self.common_commands = {"z"        : [("Stand",    ["heh"])],
                                "c"        : [("Rest",     ["kek"])],
                                #"n"        : [("next",      [""])],
                                "x"        : [("Sit",      [""])],
                                "noaction" : [("noaction",  [""])],
                                "w"        : [("/play_mp3", ["Molodec.mp3"])],
                                "e"        : [("/play_mp3", ["Otlichnopoluchaetsja.mp3"])],
                                "r"        : [("/play_mp3", ["Zdorovo.mp3"])],
                                "t"        : [("/play_mp3", ["Zamechatelno.mp3"])],
                                "y"        : [("/play_mp3", ["Poluchilos.mp3"])],
                                "u"        : [("/play_mp3", ["Prekrasno.mp3"])],
                                "i"        : [("/play_mp3", ["Molodec.mp3"])],
                                "o"        : [("/play_mp3", ["Molodec.mp3"])],
                                "p"        : [("/play_mp3", ["Horosho.mp3"])],}

        self.repeating =   {"z"        : [("/stand",    ["heh"])],
                            "c"        : [("/rest",     ["kek"])],
                            "x"        : [("Sit",      [""])],
                            "noaction" : [("noaction",  [""])],
                            "w"        : [("/play_mp3", ["Molodec.mp3"])],
                            "e"        : [("/play_mp3", ["Povtorjajzamnoj.mp3"])],
                            "r"        : [("/play_mp3", ["Zdorovo.mp3"])],
                            "o"        : [("/bend_right", [""])],
                            "p"        : [("/bend_left", [""])],
                            "v"        : [("/hands_sides", [""])],
                            "b"        : [("/hands_front", [""])],
                            "n"        : [("/left_shoulder_up", [""])],
                            "m"        : [("/right_shoulder_up", [""])],

                            "l"        : [("/play_mp3", ["Poprobujeszeraz.mp3"])],

                            "f"        : [("/play_mp3", ["Posmotrinapravo.mp3"]),
                                          ("/play_airplane_1", [""])],
                            "g"        : [("/play_mp3", ["Posmotrinalevo.mp3"]),
                                          ("/play_airplane_2", [""])],
                            "h"        : [("/play_mp3", ["Posmotrivverh.mp3"]),
                                          ("/play_car", [""])],
                            "j"        : [("/left_hand_left", ["Horosho.mp3"])],
                            "k"        : [("/right_hand_right", ["Horosho.mp3"])],

                            "w"        : [("/walk_20", [""])],
                            "a"        : [("/rot_20", [""])],
                            "s"        : [("/rot_m20", [""])],
                            "d"        : [("/walk_m30", [""])],
                            "noaction" : [("noaction",  [""])]}

        self.repeating2 =  {"z"        : [("/stand",    ["heh"])],
                            "c"        : [("/rest",     ["kek"])],
                            "x"        : [("/sit",      [""])],
                            "noaction" : [("noaction",  [""])],
                            "w"        : [("/play_mp3", ["Molodec.mp3"])],
                            "e"        : [("/play_mp3", ["Povtorjajzamnoj.mp3"])],
                            "r"        : [("/play_mp3", ["Zdorovo.mp3"])],
                            "o"        : [("/right_hand_right", [""])],
                            "p"        : [("/right_hand_front", [""])],
                            "v"        : [("/left_hand_left", [""])],
                            "b"        : [("/left_hand_front", [""])],
                            "n"        : [("/right_hand_up", [""])],
                            "m"        : [("/left_hand_up", [""])],

                            "l"        : [("/play_mp3", ["Poprobujeszeraz.mp3"])],

                            "f"        : [("/play_mp3", ["Dajpjat.mp3"]),
                                          ("/right_hand_front", [""])],
                            "g"        : [("/play_mp3", ["Posmotrinalevo.mp3"]),
                                          ("/play_airplane_2", [""])],
                            "h"        : [("/play_mp3", ["Posmotrivverh.mp3"]),
                                          ("/play_car", [""])],
                            "j"        : [("/left_hand_left", ["Horosho.mp3"])],
                            "k"        : [("/right_hand_right", ["Horosho.mp3"])],

                            "w"        : [("/walk_20", [""])],
                            "a"        : [("/rot_20", [""])],
                            "s"        : [("/rot_m20", [""])],
                            "d"        : [("/walk_m30", [""])],
                            "noaction" : [("noaction",  [""])]}

        self.exceptional = {"z"        : [("/stand",    ["heh"])],
                                "c"        : [("/rest",     ["kek"])],
                                #"n"        : [("next",      [""])],
                                "x"        : [("/sit",      [""])],
                                "noaction" : [("noaction",  [""])],
                                "w"        : [("/play_mp3", ["Molodec.mp3"])],
                                "e"        : [("/play_mp3", ["Otlichnopoluchaetsja.mp3"])],
                                "r"        : [("/play_mp3", ["Zdorovo.mp3"])],
                                "t"        : [("/play_mp3", ["Zamechatelno.mp3"])],
                                "y"        : [("/play_mp3", ["Poluchilos.mp3"])],
                                "u"        : [("/play_mp3", ["Prekrasno.mp3"])],
                                "i"        : [("/play_mp3", ["Molodec.mp3"])],
                                "o"        : [("/play_mp3", ["Molodec.mp3"])],
                                "p"        : [("/play_mp3", ["Horosho.mp3"])],
                            "a" : [("/play_mp3", ["am_nyam.mp3"])],
                            "s" : [("/play_mp3", ["Privet.mp3"])],
                            "d" : [("/play_mp3", ["Privetik.mp3"])],
                            "f" : [("/play_mp3", ["Pokauvidimsja.mp3"])],
                            "g" : [("/play_mp3", ["Najdinakakujufigurupohozhe.mp3"])],
                            "h" : [("/stop", [""])],
                            "j" : [("/play_mp3", ["am_nyam.mp3"])],
                            "k" : [("/play_mp3", ["am_nyam.mp3"])],
                            "l" : [("/play_mp3", ["am_nyam.mp3"])],

                            "m"        : [("/play_mp3", ["Poprobujeszeraz.mp3"])],

                            "noaction" : [("noaction",  [""])]}

        self.eyes = {"z" : [("/stand",    ["heh"])],
                     "c" : [("/rest",     ["kek"])],
                     "x" : [("/sit",      [""])],
                     "w" : [("/play_mp3", ["Molodec.mp3"])],
                     "e" : [("/play_mp3", ["Otlichnopoluchaetsja.mp3"])],
                     "r" : [("/play_mp3", ["Zdorovo.mp3"])],
                     "t" : [("/play_mp3", ["Zamechatelno.mp3"])],
                     "y" : [("/play_mp3", ["Poluchilos.mp3"])],
                     "u" : [("/play_mp3", ["Prekrasno.mp3"])],
                     "i" : [("/play_mp3", ["Molodec.mp3"])],
                     "o" : [("/play_mp3", ["Molodec.mp3"])],
                     "p" : [("/play_mp3", ["Horosho.mp3"])],
                     "a" : [("/red",    [""])],
                     "s" : [("/green",  [""])],
                     "d" : [("/blue",   [""])],
                     "f" : [("/orange", [""])],
                     "g" : [("/yellow", [""])],
                     "h" : [("/white",  [""])],
                     "j" : [("/lightblue", [""])],
                     "k" : [("/violet", [""])],
                     "l" : [("/play_mp3", ["Posmotrinaglazainajdikarti.mp3"])],

                     "n"        : [("/play_mp3", ["Poprobujeszeraz.mp3"])],

                     "noaction" : [("noaction",  [""])]}

        self.direct_control =  {"z"        : [("/Stand",   ["heh"])],
                                "x"        : [("/Rest",    ["kek"])],
                                "w"        : [("/increment_joint_angle", ["l_sho_roll", "0.21"])],
                                "e"        : [("/increment_joint_angle", ["l_sho_roll", "-0.21"])],
                                "r"        : [("/increment_joint_angle", ["l_elb_roll", "0.21"])],
                                "t"        : [("/increment_joint_angle", ["l_elb_roll", "-0.21"])],
                                "y"        : [("/increment_joint_angle", ["l_sho_pitch", "0.21"])],
                                "u"        : [("/increment_joint_angle", ["l_sho_pitch", "-0.21"])],
                                "i"        : [("/increment_joint_angle", ["l_elb_yaw", "0.21"])],
                                "o"        : [("/increment_joint_angle", ["l_elb_yaw", "-0.21"])],

                                "s"        : [("/increment_joint_angle", ["r_sho_roll", "0.21"])],
                                "d"        : [("/increment_joint_angle", ["r_sho_roll", "-0.21"])],
                                "f"        : [("/increment_joint_angle", ["r_elb_roll", "0.21"])],
                                "g"        : [("/increment_joint_angle", ["r_elb_roll", "-0.21"])],
                                "h"        : [("/increment_joint_angle", ["r_sho_pitch", "0.21"])],
                                "j"        : [("/increment_joint_angle", ["r_sho_pitch", "-0.21"])],
                                "k"        : [("/increment_joint_angle", ["r_sho_yaw", "0.21"])],
                                "l"        : [("/increment_joint_angle", ["r_sho_yaw", "-0.21"])],


                                "c"        : [("/increment_joint_angle", ["head_Yaw", "0.1"])],
                                "v"        : [("/increment_joint_angle", ["head_Yaw", "-0.1"])],
                                "b"        : [("/increment_joint_angle", ["head_Pitch", "0.1"])],
                                "n"        : [("/increment_joint_angle", ["head_Pitch", "-0.1"])],


                                "n"        : [("next",     [""])],
                                "noaction" : [("noaction", [""])]}


        self.direct_control_1 =  {"z"        : [("/stand",   ["heh"])],
                                "x"        : [("/rest",    ["kek"])],
                                "w"        : [("/increment_joint_angle", ["l_hip_roll", "0.1"])],
                                "e"        : [("/increment_joint_angle", ["l_hip_roll", "-0.1"])],
                                "r"        : [("/increment_joint_angle", ["l_hip_pitch", "0.1"])],
                                "t"        : [("/increment_joint_angle", ["l_hip_pitch", "-0.1"])],
                                "y"        : [("/increment_joint_angle", ["l_knee_pitch", "0.1"])],
                                "u"        : [("/increment_joint_angle", ["l_knee_pitch", "-0.1"])],
                                "i"        : [("/increment_joint_angle", ["l_ank_pitch", "0.1"])],
                                "o"        : [("/increment_joint_angle", ["l_ank_pitch", "-0.1"])],
                                "p"        : [("/increment_joint_angle", ["l_ank_roll", "0.1"])],
                                "["        : [("/increment_joint_angle", ["l_ank_roll", "-0.1"])],


                                "s"        : [("/increment_joint_angle", ["r_hip_roll", "0.05"])],
                                "d"        : [("/increment_joint_angle", ["r_hip_roll", "-0.05"])],
                                "f"        : [("/increment_joint_angle", ["r_hip_pitch", "0.05"])],
                                "g"        : [("/increment_joint_angle", ["r_hip_pitch", "-0.05"])],
                                "h"        : [("/increment_joint_angle", ["r_knee_pitch", "0.05"])],
                                "j"        : [("/increment_joint_angle", ["r_knee_pitch", "-0.05"])],
                                "k"        : [("/increment_joint_angle", ["r_ank_pitch", "0.05"])],
                                "l"        : [("/increment_joint_angle", ["r_ank_pitch", "-0.05"])],
                                ":"        : [("/increment_joint_angle", ["r_ank_roll", "0.05"])],
                                "]"        : [("/increment_joint_angle", ["r_ank_roll", "-0.05"])],


                                "c"        : [("/increment_joint_angle", ["hehe_pelvis", "0.05"])],
                                "v"        : [("/increment_joint_angle", ["hehe_pelvis", "-0.05"])],



                                "n"        : [("next",     [""])],
                                "noaction" : [("noaction", [""])]}

        # self.direct_control_2 =  {"z"      : [("/stand",   ["heh"])],
        #                         "x"        : [("/rest",    ["kek"])],
        #
        #                         "w"        : [("/increment_joint_angle", ["l_hip_roll", "-0.069813"]), ("/increment_joint_angle", ["r_hip_roll", "0.069813"]),
        #                                       ("/increment_joint_angle", ["l_hip_pitch", "-0.6806784"]), ("/increment_joint_angle", ["r_hip_pitch", "-0.698132"]),
        #                                       ("/increment_joint_angle", ["l_knee_pitch", "2.0"]), ("/increment_joint_angle", ["r_knee_pitch", "2.0"]),
        #                                       ("/increment_joint_angle", ["l_ank_pitch", "-1.17"]), ("/increment_joint_angle", ["r_ank_pitch", "-1.17"]),
        #                                       ("/increment_joint_angle", ["l_ank_roll", "0.0698132"]), ("/increment_joint_angle", ["r_ank_roll", "-0.0698132"]),
        #                                       ("/increment_joint_angle", ["hehe_pelvis", "-0.2443"])],
        #
        #                         "e"        : [("/increment_joint_angle", ["l_hip_roll", "-0"]), ("/increment_joint_angle", ["r_hip_roll", "0"]),
        #                                       ("/increment_joint_angle", ["l_hip_pitch", "-0.2792527"]), ("/increment_joint_angle", ["r_hip_pitch", "-0.27925268"]),
        #                                       ("/increment_joint_angle", ["l_knee_pitch", "0.9948377"]), ("/increment_joint_angle", ["r_knee_pitch", "0.9948377"]),
        #                                       ("/increment_joint_angle", ["l_ank_pitch", "-0.5410521"]), ("/increment_joint_angle", ["r_ank_pitch", "-0.54105207"]),
        #                                       ("/increment_joint_angle", ["l_ank_roll", "-0.0174533"]), ("/increment_joint_angle", ["r_ank_roll", "0"]),
        #                                       ("/increment_joint_angle", ["hehe_pelvis", "-0.209439"])],
        #
        #                         "r"        : [("/increment_joint_angle", ["l_hip_roll", "0.10472"]), ("/increment_joint_angle", ["r_hip_roll", "-0.10472"]),
        #                                       ("/increment_joint_angle", ["l_hip_pitch", "0.122173"]), ("/increment_joint_angle", ["r_hip_pitch", "0.10472"]),
        #                                       ("/increment_joint_angle", ["l_knee_pitch", "-0.0872665"]), ("/increment_joint_angle", ["r_knee_pitch", "-0.0872665"]),
        #                                       ("/increment_joint_angle", ["l_ank_pitch", "0.0698132"]), ("/increment_joint_angle", ["r_ank_pitch", "0.0872665"]),
        #                                       ("/increment_joint_angle", ["l_ank_roll", "-0.10472"]), ("/increment_joint_angle", ["r_ank_roll", "0.10472"]),
        #                                       ("/increment_joint_angle", ["hehe_pelvis", "-0.1745329"])],
        #
        #
        #                         "s"        : [("/increment_joint_angle", ["l_hip_pitch", "0.1"]), ("/increment_joint_angle", ["r_hip_pitch", "0.1"]),
        #                                       ("/increment_joint_angle", ["l_knee_pitch", "-0.2"]), ("/increment_joint_angle", ["r_knee_pitch", "-0.2"]),
        #                                       ("/increment_joint_angle", ["l_ank_pitch", "0.1"]), ("/increment_joint_angle", ["r_ank_pitch", "0.1"])],
        #
        #                         "c"        : [("/increment_joint_angle", ["hehe_pelvis", "0.05"])],
        #                         "v"        : [("/increment_joint_angle", ["hehe_pelvis", "-0.05"])],
        #
        #
        #                         "noaction" : [("noaction", [""])]}

        self.direct_control_3 =  {"z"        : [("/stand",   ["heh"])],
                                "x"        : [("/rest",    ["kek"])],
                                "b"        : [("/set_joint_angle", ["l_sho_roll", "0.5"])],
                                "e"        : [("/set_joint_angle", ["mode", "0.0"]), ("/set_joint_angle", ["l_sho_pitch", "1.1"]), ("/set_joint_angle", ["r_sho_pitch", "1.1"])],
                                "w"        : [("/set_joint_angle", ["mode", "1.0"]), ("/set_joint_angle", ["l_sho_pitch", "1.1"]), ("/set_joint_angle", ["r_sho_pitch", "1.1"])],
                                "r"        : [("/set_joint_angle", ["mode", "2.0"]), ("/set_joint_angle", ["l_sho_pitch", "1.1"]), ("/set_joint_angle", ["r_sho_pitch", "1.1"])],
                                # "r"        : [("/increment_joint_angle", ["l_elb_roll", "0.21"])],
                                "n"        : [("next",     [""])],
                                "noaction" : [("noaction", [""])]}


        self.key_to_command = []

        self.key_to_command.append (self.direct_control)
        self.key_to_command.append (self.direct_control_3)
        self.key_to_command.append (self.exceptional)
        self.key_to_command.append (self.repeating)
        self.key_to_command.append (self.repeating2)
        self.key_to_command.append (self.eyes)

        #print "key to command", len (self.key_to_command)

        if (phrases_path == ""):
            f = io.open (phrases_path, "r", encoding='utf-8')
            f1 = f.readlines()

            available_keys = [x for x in self.all_keys if x not in self.common_commands.keys ()]

            phrase_name = []

            for line in f1:
                out = common.rus_line_to_eng (line)
                filename = out [:26] + '.mp3'

                phrase_name.append ((line, filename))

            #print (phrase_name)

            new_list = self.common_commands.copy ()

            while (len (phrase_name) != 0):
                for key in available_keys:
                    if (len (phrase_name) == 0):
                        break

                    el = phrase_name [0]

                    new_list.update ({key : [("/play_mp3", [el [1], el [0]])]})

                    phrase_name.pop (0)

                self.key_to_command.append (new_list)
                new_list = self.common_commands.copy ()

        #else:
        #    self.key_to_command = [self.common_commands]

        if (key_to_command_ ["z"] != "empty"):
            self.key_to_command = key_to_command_

    def name (self):
        return "computer keyboard"

    def _read_data (self):
        self.read_data = cv2.waitKey (1)
        #print ("read data", self.read_data)

    def get_read_data (self):
        return self.read_data

    def _process_data (self):
        self.processed_data = self.read_data

    def _interpret_data (self):
        self.interpreted_data = self.processed_data

    def _get_command (self):
        if (self.interpreted_data >= 0):
            # key = str (chr (self.interpreted_data))
            key = (chr (self.interpreted_data)).encode('utf-8')
            #print ("key in get_command", key)

            if (key in self.key_to_command [self.curr_mode].keys ()):
                return self.key_to_command [self.curr_mode] [key]

            if (key in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']):
                self.curr_mode = int (key)# % len (self.key_to_command)

                while (self.curr_mode >= len (self.key_to_command)):
                    self.curr_mode -= len(self.key_to_command)

                print ("curr mode ", self.curr_mode, len(self.key_to_command))

        return self.key_to_command [self.curr_mode] ["noaction"]

    def get_command (self, skip_reading_data = False):
        if (skip_reading_data == False):
            self._read_data ()

        self._process_data   ()
        self._interpret_data ()

        return self._get_command ()

    def draw (self, canvas = np.ones ((700, 700, 3), np.uint8) * 220):
        result = canvas.copy ()

        #if (canvas is None):
        #    result = np.ones ((700, 700, 3), np.uint8) * 220

        cv2.putText (result, "curr mode: " + str (self.curr_mode), (30, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (20, 50, 31), 1, cv2.LINE_AA)

        str_num = 0

        for key in self.key_to_command [self.curr_mode].keys ():
            text = key + str (self.key_to_command [self.curr_mode] [key])

            cv2.putText (result, text, (30, 60 + 20 * str_num),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (20, 50, 31), 1, cv2.LINE_AA)

            str_num += 1

        return [result]
