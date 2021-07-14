import cv2
import numpy as np
import time
import math
from pathlib import Path
from time import time, sleep
import sys
from service.value_tracker import Value_tracker

#from IPython.display import clear_output

#URL-requests to the robot
import requests

#speech generation
import os.path
import cyrtranslit
from gtts import gTTS

paths = {"kompaso" : {"model_path"   : "pose_estimation/human-pose-estimation-3d.pth",
                      "phrases_path" : "data/sounds/phrases.txt",
                      "vision_path"  : "robotics_course/modules/",
                      "robot_ip"     : "192.168.1.66"},

         "elijah"  : {"model_path"   : "/Users/elijah/Dropbox/Programming/RoboCup/remote control/source/test/human-pose-estimation-3d.pth",
                      "phrases_path" : "/Users/elijah/Dropbox/Programming/RoboCup/remote control/data/sounds/phrases.txt",
                      "vision_path"  : "/Users/elijah/Dropbox/Programming/robotics_course/modules/",
                      "robot_ip"     : "192.168.1.9"}}

def get_available_cameras(upper_bound=10, lower_bound=0):
    available = []

    for i in range(lower_bound, upper_bound):
        cap = cv2.VideoCapture(i)

        if (cap.isOpened()):
            available.append(i)

        cap.release()

    return available

class Timeout_module:
    def __init__ (self, timeout_):
        self.curr_time = 0
        self.last_action_time = 0
        self.timeout = timeout_

        self._update ()

    def _update (self):
        self.curr_time = time ()

    def _update_last_action_time (self, new_last_action_time = -1):
        if (new_last_action_time == -1):
            self.last_action_time = self.curr_time

        else:
            self.last_action_time = new_last_action_time

    def timeout_passed (self, additional_condition = True, print_time = False):
        self._update ()

        time_from_last_action = self.curr_time - self.last_action_time

        if (print_time == True):
            print ("time from last command: ", time_from_last_action)

        if (time_from_last_action > self.timeout and additional_condition == True):
            self._update_last_action_time ()
            return True

        else:
            return False

def rus_line_to_eng (line):
    out = cyrtranslit.to_latin (line, 'ru')
    out = "".join (c for c in out if c not in ['!', '.', '#', ':', "'", '?', ' ', '-', '\'', ',', '\n'])
    return out


def create_vec(dot2, dot1):
    vec = np.asarray([dot2[0] - dot1[0],dot2[1] - dot1[1],dot2[2] - dot1[2]])
    denum = math.sqrt(sum(np.power(vec,2)))
    return np.array([round(vec[0]/denum, 2), round(vec[1]/denum, 2), round(vec[2]/denum, 2)])

def get_mod(vec):
    return math.sqrt(sum(np.power(vec,2)))

def angle_2_vec_ (x1, y1, x2, y2):
    dot = x1*x2 + y1*y2
    det = math.sqrt(x1**2 + y1**2)*math.sqrt(x2**2 + y2**2)

    while (abs (dot) > det):
        dot *= 0.999

    angle = math.acos(dot/det)

    return angle

def angle_2_vec (vec1, vec2):
    return angle_2_vec_ (vec1 [0], vec1 [1], vec2 [0], vec2 [1])

def head_rotation (x_pose):
    if round(x_pose,1) <= 0.3:
        return 0.84
    elif x_pose >= 0.7:
        return -0.84
    else:
        return 0
