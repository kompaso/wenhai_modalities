from modalities.modality import  Modality
from modalities.skeleton_modality import  Skeleton

import numpy as np
import common
import torch
from skel_proc import  get_skel_coords
from modules.load_state import load_state
from test.with_mobilenet import PoseEstimationWithMobileNet
from models.with_mobilenet_ import PoseEstimationWithMobileNet_
import io

#from test.demo import draww
from argparse import ArgumentParser
import json
import os
from test.input_reader import VideoReader, ImageReader
from test.draw import Plotter3d, draw_poses
from test.parse_poses import parse_poses
from test.inference_engine_pytorch import InferenceEnginePyTorch

import pydub
from pydub import AudioSegment
from pydub.playback import play
import scipy.fftpack
import cv2

class Markov_chain (Modality):
    def __init__ (self, video_path_ = "", logger_ = 0):
        self.logger = logger_
        self.read_data        = []
        self.interpreted_data = []

        self.timeout = common.Timeout_module (0.2)
        self.tick = 0

        self.commands = {"noaction": [("noaction", [""])],
                         "1": [("/stand", [""])],
                         "2": [("/left_shoulder_up", [""])],
                         "3": [("/right_shoulder_up", [""])],
                         "4": [("/left_shoulder_up", [""])],
                         "5": [("/stand", [""])],
                         "6": [("/left_hand_left", [""])],
                         "7": [("/stand", [""])],
                         "8": [("/right_hand_right", [""])],
                         "9": [("/stand", [""])],
                         "10": [("/bend_right", [""])],
                         "11": [("/bend_left", [""])],
                         "12": [("/stand", [""])],
                         "13": [("/play_airplane_1", [""])],
                         "14": [("/play_airplane_2", [""])],
                         "15": [("/play_airplane_1", [""])],
                         "16": [("/hands_sides", [""])],
                         }

    def name(self):
        return "Markov chain"

    def _read_data (self):
        pass

    def _process_data(self):
        pass

    def _interpret_data(self):
        pass

    def _get_command(self):
        comm = self.commands ["noaction"]

        if (self.timeout.timeout_passed ()):
            l = len (self.commands)

            comm = self.commands[str (self.tick % (l - 1) + 1)]
            self.tick += 1

        #print ("com", comm)

        return comm

    def get_command(self, skip_reading_data=False):
        self._read_data()
        self._process_data()
        self._interpret_data()

        return self._get_command()

    # def draw(self, img):
    #     pass

    def __init__ (self, skeleton_path_ = ""):
        self.read_data        = []
        self.interpreted_data = []

        self.timeout = common.Timeout_module (1)

        self.dataframe_num = 0

        self.commands = {"noaction": [("noaction", [""])],
                         "1": [("/stand", [""])],
                         "2": [("/hands_sides", [""])]}

        self.processed_data = {"righthand": 0,
                               "rightarm": 0,
                               "lefthand": 0,
                               "leftarm": 0}


        if (skeleton_path_ != ""):
            self.all_data = self.read_data_from_file(skeleton_path_)

    def read_data_from_file(self, path):
        with open(path, 'r') as file:
            data = file.read()
            cleared_data = ''
            for let in data:
                if let.isdigit() or let == '-':
                    cleared_data += let
                else:
                    cleared_data += ','

            cleared_data = cleared_data.split(',')
            data = [int(i) for i in cleared_data if i]
            data = np.asarray(data)
            data = data.reshape(-1, 36)

        return data

    def name(self):
        return "response to skeleton"

    def _read_data(self):
        if (self.dataframe_num >= len(self.all_data)):
            read_data = 0
            return

        self.read_data = self.all_data[self.dataframe_num]
        self.dataframe_num += 1

    def get_read_data(self):
        return self.read_data

    def _process_data (self):
        kpt_names = ['nose', 'neck', 'r_sho', 'r_elb', 'r_wri', 'l_sho',
                     'l_elb', 'l_wri', 'r_hip', 'r_knee', 'r_ank', 'l_hip',
                     'l_knee', 'l_ank', 'r_eye', 'l_eye', 'r_ear', 'l_ear']

        necessary_keypoints_names = ["l_sho", "l_elb", "l_wri", "l_hip", "r_sho", "r_elb", "r_wri", "r_hip", "neck"]
        kps = {}

        #print ("kps", kps)

        for kp in necessary_keypoints_names:
            ind = kpt_names.index (kp)
            kps.update ({kp : (self.read_data [ind * 2], self.read_data [ind * 2 + 1])})

        hips_mid  = ((kps ["r_hip"] [0] + kps ["l_hip"] [0]) / 2, (kps ["r_hip"] [1] + kps ["l_hip"] [1]) / 2)
        neck_hip  = (kps ["neck"]  [0] - hips_mid      [0], kps ["neck"]  [1] - hips_mid      [1]) #????????
        sh_r_elb  = (kps ["r_elb"] [0] - kps ["r_sho"] [0], kps ["r_elb"] [1] - kps ["r_sho"] [1])
        sh_l_elb  = (kps ["l_elb"] [0] - kps ["l_sho"] [0], kps ["l_elb"] [1] - kps ["l_sho"] [1])
        elb_r_wri = (kps ["r_wri"] [0] - kps ["r_elb"] [0], kps ["r_wri"] [1] - kps ["r_elb"] [1])
        elb_l_wri = (kps ["l_wri"] [0] - kps ["l_elb"] [0], kps ["l_wri"] [1] - kps ["l_elb"] [1])

        self.processed_data ["righthand"] = -angle_2_vec (neck_hip, sh_r_elb)
        self.processed_data ["lefthand"]  = angle_2_vec (neck_hip, sh_l_elb)
        self.processed_data ["rightarm"]  = angle_2_vec (sh_r_elb, elb_r_wri)
        self.processed_data ["leftarm"]   = angle_2_vec (sh_l_elb, elb_l_wri)

    def _interpret_data(self):
        pass

    def _get_command(self):
        comm = self.commands ["noaction"]

        if (self.timeout.timeout_passed ()):
            movement = 1

            #print ("aa", self.processed_data ["righthand"])
            if (self.processed_data ["righthand"] > -1):
                movement = 2

            comm = self.commands[str(movement)]

            #self.tick += 1

        #print ("com", comm)

        return comm

    def get_command(self, skip_reading_data=False):
        self._read_data()
        self._process_data()
        self._interpret_data()

        return self._get_command()
