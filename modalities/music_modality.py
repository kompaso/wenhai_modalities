from modalities.modality import  Modality

#from modalities.skeleton_modalities import Skeleton_3D_Music_to_dance
from modalities.modality import WorkWithPoints

import numpy as np
from common import *

import pydub
from pydub import AudioSegment
from pydub.playback import play
import scipy.fftpack
import cv2

import multiprocessing

#class Motion_source:
#    def __init__ (self):
#        pass

#    def get_motion (self, time):
#        return np.zeros (18, np.float32)

#from skeleton_modalities import smth
#class Cyclic
#class Markov_chain
#class Rhytmic_sine

#class Archieve_data
#class Archieve_data_format1
#class Archieve_data_format2

#class External_model_loader

class Music (Modality):
    def __init__ (self, music_path_ = "", logger_ = 0):
        self.logger = logger_
        self.music_path = music_path_

        self.tick = 0

        self.commands = {"noaction": [("noaction", [""])],
                         "0": [("/increment_joint_angle", ["l_sho_roll", "-0.11"])],
                         "1": [("/increment_joint_angle", ["l_sho_roll", "0.11"])]
                         }

        self.rate, self.audio = self.read(music_path_)
        self._extract_rhythm ()
        self.timeout = Timeout_module(1 / self.rhythm / 8)

        #song = AudioSegment.from_mp3 (music_path_)
        #play (song)

    def play_song (self):
        pass

    def read(self, f, normalized=False):
        """MP3 to numpy array"""
        a = pydub.AudioSegment.from_mp3(f)
        y = np.array(a.get_array_of_samples())
        if a.channels == 2:
            y = y.reshape((-1, 2))
        if normalized:
            return a.frame_rate, np.float32(y) / 2 ** 15
        else:
            return a.frame_rate, y

    def write(self, f, sr, x, normalized=False):
        """numpy array to MP3"""
        channels = 2 if (x.ndim == 2 and x.shape[1] == 2) else 1
        if normalized:  # normalized array - each item should be a float in [-1, 1)
            y = np.int16(x * 2 ** 15)
        else:
            y = np.int16(x)
        song = pydub.AudioSegment(y.tobytes(), frame_rate=sr, sample_width=2, channels=channels)
        song.export(f, format="mp3", bitrate="320k")

    def _extract_rhythm (self):
        N = 2000
        an_part = self.audio [:2000, 1]
        x = np.linspace (0, 2 * np.pi, N)

        w = scipy.fftpack.rfft (an_part)
        f = scipy.fftpack.rfftfreq (N, x[1] - x[0])
        spectrum = w**2

        cutoff_idx = spectrum > (spectrum.max () / 15)
        w2 = w.copy ()
        w2 [cutoff_idx] = 0

        print("len", w2)
        print("w", len(w2))

        self.rhythm = f [1]

    def name(self):
        return "Baseline dance generation with audio input"

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

            comm = self.commands[str (np.random.randint (1, l))]

            self.tick += 1

        return comm

    def get_command(self, skip_reading_data=False):
        self._read_data()
        self._process_data()
        self._interpret_data()

        return self._get_command()

    def draw (self, canvas = np.ones ((700, 700, 3), np.uint8) * 220):
        result = canvas.copy ()

        cv2.putText (result, self.music_path, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (20, 50, 31), 1, cv2.LINE_AA)

        return [result]

class Cyclic (Music):
    def __init__ (self, music_path_ = "", logger_ = 0, dance_length_ = 50):
        Music.__init__ (self, music_path_, logger_)
        self.tick = 0
        self.dance_length = dance_length_

        self.rate, self.audio = self.read (self.music_path)
        self._extract_rhythm ()

        #self.timeout = Timeout_module (1 / self.rhythm / 8)
        self.timeout = Timeout_module (0.01)

        print ("timeout:", self.timeout)

        #song = AudioSegment.from_mp3 (music_path_)
        #play (song)

        hip_ampl  = 0.07
        head_ampl = 0.15

        head_pose_1 = [("/set_joint_angle", ["head_Pitch", str (-head_ampl)]),
                       ("/set_joint_angle", ["head_Yaw",   str (-head_ampl)])]

        head_pose_2 = [("/set_joint_angle", ["head_Pitch", str (-head_ampl)]),
                       ("/set_joint_angle", ["head_Yaw",   "0.0"])]

        head_pose_3 = [("/set_joint_angle", ["head_Pitch", str (-head_ampl)]),
                       ("/set_joint_angle", ["head_Yaw",   str (head_ampl)])]

        legs_pose_1 = [("/set_joint_angle", ["l_hip_pitch", "0.0"]),
                       ("/set_joint_angle", ["l_knee_pitch", "0.0"]),
                       ("/set_joint_angle", ["l_ank_pitch", "0.0"]),
                       ("/set_joint_angle", ["r_hip_pitch", "0.0"]),
                       ("/set_joint_angle", ["r_knee_pitch", "0.0"]),
                       ("/set_joint_angle", ["r_ank_pitch", "0.0"])]

        legs_pose_2 = [("/set_joint_angle", ["l_hip_pitch", str (-hip_ampl)]),
                       ("/set_joint_angle", ["l_knee_pitch", str (2 * hip_ampl)]),
                       ("/set_joint_angle", ["l_ank_pitch", str (-hip_ampl)]),
                       ("/set_joint_angle", ["r_hip_pitch", str (-hip_ampl)]),
                       ("/set_joint_angle", ["r_knee_pitch", str (2 * hip_ampl)]),
                       ("/set_joint_angle", ["r_ank_pitch", str (-hip_ampl)])]

        legs_pose_3 = [("/set_joint_angle", ["l_hip_pitch", str (0)]),
                       ("/set_joint_angle", ["l_knee_pitch", str (0)]),
                       ("/set_joint_angle", ["l_ank_pitch", str (0)]),
                       ("/set_joint_angle", ["r_hip_pitch", str (-hip_ampl)]),
                       ("/set_joint_angle", ["r_knee_pitch", str (2 * hip_ampl)]),
                       ("/set_joint_angle", ["r_ank_pitch", str (-hip_ampl)])]

        legs_pose_4 = [("/set_joint_angle", ["l_hip_pitch", str (-hip_ampl)]),
                       ("/set_joint_angle", ["l_knee_pitch", str (2 * hip_ampl)]),
                       ("/set_joint_angle", ["l_ank_pitch", str (-hip_ampl)]),
                       ("/set_joint_angle", ["r_hip_pitch", str (0)]),
                       ("/set_joint_angle", ["r_knee_pitch", str (0)]),
                       ("/set_joint_angle", ["r_ank_pitch", str (0)])]

        self.commands = {
                         "noaction": [("noaction", [""])],

                         "0": head_pose_1 + legs_pose_1,

                         "1": head_pose_2 + legs_pose_2,

                         "2": head_pose_3 + legs_pose_3,

                         "3": head_pose_2 + legs_pose_4
                         }

    def play_song (self):
        pass

    def name(self):
        return "Cyclic moves performing"

    def _read_data (self):
        pass

    def _process_data(self):
        pass

    def _interpret_data(self):
        pass

    def _get_command(self):
        comm = self.commands ["noaction"]

        if (self.tick >= self.dance_length):
            return comm

        if (self.timeout.timeout_passed ()):
            l = len (self.commands)

            regular_part = self.commands[str (self.tick % (l - 1))]

            # cyclic_angle_1 = math.sin (float (self.tick) / 3) / 4
            # cyclic_angle_2 = math.sin (float (self.tick + 1.5) / 3) / 4

            # unique_joints = {
            #                  "l_sho_roll" : cyclic_angle_1 + 0.4,
            #                  "l_elb_roll" : cyclic_angle_1 - 0.4,
            #                  "l_sho_pitch": cyclic_angle_1 - 0.4,
            #
            #                  "r_sho_roll" : cyclic_angle_1 - 0.4,
            #                  "r_elb_roll" : cyclic_angle_1 + 0.4,
            #                  "r_sho_pitch": cyclic_angle_2 - 0.4
            #                 }
            #
            # unique_part= []
            #
            # for unique_joint in unique_joints.keys ():
            #     unique_part += [("/set_joint_angle", [unique_joint, str (unique_joints [unique_joint])])]

            unique_part = []

            comm = regular_part + unique_part

            self.tick += 1

        return comm

    def get_command(self, skip_reading_data=False):
        self._read_data()
        self._process_data()
        self._interpret_data()

        return self._get_command()

class Skeleton_3D_Music_to_dance (WorkWithPoints):
    def __init__ (self, skeleton_path_ = "", logger_ = 0):
        WorkWithPoints.__init__(self, logger_, maxlen_=20)
        self.all_data         = []
        self.dataframe_num = 0
        self.previous_knee = 0
        self.previous_hip = 0
        self.previous_ankl = 0
        self.mode = 0.0

        self.skeleton_path = skeleton_path_

        if (skeleton_path_ != ""):
            verbose = False
            if( os.path.isfile (self.skeleton_path) == True ):
                print( "Skeleton file: ", self.skeleton_path)

                #skeleton_data = open(skeleton_path_, 'r')
                #all_skeleton_frames = self.read_skeleton_data_from_NTU(skeleton_data, verbose )

                f = open (skeleton_path_, )
                data = json.load (f)
                all_skeleton_frames = data["skeletons"] [10:1550]

                self.all_data = all_skeleton_frames
            else:
                print("\nNo skeleton file with name: ", data_path)
                exit(0)

    def name (self):
        return "skeleton"

    def _read_data (self):
        if (self.dataframe_num >= len (self.all_data)):
            read_data = 0
            return

        self.read_data = self.all_data [self.dataframe_num]
        self.dataframe_num += 1

    def create_dicts_with_coords_3D(self):
        kps = {}
        if self.read_data != []:
            for kp in self.necessary_keypoints_names:
                ind = self.kpt_names.index(kp)
                if kp == 'mid_hip':
                    if (kps["l_hip"][0] > 0 and kps["r_hip"][0] > 0):
                        kps.update ({kp : [(self.read_data[6][0] + self.read_data[12][0]) / 2, (self.read_data[6][1] +
                                            self.read_data[12][1]) / 2, (self.read_data[6][2] + self.read_data[12][2]) / 2]})
                    else:
                        kps.update ({kp : [self.read_data[0][0], self.read_data[0][1] + 200,  self.read_data[0][2]]})
                else:
                    kps.update ({kp : [self.read_data[ind][0], self.read_data[ind][1],  self.read_data[ind][2]]})

                self.kps_mean[kp]["x"].append(kps[kp][0])
                self.kps_mean[kp]["y"].append(kps[kp][1])
                self.kps_mean[kp]["z"].append(kps[kp][2])
        return kps

    def get_read_data (self):
        return self.read_data

    def _process_data (self, frame = None):
        # name = self.read_data
        # self.read_data = self.read_data

        self.interpreted_data = self.create_dicts_with_coords_3D()

        kps = self.get_mean_cords(self.kps_mean)

        # self.processed_data["mode"] = 1

        ##################################################left_full_hand##############################################################
        l_hip_neck = common.create_vec(kps["mid_hip"], kps["neck"])
        neck_l_sho = common.create_vec(kps["neck"], kps["l_sho"])
        l_elb_sho = common.create_vec(kps["l_elb"], kps["l_sho"])
        l_sho_elb = common.create_vec(kps["l_sho"], kps["l_elb"])
        l_elb_wri = common.create_vec(kps["l_elb"], kps["l_wri"])

        N_l_body_plane = np.cross(neck_l_sho, l_hip_neck)
        N_neck_l_sho_elb = np.cross(neck_l_sho, l_elb_sho)
        N_l_sho_elb_wri = np.cross(l_elb_sho, l_elb_wri)

        R_l_body_plane = np.cross(l_hip_neck, N_l_body_plane)
        R_l_arm = np.cross(l_elb_sho, N_neck_l_sho_elb)
        R_lbp_lse = np.cross(R_l_body_plane, l_sho_elb)

        mod_N_neck_l_sho_elb = common.get_mod(N_neck_l_sho_elb)
        mod_N_l_body_plane = common.get_mod(N_l_body_plane)
        mod_N_l_sho_elb_wri = common.get_mod(N_l_sho_elb_wri)
        mod_l_hip_neck = common.get_mod(l_hip_neck)
        mod_l_sho_elb = common.get_mod(l_sho_elb)
        mod_l_elb_sho = common.get_mod(l_elb_sho)
        mod_l_elb_wri = common.get_mod(l_elb_wri)
        mod_R_l_body_plane = common.get_mod(R_l_body_plane)
        mod_R_lbp_lse = common.get_mod(R_lbp_lse)
        mod_R_l_arm = common.get_mod(R_l_arm)

        l_sho_pitch_raw = math.acos(np.dot(l_hip_neck, R_lbp_lse) / (mod_l_hip_neck * mod_R_lbp_lse)) - 0.65
        l_elb_yaw_raw = math.acos(
            np.dot(N_neck_l_sho_elb, N_l_sho_elb_wri) / (mod_N_neck_l_sho_elb * mod_N_l_sho_elb_wri))

        phi_lsp = math.acos(np.dot(l_sho_elb, l_hip_neck) / (mod_l_hip_neck * mod_l_sho_elb))
        phi_ley_1 = math.acos(np.dot(l_elb_wri, N_neck_l_sho_elb) / (mod_l_elb_wri * mod_N_neck_l_sho_elb))
        phi_ley_2 = math.acos(np.dot(l_elb_wri, R_l_arm) / (mod_l_elb_wri * mod_R_l_arm))

        l_elb_yaw = 0
        if phi_ley_1 <= 1.57:
            l_elb_yaw = - l_elb_yaw_raw
        if phi_ley_1 > 1.57 and phi_ley_2 > 1.57:
            l_elb_yaw = l_elb_yaw_raw
        if phi_ley_1 > 1.57 and phi_ley_2 <= 1.57:
            l_elb_yaw = l_elb_yaw_raw - 6.28

        if phi_lsp <= 1.57:
            l_sho_pitch = -l_sho_pitch_raw
        else:
            l_sho_pitch = l_sho_pitch_raw

        l_sho_roll = 1.57 - math.acos(np.dot(l_sho_elb, R_l_body_plane) / (mod_l_sho_elb * mod_R_l_body_plane))
        l_elb_roll = -(3.14 - math.acos(np.dot(l_elb_wri, l_elb_sho) / (mod_l_elb_wri * mod_l_elb_sho)))

        #####################################################################################################################
        self.angles_mean["l_sho_pitch"].append(l_sho_pitch)
        self.angles_mean["l_sho_roll"].append(l_sho_roll)
        self.angles_mean["l_elb_yaw"].append(l_elb_yaw)
        self.angles_mean["l_elb_roll"].append(l_elb_roll)

        # self.logger.update("l shoul pitch", round(self.get_mean(self.angles_mean["l_sho_pitch"]), 2))
        # self.logger.update("l shoul roll", round(self.get_mean(self.angles_mean["l_sho_roll"]), 2))
        # self.logger.update("l elb yaw", round(self.get_mean(self.angles_mean["l_elb_yaw"]), 2))
        # self.logger.update("l elb roll", round(self.get_mean(self.angles_mean["l_elb_roll"]), 2))

        self.processed_data ["l_sho_pitch"]  = round(self.get_mean(self.angles_mean["l_sho_pitch"]), 2)
        self.processed_data ["l_sho_roll"]  = round(self.get_mean(self.angles_mean["l_sho_roll"]), 2)
        self.processed_data ["l_elb_yaw"]  = round(self.get_mean(self.angles_mean["l_elb_yaw"]), 2)
        self.processed_data ["l_elb_roll"]  = round(self.get_mean(self.angles_mean["l_elb_roll"]), 2)
        ##############################################################################################################################

        ##########################################r_full_hand###############################################################
        r_hip_neck = common.create_vec(kps["mid_hip"], kps["neck"])
        neck_r_sho = common.create_vec(kps["neck"], kps["r_sho"])
        r_elb_sho = common.create_vec(kps["r_elb"], kps["r_sho"])
        r_sho_elb = common.create_vec(kps["r_sho"], kps["r_elb"])
        r_elb_wri = common.create_vec(kps["r_elb"], kps["r_wri"])

        N_r_body_plane = -np.cross(neck_r_sho, r_hip_neck)
        N_neck_r_sho_elb = np.cross(neck_r_sho, r_elb_sho)
        N_r_sho_elb_wri = np.cross(r_elb_sho, r_elb_wri)

        R_r_body_plane = np.cross(r_hip_neck, N_r_body_plane)
        R_r_arm = np.cross(r_elb_sho, N_neck_r_sho_elb)
        R_rbp_rse = np.cross(R_r_body_plane, r_sho_elb)

        mod_N_neck_r_sho_elb = common.get_mod(N_neck_r_sho_elb)
        mod_N_r_sho_elb_wri = common.get_mod(N_r_sho_elb_wri)
        mod_r_hip_neck = common.get_mod(r_hip_neck)
        mod_r_sho_elb = common.get_mod(r_sho_elb)
        mod_r_elb_sho = common.get_mod(r_elb_sho)
        mod_r_elb_wri = common.get_mod(r_elb_wri)
        mod_R_rbp_rse = common.get_mod(R_rbp_rse)
        mod_R_r_body_plane = common.get_mod(R_r_body_plane)
        mod_R_r_arm = common.get_mod(R_r_arm)

        r_sho_pitch_raw = math.acos(np.dot(r_hip_neck, R_rbp_rse) / (mod_r_hip_neck * mod_R_rbp_rse)) - 0.65
        r_elb_yaw_raw = math.acos(
            np.dot(N_neck_r_sho_elb, N_r_sho_elb_wri) / (mod_N_neck_r_sho_elb * mod_N_r_sho_elb_wri))

        phi_rsp = math.acos(np.dot(r_sho_elb, r_hip_neck) / (mod_r_hip_neck * mod_r_sho_elb))
        phi_rey_1 = math.acos(np.dot(r_elb_wri, N_neck_r_sho_elb) / (mod_r_elb_wri * mod_N_neck_r_sho_elb))
        phi_rey_2 = math.acos(np.dot(r_elb_wri, R_r_arm) / (mod_r_elb_wri * mod_R_r_arm))

        r_elb_yaw = 0
        if phi_rey_1 <= 1.57:
            r_elb_yaw = r_elb_yaw_raw
        if phi_rey_1 > 1.57 and phi_rey_2 > 1.57:
            r_elb_yaw = -r_elb_yaw_raw
        if phi_rey_1 > 1.57 and phi_rey_2 <= 1.57:
            r_elb_yaw = r_elb_yaw_raw - 6.28

        if phi_rsp <= 1.57:
            r_sho_pitch = -r_sho_pitch_raw
        else:
            r_sho_pitch = r_sho_pitch_raw

        r_sho_roll = 1.57 - math.acos(np.dot(r_sho_elb, R_r_body_plane) / (mod_r_sho_elb * mod_R_r_body_plane))
        r_elb_roll = 3.14 - math.acos(np.dot(r_elb_wri, r_elb_sho) / (mod_r_elb_wri * mod_r_elb_sho))
        #####################################################################################################################
        self.angles_mean["r_sho_pitch"].append(r_sho_pitch)
        self.angles_mean["r_sho_roll"].append(r_sho_roll)
        self.angles_mean["r_elb_yaw"].append(r_elb_yaw)
        self.angles_mean["r_elb_roll"].append(r_elb_roll)

        # self.logger.update("r shoul pitch" , round(self.get_mean(self.angles_mean["r_sho_pitch"]), 2))
        # self.logger.update("r shoul roll", round(self.get_mean(self.angles_mean["r_sho_roll"]), 2))
        # self.logger.update("r elb yaw", round(self.get_mean(self.angles_mean["r_elb_yaw"]), 2))
        # self.logger.update("r elb roll", round(self.get_mean(self.angles_mean["r_elb_roll"]), 2))

        self.processed_data ["r_sho_pitch"] = round(self.get_mean(self.angles_mean["r_sho_pitch"]), 2)
        self.processed_data ["r_sho_roll"]  = round(self.get_mean(self.angles_mean["r_sho_roll"]), 2)
        self.processed_data ["r_elb_yaw"]   = round(self.get_mean(self.angles_mean["r_elb_yaw"]), 2)
        self.processed_data ["r_elb_roll"]  = round(self.get_mean(self.angles_mean["r_elb_roll"]), 2)
        #################################################################################################################################

    def _interpret_data (self):
        self.interpreted_data = self.processed_data

    def _get_command (self):
        commands = []

        #print ("keys:", self.processed_data.keys ())

        smol_listb = ["l_sho_roll", "l_elb_roll", "l_sho_pitch",
                      "r_sho_roll", "r_elb_roll", "r_sho_pitch"]

        smol_dict = {}

        for key in smol_listb:
            smol_dict.update ({key : self.processed_data [key]})

        #for key in self.processed_data.keys ():
        for key in smol_dict.keys ():
            commands.append (("/set_joint_angle", [key, str (self.processed_data [key])]))

        return commands

    def get_command (self, skip_reading_data = False):
        if (skip_reading_data == False):
            self._read_data ()

        self._process_data   ()
        self._interpret_data ()

        return self._get_command ()

    def draw (self, canvas = np.ones ((700, 700, 3), np.uint8) * 220):
        result = canvas.copy ()

        cv2.putText (result, self.skeleton_path, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (20, 50, 31), 1, cv2.LINE_AA)

        return [result]
