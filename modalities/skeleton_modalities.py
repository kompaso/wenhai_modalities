from modalities.modality import WorkWithPoints
import common
import os

from pose_estimation.one_euro_filter import OneEuroFilter

import numpy as np
import cv2
import math

class Skeleton_2D(WorkWithPoints):
    def __init__ (self, skeleton_path_ = "", logger_ = 0):
        WorkWithPoints.__init__(self)
        self.all_data         = []

        self.dataframe_num = 0
        self.read_data_from_file_ = False

        if (skeleton_path_ != ""):
            self.all_data = self.read_data_from_file(skeleton_path_)


    def name (self):
        return "skeleton"

    def _read_data (self):
        if (self.dataframe_num >= len (self.all_data)):
            read_data = 0
            return

        self.read_data = self.all_data [self.dataframe_num]
        self.dataframe_num += 1

    def get_read_data (self):
        return self.read_data

    def hand_up_angles(self, angle, hand):
        hand_roll  = angle
        hand_pitch = 0

        if hand == "righthand" :
            k = 1
            if (angle <= -1.3*k and angle > -1.8*k):
                hand_roll = -1.3*k
                hand_pitch = (2.04 + 3.14 / 5) / 0.5 * (angle + 1.3)

            elif (angle <= -1.8*k):
                hand_roll = -1.3*k - (angle + 1.8*k)
                hand_pitch = - (2.04 + 3.14 / 5)

            return hand_roll, hand_pitch

        if hand == "lefthand" :
            k = -1
        if (angle >= 1.3 and angle < 1.8):
            hand_roll = -1.3*k
            hand_pitch = (2.04 + 3.14 / 5) / 0.5 * (angle*k + 1.3)

        elif (angle >= -1.8*k):
            hand_roll = -1.3*k - (angle + 1.8*k)
            hand_pitch = - (2.04 + 3.14 / 5)

        return hand_roll, hand_pitch

    def _process_data (self, frame = None):
        kps = {}

        for kp in self.necessary_keypoints_names:
            ind = self.kpt_names.index (kp)
            kps.update ({kp : (self.read_data [ind * 2], self.read_data [ind * 2 + 1])})

    def _interpret_data (self):
        self.interpreted_data = self.processed_data

    def _get_command (self):
        commands = []

        for key in self.processed_data.keys ():
            commands.append (("/set_joint_angle", [key, str (self.processed_data [key])]))

        return commands

    def get_command (self, skip_reading_data = False):
        if (skip_reading_data == False):
            self._read_data ()

        self._process_data   ()
        self._interpret_data ()

        return self._get_command ()

class Skeleton_3D(WorkWithPoints):
    def __init__ (self, skeleton_path_ = "", logger_ = 0):
        WorkWithPoints.__init__(self, logger_, maxlen_=15)
        self.all_data         = []
        self.dataframe_num = 0
        self.previous_knee = 0
        self.previous_hip = 0
        self.previous_ankl = 0
        self.filter = OneEuroFilter(freq=1, beta=0.0001)
        self.mode = 0.0
        self.count = 0


        if (skeleton_path_ != ""):
            verbose = False
            if (os.path.isfile(skeleton_path_) == True):
                print( "Skeleton file: ", skeleton_path_)
                skeleton_data = open(skeleton_path_, 'r')
                all_skeleton_frames = self.read_skeleton_data_from_NTU(skeleton_data, verbose )
                self.all_data = all_skeleton_frames
            else:
                print("\nNo skeleton file with name: ", data_path)
                exit(0)


    def name (self):
        return "skeleton"


    def detecting_mode(self, kps):
        if (kps['l_ank'][1] - kps['r_ank'][1]) > 30:
            return 1.0
        elif (kps['r_ank'][1] - kps['l_ank'][1]) > 30:
            return 2.0
        else:
            return 0.0


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
                ind = self.kpt_names_lw.index(kp)
                if kp == 'mid_hip':
                    if (kps["l_hip"][0] > 0 and kps["r_hip"][0] > 0):
                        kps.update ({kp : [int((self.read_data[6][0] + self.read_data[12][0]) / 2), int((self.read_data[6][1] + self.read_data[12][1]) / 2), int((self.read_data[6][2] + self.read_data[12][2]) / 2)]})
                    else:
                        kps.update ({kp : [int(self.read_data[0][0]), int(self.read_data[0][1] + 200),  int(self.read_data[0][2])]})
                else:
                    kps.update ({kp : [self.read_data[ind][0], self.read_data[ind][1],  self.read_data[ind][2]]})
                self.kps_mean[kp]["x"].append(kps[kp][0])
                self.kps_mean[kp]["y"].append(kps[kp][1])
                self.kps_mean[kp]["z"].append(kps[kp][2])
        return kps


    def get_read_data (self):
        return self.read_data


    def _process_data (self, frame = None):

        self.interpreted_data = self.create_dicts_with_coords_3D()

        kps = self.get_mean_cords(self.kps_mean)

        ##################################################left_full_hand##############################################################
        l_hip_neck = common.create_vec(kps["mid_hip"], kps["neck"])
        mod_l_hip_neck = common.get_mod(l_hip_neck)

        neck_l_sho = common.create_vec(kps["neck"], kps["l_sho"])
        l_elb_sho = common.create_vec(kps["l_elb"], kps["l_sho"])
        l_sho_elb = common.create_vec(kps["l_sho"], kps["l_elb"])

        N_l_body_plane = np.cross(neck_l_sho, l_hip_neck)
        R_l_body_plane = np.cross(l_hip_neck, N_l_body_plane)

        R_lbp_lse = np.cross(R_l_body_plane, l_sho_elb)
        mod_R_lbp_lse = common.get_mod(R_lbp_lse)

        l_sho_pitch_raw = math.acos(np.dot(l_hip_neck, R_lbp_lse)*(mod_l_hip_neck*mod_R_lbp_lse))

        l_elb_wri = common.create_vec(kps["l_elb"], kps["l_wri"])

        N_neck_l_sho_elb = np.cross(neck_l_sho, l_elb_sho)
        N_l_sho_elb_wri = np.cross(l_elb_sho, l_elb_wri)

        R_l_arm = np.cross(l_elb_sho, N_neck_l_sho_elb)

        mod_N_neck_l_sho_elb = common.get_mod(N_neck_l_sho_elb)
        mod_N_l_body_plane = common.get_mod(N_l_body_plane)
        mod_N_l_sho_elb_wri = common.get_mod(N_l_sho_elb_wri)

        mod_l_sho_elb = common.get_mod(l_sho_elb)
        mod_l_elb_sho = common.get_mod(l_elb_sho)
        mod_l_elb_wri = common.get_mod(l_elb_wri)
        mod_R_l_body_plane = common.get_mod(R_l_body_plane)

        mod_R_l_arm = common.get_mod(R_l_arm)

        l_elb_yaw_raw = math.acos(np.dot(N_neck_l_sho_elb, N_l_sho_elb_wri)/(mod_N_neck_l_sho_elb*mod_N_l_sho_elb_wri))

        phi_lsp = math.acos(np.dot(l_sho_elb,l_hip_neck)/(mod_l_hip_neck * mod_l_sho_elb))
        phi_ley_1 = math.acos(np.dot(l_elb_wri, N_neck_l_sho_elb)/(mod_l_elb_wri * mod_N_neck_l_sho_elb))
        phi_ley_2 = math.acos(np.dot(l_elb_wri, R_l_arm)/(mod_l_elb_wri * mod_R_l_arm))

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

        l_sho_roll = 1.57 - math.acos(np.dot(l_sho_elb, R_l_body_plane)/(mod_l_sho_elb *mod_R_l_body_plane ))
        l_elb_roll = -(3.14 - math.acos(np.dot(l_elb_wri, l_elb_sho)/(mod_l_elb_wri*mod_l_elb_sho)))

#####################################################################################################################
        self.angles_mean["l_sho_pitch"].append(self.filter(l_sho_pitch))
        self.angles_mean["l_sho_roll"].append(l_sho_roll)
        self.angles_mean["l_elb_yaw"].append(self.filter(l_elb_yaw))
        self.angles_mean["l_elb_roll"].append(l_elb_roll + 0.2)

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

        r_sho_pitch_raw = math.acos(np.dot(r_hip_neck, R_rbp_rse)/(mod_r_hip_neck*mod_R_rbp_rse))

        r_elb_yaw_raw = math.acos(np.dot(N_neck_r_sho_elb, N_r_sho_elb_wri)/(mod_N_neck_r_sho_elb*mod_N_r_sho_elb_wri))

        phi_rsp = math.acos(np.dot(r_sho_elb,r_hip_neck)/(mod_r_hip_neck * mod_r_sho_elb))
        phi_rey_1 = math.acos(np.dot(r_elb_wri, N_neck_r_sho_elb)/(mod_r_elb_wri * mod_N_neck_r_sho_elb))
        phi_rey_2 = math.acos(np.dot(r_elb_wri, R_r_arm)/(mod_r_elb_wri * mod_R_r_arm))

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

        r_sho_roll = 1.77 - math.acos(np.dot(r_sho_elb, R_r_body_plane)/(mod_r_sho_elb *mod_R_r_body_plane ))
        r_elb_roll = 3.14 - math.acos(np.dot(r_elb_wri, r_elb_sho)/(mod_r_elb_wri*mod_r_elb_sho))
#####################################################################################################################
        self.angles_mean["r_sho_pitch"].append(r_sho_pitch)
        self.angles_mean["r_sho_roll"].append(r_sho_roll)
        self.angles_mean["r_elb_yaw"].append(r_elb_yaw)
        self.angles_mean["r_elb_roll"].append(r_elb_roll)


        ################################################l_hip###########################################################################
        l_mhip_lhip = common.create_vec(kps["mid_hip"],kps["l_hip"])
        l_hip_knee = common.create_vec(kps["l_hip"],kps["l_knee"])
        N_mh_neck_lhip = np.cross(l_mhip_lhip, l_hip_neck)
        R_l_hip = np.cross(l_hip_neck, N_mh_neck_lhip)

        N_neck_l_mh_hk = np.cross(N_mh_neck_lhip, l_hip_knee)


        mod_l_mhip_lhip = common.get_mod(l_mhip_lhip)
        mod_l_hip_knee = common.get_mod(l_hip_knee)
        mod_R_l_hip = common.get_mod(R_l_hip)
        mod_N_mh_neck_lhip = common.get_mod(N_mh_neck_lhip)
        mod_N_neck_l_mh_hk = common.get_mod(N_neck_l_mh_hk)


        ###############################################l_knee#####################################################################
        l_knee_hip = common.create_vec(kps["l_knee"], kps["l_hip"])
        l_knee_ankle = common.create_vec(kps["l_knee"], kps["l_ank"])
        N_l_hip_knee_ankle = np.cross(l_knee_ankle, l_knee_hip)
        R_l_leg = np.cross(N_l_hip_knee_ankle, l_knee_hip)

        mod_l_knee_hip = common.get_mod(l_knee_hip)
        mod_l_knee_ankle = common.get_mod(l_knee_ankle)

        # print(mod_l_knee_ankle, mod_l_knee_hip, np.dot(l_knee_ankle, l_knee_hip),l_knee_ankle, l_knee_hip )
        left_knee_pitch_raw = np.dot(l_knee_ankle, l_knee_hip)/(float(mod_l_knee_ankle)*float(mod_l_knee_hip))
        if left_knee_pitch_raw > 1:
            left_knee_pitch_raw = 1

        ##############################################r_knee####################################################################
        r_knee_hip = common.create_vec(kps["r_knee"], kps["r_hip"])
        r_knee_ankle = common.create_vec(kps["r_knee"], kps["r_ank"])
        N_r_hip_knee_ankle = np.cross(r_knee_ankle, r_knee_hip)
        R_r_leg = np.cross(N_r_hip_knee_ankle, r_knee_hip)

        mod_r_knee_hip = common.get_mod(r_knee_hip)
        mod_r_knee_ankle = common.get_mod(r_knee_ankle)
        right_knee_pitch_raw = np.dot(r_knee_ankle, r_knee_hip)/(mod_r_knee_ankle*mod_r_knee_hip)
        if right_knee_pitch_raw > 1:
            right_knee_pitch_raw = 1
        right_knee_pitch = 3.14 - math.acos(right_knee_pitch_raw)
        ###########################################################################################################################
        # self.angles_mean["r_knee_pitch"].append(right_knee_pitch)

        l_knee = round(self.get_mean(self.angles_mean["l_knee_pitch"]), 2)
        r_knee =  round(self.get_mean(self.angles_mean["r_knee_pitch"]), 2)

        ########################################test ankle#################################################################

        # x = math.acos(np.dot(N_l_body_plane, l_knee_ankle)/(mod_N_l_body_plane * mod_l_knee_ankle))
        # self.logger.update("ankle pitch", round(x, 2))
        ###############################################r_hip#########################################################################
        r_mhip_rhip = common.create_vec(kps["mid_hip"],kps["r_hip"])
        r_hip_knee = common.create_vec(kps["r_hip"],kps["r_knee"])
        N_mh_neck_rhip = np.cross(r_mhip_rhip, r_hip_neck)
        R_r_hip = np.cross(r_hip_neck, N_mh_neck_rhip)

        N_neck_r_mh_hk = np.cross(N_mh_neck_rhip, r_hip_knee)


        mod_r_mhip_rhip = common.get_mod(r_mhip_rhip)
        mod_r_hip_knee = common.get_mod(r_hip_knee)
        mod_R_r_hip = common.get_mod(R_r_hip)
        mod_N_mh_neck_rhip = common.get_mod(N_mh_neck_rhip)
        mod_N_neck_r_mh_hk = common.get_mod(N_neck_r_mh_hk)

        # right_hip_roll_raw = math.acos(np.dot(R_r_hip, N_neck_r_mh_hk)/(mod_R_r_hip*mod_N_neck_r_mh_hk))

        phi_rhr = math.acos(np.dot(r_hip_knee, R_r_hip)/(mod_r_hip_knee*mod_R_r_hip))


        right_hip_pitch = - 1.57 + math.acos(np.dot(r_hip_knee,N_mh_neck_rhip)/(mod_r_hip_knee*mod_N_mh_neck_rhip))
        ###########################################################################################################################
        # self.angles_mean["r_hip_roll"].append(right_hip_roll)
        self.angles_mean["r_hip_pitch"].append(right_hip_pitch)
        self.angles_mean["r_ank_pitch"].append(right_hip_pitch)
        # # self.angles_mean["r_elb_yaw"].append(r_elb_yaw)
        # # self.angles_mean["r_elb_roll"].append(r_elb_roll)
        x = round(self.get_mean(self.angles_mean["l_knee_pitch"]), 2)
        y = round(self.get_mean(self.angles_mean["r_knee_pitch"]), 2)
        # self.logger.update("r knee" , round((x + y)/2, 2))
        # self.logger.update("l knee", round((x + y)/2, 2))
        # # self.logger.update("r elb yaw", round(self.get_mean(self.angles_mean["r_elb_yaw"]), 2))
        # # self.logger.update("r elb roll", round(self.get_mean(self.angles_mean["r_elb_roll"]), 2))
        #
        # # self.processed_data ["r_hip_roll"] = round(self.get_mean(self.angles_mean["r_hip_roll"]), 2)
        # z = (x + y)/5
        # if (z < 0.7):
        #     self.processed_data ["l_hip_pitch"] = -round((x + y)/6, 2)
        #     self.processed_data ["r_hip_pitch"]  = -round((x + y)/6, 2)
        #     self.processed_data ["l_knee_pitch"] = round((x + y)/3, 2)
        #     self.processed_data ["r_knee_pitch"]  = round((x + y)/3, 2)
        #     self.processed_data ["l_ank_pitch"] = -round((x + y)/5, 2)
        #     self.processed_data ["r_ank_pitch"]  = -round((x + y)/5, 2)
        # else:
        #     self.processed_data ["l_hip_pitch"] = -0.7
        #     self.processed_data ["r_hip_pitch"]  = -0.7
        #     self.processed_data ["l_knee_pitch"] = 1.0
        #     self.processed_data ["r_knee_pitch"]  = 1.0
        #     self.processed_data ["l_ank_pitch"] = -0.7
        #     self.processed_data ["r_ank_pitch"]  = -0.7
        # self.processed_data ["l_hip_roll"] = -round(self.get_mean(self.angles_mean["r_hip_roll"]), 2)
        # self.processed_data ["l_ank_pitch"] = round(self.get_mean(self.angles_mean["r_ank_pitch"]), 2)
        # self.processed_data ["l_hip_pitch"]  = round(self.get_mean(self.angles_mean["r_hip_pitch"]), 2)
        # # self.processed_data ["r_elb_yaw"]   = round(self.get_mean(self.angles_mean["r_elb_yaw"]), 2)
        # # self.processed_data ["r_elb_roll"]  = round(self.get_mean(self.angles_mean["r_elb_roll"]), 2)
        # self.processed_data ["mode"] = "Double"
        #############################################################################################################################

        ###################################################head#######################################################################
        neck_l_sho = common.create_vec(kps["neck"], kps["l_sho"])
        neck_nose = common.create_vec(kps["neck"], kps["nose"])
        l_ear_eye = common.create_vec(kps["l_ear"], kps["l_eye"])
        mid_hip_neck = common.create_vec(kps["mid_hip"], kps["neck"])


        mod_neck_l_sho = common.get_mod(neck_l_sho)
        mod_neck_nose = common.get_mod(neck_nose)
        mod_l_ear_eye = common.get_mod(l_ear_eye)
        mod_mid_hip_neck = common.get_mod(mid_hip_neck)

        nose_neck_sho = np.dot(neck_l_sho, neck_nose)
        mid_hip_neck_nose = np.dot(mid_hip_neck, neck_nose)

        head_Yaw = -(1.57 - math.acos(nose_neck_sho/(mod_neck_l_sho*mod_neck_nose)))

####################bad numerical solution#########################
        head_pitch = -(math.acos(mid_hip_neck_nose/(mod_neck_nose * mod_mid_hip_neck))-0.6)


        ############################################################################################################################3
        self.angles_mean["head_Yaw"].append(head_Yaw)
        self.angles_mean["head_Pitch"].append(head_pitch)

        self.logger.update("Head Yaw", round(self.get_mean(self.angles_mean["head_Yaw"]), 2))
        self.logger.update("Head Pitch", round(self.get_mean(self.angles_mean["head_Pitch"]), 3))

        self.processed_data ["head_Yaw"]  = -round(self.get_mean(self.angles_mean["head_Yaw"]), 3)*2.5
        self.processed_data ["head_Pitch"]  = -round(self.get_mean(self.angles_mean["head_Pitch"]), 3)*1.5
        #############################################################################################################################

        #############################################################################################################################

        detecting_mode_1 =  math.acos(np.dot(r_hip_knee, l_hip_knee)/(mod_l_hip_knee*mod_r_hip_knee))



    def _interpret_data (self):
        self.interpreted_data = self.processed_data

    def _get_command (self):
        commands = []

        for key in self.processed_data.keys ():
            commands.append(("/set_joint_angle", [key, str (self.processed_data [key])]))

        return commands

    def get_command (self, skip_reading_data = False):
        if (skip_reading_data == False):
            self._read_data ()

        self._process_data   ()
        self._interpret_data ()

        return self._get_command ()
