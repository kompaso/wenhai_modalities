# -*- coding: utf-8 -*-
import math

import cv2
import numpy as np


previous_position = []
theta, phi = 3.1415/4, -3.1415/6
should_rotate = False
scale_dx = 800
scale_dy = 800


class Plotter3d:
    SKELETON_EDGES = np.array([[11, 10], [10, 9], [9, 0], [0, 3], [3, 4], [4, 5], [0, 6], [6, 7], [7, 8], [0, 12],
                               [12, 13], [13, 14], [0, 1], [1, 15], [15, 16], [1, 17], [17, 18]])

    def __init__(self, canvas_size, origin=(0.5, 0.5), scale=0.1):
        self.origin = np.array([origin[1] * canvas_size[1], origin[0] * canvas_size[0]], dtype=np.float32)  # x, y
        self.scale = np.float32(scale)
        self.theta = 0
        self.phi = 0
        axis_length = 200
        axes = [
            np.array([[-axis_length/2, -axis_length/2, 0], [axis_length/2, -axis_length/2, 0]], dtype=np.float32),
            np.array([[-axis_length/2, -axis_length/2, 0], [-axis_length/2, axis_length/2, 0]], dtype=np.float32),
            np.array([[-axis_length/2, -axis_length/2, 0], [-axis_length/2, -axis_length/2, axis_length]], dtype=np.float32)]
        step = 20
        for step_id in range(axis_length // step + 1):  # add grid
            axes.append(np.array([[-axis_length / 2, -axis_length / 2 + step_id * step, 0],
                                  [axis_length / 2, -axis_length / 2 + step_id * step, 0]], dtype=np.float32))
            axes.append(np.array([[-axis_length / 2 + step_id * step, -axis_length / 2, 0],
                                  [-axis_length / 2 + step_id * step, axis_length / 2, 0]], dtype=np.float32))
        self.axes = np.array(axes)

    def plot(self, img, vertices, edges):
        global theta, phi
        img.fill(0)
        R = self._get_rotation(theta, phi)
        self._draw_axes(img, R)
        if len(edges) != 0:
            self._plot_edges(img, vertices, edges, R)

    def _draw_axes(self, img, R):
        axes_2d = np.dot(self.axes, R)
        axes_2d = axes_2d * self.scale + self.origin
        for axe in axes_2d:
            axe = axe.astype(int)
            cv2.line(img, tuple(axe[0]), tuple(axe[1]), (128, 128, 128), 1, cv2.LINE_AA)

    def _plot_edges(self, img, vertices, edges, R):
        vertices_2d = np.dot(vertices, R)
        edges_vertices = vertices_2d.reshape((-1, 2))[edges] * self.scale + self.origin
        for edge_vertices in edges_vertices:
            edge_vertices = edge_vertices.astype(int)
            cv2.line(img, tuple(edge_vertices[0]), tuple(edge_vertices[1]), (255, 255, 255), 1, cv2.LINE_AA)

    def _get_rotation(self, theta, phi):
        sin, cos = math.sin, math.cos
        return np.array([
            [ cos(theta),  sin(theta) * sin(phi)],
            [-sin(theta),  cos(theta) * sin(phi)],
            [ 0,                       -cos(phi)]
        ], dtype=np.float32)  # transposed

    @staticmethod
    def mouse_callback(event, x, y, flags, params):
        global previous_position, theta, phi, should_rotate, scale_dx, scale_dy
        if event == cv2.EVENT_LBUTTONDOWN:
            previous_position = [x, y]
            should_rotate = True
        if event == cv2.EVENT_MOUSEMOVE and should_rotate:
            theta += (x - previous_position[0]) / scale_dx * 6.2831  # 360 deg
            phi -= (y - previous_position[1]) / scale_dy * 6.2831 * 2  # 360 deg
            phi = max(min(3.1415 / 2, phi), -3.1415 / 2)
            previous_position = [x, y]
        if event == cv2.EVENT_LBUTTONUP:
            should_rotate = False


body_edges = np.array(
    [[0, 1],  # neck - nose
     [1, 16], [16, 18],  # nose - l_eye - l_ear
     [1, 15], [15, 17],  # nose - r_eye - r_ear
     [0, 3], [3, 4], [4, 5],     # neck - l_shoulder - l_elbow - l_wrist
     [0, 9], [9, 10], [10, 11],  # neck - r_shoulder - r_elbow - r_wrist
     [0, 6], [6, 7], [7, 8],        # neck - l_hip - l_knee - l_ankle
     [0, 12], [12, 13], [13, 14]])  # neck - r_hip - r_knee - r_ankle

def draw_poses(img, poses_2d):
    res = []
    # pose = np.array(poses_2d[0][0:-1]).reshape((-1, 3)).transpose()


    if len (poses_2d) == 0:
        return []
    elif len(poses_2d) >= 1:
        pose = np.array(poses_2d[0][0:-1]).reshape((-1, 3)).transpose()

    was_found = pose[2, :] > 0
    for edge in body_edges:
        if was_found[edge[0]] and was_found[edge[1]]:
            cv2.line(img, tuple(pose[0:2, edge[0]].astype(int)), tuple(pose[0:2, edge[1]].astype(int)),
                     (255, 255, 0), 4, cv2.LINE_AA)
    for kpt_id in range(pose.shape[1]):
        res.append(pose[0:2, kpt_id].astype(int))
        # if pose[2, kpt_id] != -1:
        #     cv2.circle(img, tuple(pose[0:2, kpt_id].astype(int)), 3, (0, 255, 255), -1, cv2.LINE_AA)
    y = np.asarray(res).flatten()
#     print(res)
#     keys_ = ['neck', 'nose',
#                  'l_sho', 'l_elb',
#                  'l_wri', 'l_hip',
#                  'l_knee', 'l_ank',
#                  'r_sho', 'r_elb',
#                  'r_wri', 'r_hip',
#                  'r_knee', 'r_ank',
#                  'r_eye', 'l_eye',
#                  'r_ear', 'l_ear']
# ####????????
#     keys = ['nose', 'neck',
#                  'r_sho', 'r_elb', 'r_wri',
#                  'l_sho', 'l_elb', 'l_wri',
#                  'r_hip', 'r_knee', 'r_ank',
#                   'l_hip', 'l_knee', 'l_ank',
#                  'r_eye', 'l_eye',
#                  'r_ear', 'l_ear']
#
#
#     x = np.zeros(y.shape)
#     x[0:2] = y[2:4] ###??????????, ???????????????? ?????????????????? ?????????? ??????????
#     x[2:4] = y[0:2]
#     x[4:6] = y[18:20]#+
#     x[6:8] = y[20:22]
#     x[8:10] = y[22:24]
#     x[10:12] = y[6:8] #+
#     x[12:14] = y[8:10]
#     x[14:16] = y[10:12]
#     x[16:18] = y[12:14]
#     x[22:24] = y[24:26]
#     x[28:30] = y[28:30] #+
#     x[30:32] = y[30:32] #+
#     x[32:34] = y[32:34] #+
#     x[34:36] = y[34:36] #+


    return y
