#!/usr/bin/env python3.8

from typing import Callable, Tuple
import numpy as np
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import Rotation as R
from functools import partial
from number_works import project_vec_to_plane, normalize_vec
#import random

def get_orbit_path_func(center: np.ndarray, radius: float, pitch: float, angle: float, start: np.ndarray, start_time: float) -> Callable[[float], Tuple[np.ndarray, np.ndarray]]:
    forward = np.array([1, 0, 0])
    initial_look = R.from_rotvec(rotvec=np.array([0, 1, 0]) * pitch).apply(-forward)
    arc_direction = -1 if start[1] > 0 else 1
    arc_rots = R.from_rotvec([np.array([0, 0, 1]) * arc_direction * angle / 2, np.array([0, 0, 1]) * (-arc_direction) * angle/ 2])
    times = [start_time, 1.0]
    arc_slerp = Slerp(times, arc_rots)
    arc_start = arc_slerp(start_time).apply(initial_look) * radius + center
    init_path_dist = np.linalg.norm(arc_start -  start)
    init_path_ori = normalize_vec(arc_start -  start)
    def orbit_path_func(t: float):
        if t < start_time:
            look_from = init_path_ori * init_path_dist * t / start_time + start
        else:
            look_from = arc_slerp(t).apply(initial_look) * radius + center
        return look_from, normalize_vec(center - look_from)
    return orbit_path_func

def get_orbit_directions_func(half_rotate_by: float, start_from: np.ndarray) -> Callable[[float], np.ndarray]:
    # flip the initial direction around the x axis to get the end state
    forward = np.array([1, 0, 0])
    half_rot = R.from_rotvec(rotvec=np.array([0, 0, 1]) * -half_rotate_by)
    rotate_from = half_rot.apply(forward)
    ## Compute the offset value
    rotate_to = half_rot.inv().apply(forward)
    ### first check if the start_from vector is between rotate_from and rotate_to
    within_angle_cos = np.cos(np.pi - half_rotate_by)
    start_from_val = 0
    to_rotate_from_cos = np.dot(start_from, rotate_from)
    to_rotate_to_cos = np.dot(start_from, rotate_to)
    if to_rotate_from_cos < within_angle_cos or to_rotate_to_cos < within_angle_cos:
        # it's outside the two bounds
        start_from_val = 1 if to_rotate_to_cos > to_rotate_from_cos else 0
    else:
        start_from_val = np.arccos(to_rotate_from_cos) / 2 / half_rotate_by
    offset = start_from_val
    return get_orbit_directions_func_intl(rotate_from=rotate_from, rotate_by= 2 * half_rotate_by, offset=offset)

def get_orbit_directions_func_intl(rotate_from: np.ndarray, rotate_by: float, offset: float) -> Callable[[float], np.ndarray]:
    # rots = [R.identity, R.from_rotvec(np.array([0, 0, 1]) * rotate_by)]
    #flip = random.random() > 0.5
    flip = False
    rots = R.from_rotvec([[0, 0, 0], np.array([0, 0, 1]) * rotate_by]) if flip else R.from_rotvec([np.array([0, 0, 1]) * rotate_by, [0, 0, 0]]) # add a coin flip to introduce a bit of variation
    times = [0.0, 1.0] 
    slerp = Slerp(times, rots)
    mapping_func = lambda x: np.sin((x - 0.5) * np.pi) / 2 + 0.5
    def calc_orbit_directions(s: Slerp, t: float):
        rot_now = s(mapping_func(t + offset))
        return rot_now.apply(rotate_from)
    return partial(calc_orbit_directions, slerp)

def get_pan_focus_offset_func(focus_start: np.ndarray, cam_start: np.ndarray, total_offset: float, direction_plus: bool) -> Callable[[float], np.ndarray]:
    init_orientation = focus_start - cam_start
    # Project to the xz plane, so that it's pointing forward
    orientation = project_vec_to_plane(init_orientation, np.array([0, 1, 0]), normalize=True)
    def calc_focus_with_offset(t: float):
        focus = focus_start + t * total_offset * (1 if direction_plus else -1) * np.array([0, 1, 0])
        focus_orientation = np.concatenate((focus, orientation), axis=None)
        #print(f"focus is {focus}, orientation is {orientation}, concat is {focus_orientation}")
        return focus_orientation
    return calc_focus_with_offset

def get_pan_focus_func_two_hands(focus_left: np.ndarray, focus_right: np.ndarray, cam_start: np.ndarray):
    ## Start from the hand that's closer to the current camera position
    left2cam = np.linalg.norm(focus_left - cam_start)
    right2cam = np.linalg.norm(focus_right - cam_start)
    focus_start, focus_end = focus_left if left2cam < right2cam else focus_right, focus_right if left2cam < right2cam else focus_left
    focus_track = focus_end - focus_start
    focus_l2r = focus_right - focus_left
    #focus_l2r_dir = normalize_vec(focus_l2r)
    focus_l2r_dir_projected = project_vec_to_plane(focus_l2r, n=np.array([0, 0, 1]), normalize=False)
    look_ori = normalize_vec(np.cross(focus_l2r_dir_projected, np.array([0, 0, 1])))
    def cal_focus(t: float) -> np.ndarray:
        f = focus_start + focus_track * t
        focus_orientation = np.concatenate((f, look_ori), axis=None)
        return focus_orientation
    return cal_focus


    
    

