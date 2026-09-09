# ffplay /dev/video0

import cv2
import numpy as np
import json
from collections import deque
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter
import pandas as pd
import time
import requests
import math



## file location
can_h_fn = "./birdview/hummer_path/can_heading.txt"
gps_ref_fn = "./birdview/hummer_path/gps_ref_p2.txt"
gps_fn = "./birdview/hummer_path/gps.txt"
path_fn = "./birdview/hummer_path/pathx5_can_heading.txt"

v2spot_file_path = "vehicle2spot_path.txt"
v2spot_file_path_world = "vehicle2spot_path0.txt"
track_fn = "track_test.txt"

decision_point = 5 ## how many right point estimation valid before making path modification
LR_vary_thr = 0.5 # left/right lane agree in estimation
decision_point2 = 5
    

## vehicle

vehicle_width = 2.196 # in meter
vehicle_height = 1.976
vehicle_length = 4.999

lane_length = 5.0  # meters
lane_offset = 1.5  # right of target
lane_width = 0.2   # optional lane width

# spot_width = 2.74 # 9feet, but need to change based on tape distance
spot_width = 3.35 # 9feet, but need to change based on tape distance; new lane taped 3.0m
spot_length = 5.79

lane_tape_right = 2.54

############## 70 camera ##############

# fov
# hfov_deg = 70
# vfov_deg = 70 

# # fixed look-ahead distance
# distance = 0.25 

# # look-ahead distance by tilt degree and mount height and fov
# cam_height = 0.1
# tilt_deg = 20



############# 170 camera ##############

# camera matrix:
#  [[709.98699472   0.         683.50483975]
#  [  0.         710.25435411 369.28104341]
#  [  0.           0.           1.        ]]
# distortion coefficients:  [-0.32463572  0.11543686 -0.00232406 -0.00286906 -0.02013232]


# fov
# hfov_deg = 170 
# vfov_deg = 60




## edge & line detection
blur_thr = 2 # test good at 9, smaller resolution lower blur to 5-7
hough_thr = 100 # 100, smaller resolution lower to 60
angle_thr_deg = 85


## test streaming screen recording W,H
# 2592(H) x 1944(V) @ 15fps 170 Degree Fisheye Lens
# w = 1916
# h = 1072

## 1st test gourp of screen recording
# w = 2592
# h = 1944

# ## realtime
# w = 1920
# h = 1080 

w = 1280
h = 720 

w = 640
h = 480 

## 2nd test gourp of screen recording
# w = 1941
# h = 1076

mask_L = -3
mask_R = 3
mask_D = 1.0
mask_U = 10.0

# 3m wide close at 1m, 30m wide far at 15m
ROI_LB = (-5.5, 0.5)
ROI_LT = (-40.0, 10.0) 
ROI_RB = (3.5, 0.5)
ROI_RT = (10.0, 10.0)
valid_depth = [ROI_LB[1], ROI_LT[1]]

rtp_valid = False
rbp_valid = False
ltp_valid = False
lbp_valid = False

# fov test 170 cam 
hfov_deg = 107 # 107 calibrate by software
vfov_deg = 74.5

hfov_deg = 110 # 107 calibrate by software
vfov_deg = 70

## est by hand
hfov_deg = 94 # 107 calibrate by software
vfov_deg = 58

## without scale, 640x480
hfov_deg = 150 # 107 calibrate by software
vfov_deg = 70


# fov test 180 cam
# hfov_deg = 170 # since matrix tool not trustful 
# vfov_deg = 85



# --- Camera Calibration Parameters ---
# Adjust based on specific camera hardware
# focal_length = w * 0.7 #113 for 170 # w * 0.8 for 64 degree only       # Focal length in pixels
camera_height = 0.63 #0.7          # Height of camera from ground (meters)
pitch_angle = np.radians(15) ## 28 for test0205, 26 for left/right in 34 for test1.mp4 # 29 for off_close, far and 2 vid recording
wheel_dis = 1.80

# parking width 268 (each side 134), len 584
# wheel 180, 28 ea30

# measured:
# cam height 0.64
# pitch 15
# fov 115, 60

# 1st frame infront 77 
# 2nd frame infront 23 infront

# 3rd can heading parked in. 352.5
# 55.5,

  ## distortion 170 cam
K = np.array([
    [709.98699472,   0.0,         683.50483975],
    [  0.0,         710.25435411, 369.28104341],
    [  0.0,           0.0,           1.0        ]
])

dist_coeffs = np.array([
    -0.32463572,
     0.11543686,
    -0.00232406,
    -0.00286906,
    -0.02013232
])

#for 170 cam 640x480 res
K = np.array([[273.92842905, 0.0, 316.86198264], [ 0.0, 274.7565911, 236.44369128], [ 0.0, 0.0, 1.0 ]])
dist_coeffs = np.array([-0.351, 0.132, 0.0034, 0.0025, -0.024])

Det_heading_trigger = [5, 50]
Det_distance_trigger = [3, 8]

###########################



def plot_lane_history(history_data):
    """
    history_data: dictionary from tracker.get_history_list()
    containing 'left_start', 'left_end', 'right_start', 'right_end'
    """
    frames = np.arange(len(history_data['right_start']))
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    # --- Plot 1: Lateral (X) Movement ---
    for key, color, label in [
        ('left_start', 'lightgreen', 'L-Start'), ('left_end', 'darkgreen', 'L-End'),
        ('right_start', 'salmon', 'R-Start'), ('right_end', 'red', 'R-End')
    ]:
        data = history_data[key]
        # Extract X coordinates, handling None values
        x_vals = [pt[0] if pt else None for pt in data]
        ax1.plot(frames, x_vals, label=label, color=color, linewidth=2)

    ax1.set_title("Lateral (X) Position Change (Meters)")
    ax1.set_ylabel("Meters (Left - / Right +)")
    ax1.legend(loc='upper right', ncol=2)
    ax1.grid(True, alpha=0.3)

    # --- Plot 2: Longitudinal (Y) Movement ---
    for key, color, label in [
        ('left_start', 'lightgreen', 'L-Start'), ('left_end', 'darkgreen', 'R-Start'),
        ('right_start', 'salmon', 'R-Start'), ('right_end', 'red', 'R-End')
    ]:
        data = history_data[key]
        # Extract Y coordinates
        y_vals = [pt[1] if pt else None for pt in data]
        ax2.plot(frames, y_vals, label=label, color=color, linewidth=2)

    ax2.set_title("Longitudinal (Y) Distance Change (Meters)")
    ax2.set_xlabel("Frame Number")
    ax2.set_ylabel("Distance Forward (m)")
    ax2.legend(loc='center right')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('location_change.png')
    plt.close(fig) 
    
    # --- Top-Down Path Plot ---
    plt.figure(figsize=(8, 10)) # Increased size slightly to accommodate text

    # for key, color in [('left_start', 'Greens'), ('left_end', 'YlGn'), 
                       # ('right_start', 'Reds'), ('right_end', 'OrRd')]:
    for key, color in [('right_start', 'Reds')]:
        
        # Filter valid points and keep track of their original indices
        valid_indices = [i for i, pt in enumerate(history_data[key]) if pt is not None]
        data = np.array([history_data[key][i] for i in valid_indices])

        if len(data) > 0:
            # Plot the points
            plt.scatter(data[:,0], data[:,1], c=np.arange(len(data)), cmap=color, s=15)
            
            # Add index labels beside each point
            for i, idx in enumerate(valid_indices):
                # Coordinates for the text (with a tiny offset for readability)
                plt.text(data[i, 0] + 0.05, data[i, 1] + 0.05, str(idx), 
                         fontsize=8, alpha=0.7)

    plt.axvline(0, color='black', linestyle='--') 
    plt.title("Top-Down Trajectory with Frame Indices")
    plt.xlabel("Lateral (m)")
    plt.ylabel("Forward (m)")
    plt.xlim(-4, 4)
    plt.grid(True, alpha=0.3)
    plt.savefig('top_down_trajectory.png')
    plt.close()




def plot_pc_history(history_data, tag):

    # Extract data and filter out None values
    raw_data = history_data[tag]
    
    # Zip frames with data so we only plot valid points
    # pt[0] is Lateral (X), pt[1] is Forward (Y)
    valid_points = [(i, pt[0], pt[1]) for i, pt in enumerate(raw_data) if pt is not None]
    
    if not valid_points:
        print("No valid data to plot.")
        return

    frames, x_vals, y_vals = zip(*valid_points)

    # --- Plot 1: X and Y Position over Time ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Subplot A: Lateral (X)
    ax1.plot(frames, x_vals, label='Lateral (X)', color='blue', linewidth=2)
    ax1.set_title("Parking Center Position Change")
    ax1.set_ylabel("X (Meters: Left - / Right +)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right')

    # Subplot B: Forward (Y)
    ax2.plot(frames, y_vals, label='Forward (Y)', color='blue', linewidth=2)
    ax2.set_ylabel("Y (Meters: Distance to Center)")
    ax2.set_xlabel("Frame")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right')

    fig.tight_layout()
    fig.savefig('pc_location_change.png')
    plt.close(fig)   

    # --- Plot 2: Top-Down Path (Remains the same) ---
    fig = plt.figure(figsize=(8, 10))

    # 1. Filter data while preserving original indices
    valid_indices = [i for i, pt in enumerate(raw_data) if pt is not None]
    path_data = np.array([raw_data[i] for i in valid_indices])

    if len(path_data) > 0:
        # 2. Draw the scatter points
        scatter = plt.scatter(path_data[:, 0], path_data[:, 1],
                             c=np.arange(len(path_data)), cmap='Greens', s=15)
        
        # 3. Annotate each point with its frame index
        for i, idx in enumerate(valid_indices):
            # We add a small offset (+0.05) so the text doesn't sit on the dot
            plt.annotate(str(idx), 
                         (path_data[i, 0], path_data[i, 1]),
                         textcoords="offset points", 
                         xytext=(5, 5), 
                         fontsize=8, 
                         alpha=0.6)

    plt.axvline(0, color='black', linestyle='--')
    plt.title("Top-Down Trajectory (Path Map) with Frame Indices")
    plt.xlabel("Lateral (m)")
    plt.ylabel("Forward (m)")
    plt.xlim(-4, 4)
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('pc_top_down_trajectory.png')
    plt.close(fig)


def lane_history_filtered(history_data):
    filtered_history = {}
    
    for key in ['left_start', 'left_end', 'right_start', 'right_end', 'park_cen', 'park_cen_veh', 'park_cen_world']:
        raw_pts = history_data[key]
        
        # 1. Handle Nones (Interpolate so the filter doesn't break)
        x_raw = np.array([p[0] if p else np.nan for p in raw_pts])
        y_raw = np.array([p[1] if p else np.nan for p in raw_pts])
        
        # Simple linear interpolation for missing frames
        x_clean = pd.Series(x_raw).interpolate(method='linear')
        y_clean = pd.Series(y_raw).interpolate(method='linear')

        # 2. Apply Savitzky-Golay Smoothing
        # Window size 15 is usually good for 30fps video
        filtered_history[key] = list(zip(
            savgol_filter(x_clean, 15, 2),
            savgol_filter(y_clean, 15, 2)
        ))

    return filtered_history
    
#########################


def est_distance_grid(frame):
    h, w = frame.shape[:2]
    
    # Draw Depth Lines (Lane width markers)
    # Using the standard 9ft (~2.7m) parking width as the outer bounds
    for xw in [mask_L, 0.0, mask_R]: 
        pts = []
        for yw in np.linspace(1.0, mask_U, 20):
            # pt = project_point_with_fl(xw, yw)
            pt = project_point_with_fov_to_img(xw, yw)
            if pt and 0 <= pt[0] < w and 0 <= pt[1] < h:
                pts.append(pt)
        
        if len(pts) > 1:
            cv2.polylines(frame, [np.array(pts)], False, (0, 255, 255), 1)

            # Labeling
            label_pt = pts[0]
            cv2.putText(frame, f"{xw}m", (label_pt[0]-25, label_pt[1]+30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    ## tire
    for xw in [-wheel_dis/2, wheel_dis/2]: 
        pts = []
        for yw in np.linspace(0.5, 0.6, 20):
            # pt = project_point_with_fl(xw, yw)
            pt = project_point_with_fov_to_img(xw, yw)
            if pt and 0 <= pt[0] < w and 0 <= pt[1] < h:
                pts.append(pt)
        
        if len(pts) > 1:
            cv2.polylines(frame, [np.array(pts)], False, (0, 0, 255), 2)

            # Labeling
            label_pt = pts[1]
            cv2.putText(frame, f"wheel", (label_pt[0]-25, label_pt[1]-40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # lane
    for xw in [-spot_width/2, spot_width/2]: 
        pts = []
        for yw in np.linspace(1.0, 5, 20):
            # pt = project_point_with_fl(xw, yw)
            pt = project_point_with_fov_to_img(xw, yw)
            if pt and 0 <= pt[0] < w and 0 <= pt[1] < h:
                pts.append(pt)
        
        if len(pts) > 1:
            cv2.polylines(frame, [np.array(pts)], False, (255, 255, 0), 2)

            # Labeling
            label_pt = pts[0]
            cv2.putText(frame, f"lane", (label_pt[0]-25, label_pt[1]+30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)



    # Draw Distance Markers (1m to 9m)
    for yw in range(1, int(mask_U)+1):
        pts = []
        for xw in np.linspace(mask_L, mask_R, 10):
            # pt = project_point_with_fl(xw, yw)
            pt = project_point_with_fov_to_img(xw, yw)
            if pt and 0 <= pt[0] < w and 0 <= pt[1] < h:
                pts.append(pt)
        
        if len(pts) > 1:
            # Red for close-range (<2m), Yellow otherwise
            # color = (0, 0, 255) if yw <= 2 else (0, 255, 255)
            color = (0, 255, 255)
            cv2.polylines(frame, [np.array(pts)], False, color, 1)
            
            # Labeling
            label_pt = pts[0]
            cv2.putText(frame, f"{yw}m", (label_pt[0]-30, label_pt[1]+5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Generate final mask and corners
    # mask, corners_10m = get_10m_mask(h, w)
    mask, corners_10m = get_15m_mask(h, w)
    # mask, corners_10m = get_15m_mask_oov(h, w)

    return frame, np.array(corners_10m, dtype=np.int32), mask

# Coordinate Projection from dis to imge pixel
def project_point_with_fl(xw, yw):
    """
    xw: Lateral distance from center (meters)
    yw: Longitudinal distance forward (meters)
    """
    # Transform Ground (xw, yw, 0) to Camera Coordinates (Pitched Down)
    # Z is forward, Y is down, X is right
    z_cam = yw * np.cos(pitch_angle) + camera_height * np.sin(pitch_angle)
    y_cam = camera_height * np.cos(pitch_angle) - yw * np.sin(pitch_angle)
    x_cam = xw
    
    # Clipping: Only project points in front of the camera
    if z_cam <= 0.1: 
        return None
    
    # Pinhole projection to pixel coordinates
    u = int((x_cam * focal_length / z_cam) + w / 2)
    v = int((y_cam * focal_length / z_cam) + h / 2)
    
    return (u, v)



def project_point_with_fov_to_img(xw, yw):
    """
    xw: lateral distance (meters, +right)
    yw: forward distance on ground (meters)
    """

    # Convert FOVs to radians
    fov_x = np.deg2rad(hfov_deg)   # horizontal FOV
    fov_y = np.deg2rad(vfov_deg)    # vertical FOV

    # Compute focal lengths in pixels
    fx = (w / 2) / np.tan(fov_x / 2)
    fy = (h / 2) / np.tan(fov_y / 2)

    # Ground → Camera coordinates (camera pitched down)
    z_cam = yw * np.cos(pitch_angle) + camera_height * np.sin(pitch_angle)
    y_cam = camera_height * np.cos(pitch_angle) - yw * np.sin(pitch_angle)
    x_cam = xw

    # Only project points in front of camera
    # if z_cam <= 0.1:
    #     return None

    # Perspective projection
    u = int(fx * x_cam / z_cam + w / 2)
    v = int(fy * y_cam / z_cam + h / 2)

    # Image bounds check
    # if not (0 <= u < w and 0 <= v < h):
    #     return None

    return (u, v)


def get_10m_mask(h, w):
        mask = np.zeros((h, w), dtype=np.uint8)
        # 19ft x 9ft area (~5.8m x 2.7m) or your custom 10m zone
        corners_world = [(-2.0, 2.0), (2.0, 2.0), (2.0, 10.0), (-2.0, 10.0)]
        
        pixel_pts = []
        for xw, yw in corners_world:
            # pt = project_point_with_fl(xw, yw)
            pt = project_point_with_fov_to_img(xw, yw)
            if pt: pixel_pts.append(pt)
            
        if len(pixel_pts) >= 3:
            cv2.fillPoly(mask, [np.array(pixel_pts, dtype=np.int32)], 255)
        return mask, pixel_pts


def get_15m_mask(h, w):
        mask = np.zeros((h, w), dtype=np.uint8)
        # 19ft x 9ft area (~5.8m x 2.7m) or your custom 10m zone
        corners_world = [ROI_LB, ROI_RB, ROI_RT, ROI_LT]
        
        pixel_pts = []
        for xw, yw in corners_world:
            # pt = project_point_with_fl(xw, yw)
            pt = project_point_with_fov_to_img(xw, yw)
            if pt: pixel_pts.append(pt)
            
        if len(pixel_pts) >= 3:
            cv2.fillPoly(mask, [np.array(pixel_pts, dtype=np.int32)], 255)
        return mask, pixel_pts




def sutherland_hodgman_clip(polygon, img_w, img_h):
    """
    Clip a polygon to image boundaries (0,0)-(img_w-1, img_h-1)
    polygon: list of [u,v]
    Returns: clipped polygon points
    """
    def clip_edge(polygon, edge):
        clipped = []
        for i in range(len(polygon)):
            curr = polygon[i]
            prev = polygon[i-1]
            
            if edge(curr):
                if not edge(prev):
                    # Intersection
                    inter = line_intersect(prev, curr, edge)
                    if inter is not None:
                        clipped.append(inter)
                clipped.append(curr)
            elif edge(prev):
                inter = line_intersect(prev, curr, edge)
                if inter is not None:
                    clipped.append(inter)
        return clipped

    def line_intersect(p1, p2, edge_func):
        # Find intersection of line segment p1-p2 with boundary defined by edge_func
        x1, y1 = p1
        x2, y2 = p2

        dx = x2 - x1
        dy = y2 - y1

        if dx == 0 and dy == 0:
            return None

        # We iterate over 4 borders
        for border in ['left', 'right', 'top', 'bottom']:
            if border == 'left':
                x = 0
                if dx != 0:
                    t = (x - x1)/dx
                    if 0 <= t <= 1:
                        y = y1 + t*dy
                        if 0 <= y <= img_h-1:
                            return [x, y]
            elif border == 'right':
                x = img_w-1
                if dx != 0:
                    t = (x - x1)/dx
                    if 0 <= t <= 1:
                        y = y1 + t*dy
                        if 0 <= y <= img_h-1:
                            return [x, y]
            elif border == 'top':
                y = 0
                if dy != 0:
                    t = (y - y1)/dy
                    if 0 <= t <= 1:
                        x = x1 + t*dx
                        if 0 <= x <= img_w-1:
                            return [x, y]
            elif border == 'bottom':
                y = img_h-1
                if dy != 0:
                    t = (y - y1)/dy
                    if 0 <= t <= 1:
                        x = x1 + t*dx
                        if 0 <= x <= img_w-1:
                            return [x, y]
        return None

    # Define inside functions for each border
    inside_left   = lambda p: p[0] >= 0
    inside_right  = lambda p: p[0] <= img_w-1
    inside_top    = lambda p: p[1] >= 0
    inside_bottom = lambda p: p[1] <= img_h-1

    clipped = polygon
    for edge in [inside_left, inside_right, inside_top, inside_bottom]:
        clipped = clip_edge(clipped, edge)
        if not clipped:
            break
    return np.array(clipped, dtype=np.int32)

def get_15m_mask_oov(h, w):
    mask = np.zeros((h, w), dtype=np.uint8)
    
    # World corners of ROI
    corners_world = [ROI_LB, ROI_RB, ROI_RT, ROI_LT]
    
    pixel_pts = []
    for xw, yw in corners_world:
        pt = project_point_with_fov_to_img(xw, yw)
        if pt is not None:
            pixel_pts.append(list(pt))
        else:
            # Keep as is, will be clipped
            pixel_pts.append(list(pt) if pt else [np.nan, np.nan])
    
    # Remove NaNs for clipping algorithm
    pixel_pts = [p for p in pixel_pts if not np.isnan(p[0])]
    
    if len(pixel_pts) >= 3:
        clipped_pts = sutherland_hodgman_clip(pixel_pts, w, h)
        cv2.fillPoly(mask, [clipped_pts], 255)
    else:
        clipped_pts = np.array([], dtype=np.int32)
    
    return mask, clipped_pts




def pixel_to_world(u, v):
    """
    Exact inverse of project_point_with_fov_to_img
    Assumes the point lies on the ground plane
    """

    cx = w / 2
    cy = h / 2

    # Focal lengths
    fov_x = np.deg2rad(hfov_deg)
    fov_y = np.deg2rad(vfov_deg)

    fx = (w / 2) / np.tan(fov_x / 2)
    fy = (h / 2) / np.tan(fov_y / 2)

    # Normalized camera ray
    x_cam = (u - cx) / fx
    y_cam = (v - cy) / fy
    z_cam = 1.0

    r = y_cam / z_cam

    cp = np.cos(pitch_angle)
    sp = np.sin(pitch_angle)

    denom = (r * cp + sp)
    if denom <= 0:
        return None  # above horizon

    # Solve forward distance
    yw = camera_height * (cp - r * sp) / denom

    if yw <= 0:
        return None

    # Recover z_cam scale
    zc = yw * cp + camera_height * sp

    # Lateral distance
    xw = x_cam * zc

    return xw, yw, 0.0



def find_anchor_points(endpoints):
    real_world_coords = {}

    width, height = w, h

    for side in ['left', 'right']:
        if endpoints[side] is not None:
            start_px = endpoints[side]['start']
            end_px = endpoints[side]['end']
            
            # Map Start Point (usually the point closer to the car)
            rw_start = pixel_to_world(start_px[0], start_px[1])
            
            # Map End Point (usually the point further away)
            rw_end = pixel_to_world(end_px[0], end_px[1])
            
            real_world_coords[side] = {
                'start_meters': rw_start,
                'end_meters': rw_end
            }
            print("---", real_world_coords)
        else:
            real_world_coords[side] = None

    
    return real_world_coords


class LaneTracker:
    def __init__(self, max_history=30):
        # We use deque with maxlen to automatically remove old frames
        self.history = {
            'left_start': deque(maxlen=max_history),
            'left_end':   deque(maxlen=max_history),
            'right_start': deque(maxlen=max_history),
            'right_end':  deque(maxlen=max_history),
            'park_cen':  deque(maxlen=max_history),
            'park_cen_veh':  deque(maxlen=max_history),
            'park_cen_world':  deque(maxlen=max_history),
            'park_heading_world': deque(maxlen=max_history),
            'park_cen_l':  deque(maxlen=max_history),
            'park_cen_veh_l':  deque(maxlen=max_history),
            'park_cen_world_l':  deque(maxlen=max_history),
            'park_heading_world_l': deque(maxlen=max_history),
            'veh_gps':  deque(maxlen=max_history),
            'veh_heading':  deque(maxlen=max_history)
        }

        self.stable_tag = False
        self.start_gps = None
        self.current_gps = None
        self.current_heading = None
        self.trust_range_2v = [0, 0]
        self.target_static = None


    def get_static_target(self):

        with open(path_fn, 'r') as f:
            line = f.readline()
        path = json.loads(line)
        target = path[-1]

        self.target_static = target


    def update(self, real_world_coords, pc, pc_l, pc2veh, pc2veh_l, pc2world, pc2world_l, gps, heading, ph, ph_l):
        
        self.history['veh_gps'].append(gps)
        self.history['veh_heading'].append(heading)
        
        self.history['park_cen'].append(pc)
        self.history['park_cen_veh'].append(pc2veh)
        self.history['park_cen_world'].append(pc2world)
        self.history['park_heading_world'].append(ph)

        self.history['park_cen_l'].append(pc_l)
        self.history['park_cen_veh_l'].append(pc2veh_l)
        self.history['park_cen_world_l'].append(pc2world_l)
        self.history['park_heading_world_l'].append(ph_l)
        

        for side in ['left', 'right']:
            data = real_world_coords.get(side)
            
            if data and data['start_meters'] and data['end_meters']:
                # Save actual coordinates
                self.history[f'{side}_start'].append(data['start_meters'])
                self.history[f'{side}_end'].append(data['end_meters'])
                
            else:
                # Append None to keep the timeline consistent, 
                # or skip if you only want valid detections.
                self.history[f'{side}_start'].append(None)
                self.history[f'{side}_end'].append(None)

        self.current_gps = gps
        self.current_heading = heading


    def filter_win(self, alpha=0.3):
        """
        Applies exponential smoothing to the world coordinates and heading.
        Formula: s_t = alpha * x_t + (1 - alpha) * s_{t-1}
        
        Args:
            alpha (float): Smoothing factor (0 to 1). 
                           Lower = smoother but more lag.
                           Higher = more responsive but noisier.
        """
        # 1. Get the most recent valid points
        pc_history = [pt for pt in self.history['park_cen_world'] if pt is not None]
        h_history = [h for h in self.history['veh_heading'] if h is not None]

        if len(pc_history) < 2:
            return None, None

        # 2. Initialize smoothed values with the first valid data point
        smooth_pc = np.array(pc_history[0])
        smooth_h = h_history[0]

        # 3. Iteratively apply the EMA filter through the valid history
        for i in range(1, len(pc_history)):
            current_pc = np.array(pc_history[i])
            smooth_pc = alpha * current_pc + (1 - alpha) * smooth_pc
            
        for i in range(1, len(h_history)):
            current_h = h_history[i]
            # Handle heading: simple EMA works if angles don't jump 360->0
            smooth_h = alpha * current_h + (1 - alpha) * smooth_h

        return smooth_pc.tolist(), float(smooth_h)



    def check_consistance(self, window_size=10, std_threshold=0.15):
  
        # 1. Get the most recent N points from history, filtering out None values
        recent_data = list(self.history['park_cen_world'])[-window_size:]
        valid_points = [pt for pt in recent_data if pt is not None]

        # 2. We need a minimum number of valid frames to decide on stability
        if len(valid_points) < (window_size // 2):
            self.stable_tag = False
            return False

        # 3. Calculate Standard Deviation for X (Lateral) and Y (Forward)
        points_array = np.array(valid_points)
        std_x = np.std(points_array[:, 0])
        std_y = np.std(points_array[:, 1])

        print("std. left/right", std_x)

        # 4. Update the stable_tag
        # If the points are staying within a tight circle, it's consistent.
        if std_x < std_threshold and std_y < std_threshold:
            self.stable_tag = True
        else:
            self.stable_tag = False

        return self.stable_tag



    def get_history_list(self):
        """Returns the history as a standard Python list for saving/JSON"""
        return {key: list(val) for key, val in self.history.items()}

    def get_recent_est(self, v2w_heading, offset, window_size=10):
        
        recent_data_w = list(self.history['park_cen_world'])[-window_size:]
        recent_data_c = list(self.history['park_cen'])[-window_size:]
        
        val = None
        for i in range(window_size):
            if recent_data_c[-i]:
                val = recent_data_c[-i]
                break

        if not val:
            return None

        world_x, world_y = parking_center_to_vehicle(val[0], val[1], v2w_heading) ## doesn't matter with the sensed heading
        world_x0, world_y0 = target_wrt_initial_point(world_x, world_y, offset)
        pc2world = [world_x0, world_y0]
        
        print("track recent cen", recent_data_w, recent_data_c)
        print("last one using stopped sensed heading transfer again", pc2world)

        return pc2world


    def get_recent_est_by_both(self, v2w_heading, offset, window_size=10):

        recent_data_w = list(self.history['park_cen_world'])[-window_size:]
        recent_data_c = list(self.history['park_cen'])[-window_size:]

        recent_data_wl = list(self.history['park_cen_world_l'])[-window_size:]
        recent_data_cl = list(self.history['park_cen_l'])[-window_size:]
        
        val1 = None
        for i in range(window_size):
            if recent_data_c[-i]:
                val1 = recent_data_c[-i]
                break

        if not val1:
            return None

        val2 = None
        for i in range(window_size):
            if recent_data_cl[-i]:
                val2 = recent_data_cl[-i]
                break

        if not val2:
            return None

        val = [(val1[0]+val2[0])/2, (val1[1]+val2[1])/2]

        world_x, world_y = parking_center_to_vehicle(val[0], val[1], v2w_heading) ## doesn't matter with the sensed heading
        world_x0, world_y0 = target_wrt_initial_point(world_x, world_y, offset)
        pc2world = [world_x0, world_y0]
        
        print("track recent cen R+L", recent_data_w, recent_data_c, recent_data_wl, recent_data_cl)
        print("last one using stopped sensed heading transfer again R+L", pc2world)

        return pc2world


def edge_det(frame, hough_thr, blur_thr):

    # h, w = frame.shape[:2]
    # new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1)
    # undistorted = cv2.undistort(frame, K, D, None, new_K)
    # gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (blur_thr,blur_thr), 0)
    edges = cv2.Canny(blurred, 50, 150)
    # thr: minimum number of votes needed to detect a line
    lines = cv2.HoughLines(edges, 1, np.pi/180, hough_thr)
    return edges, lines




def edge_det_mask_range(frame, hough_thr, blur_thr, mask):

    # h, w = frame.shape[:2]
    # new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1)
    # undistorted = cv2.undistort(frame, K, D, None, new_K)
    # gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
    
    # 1. Standard Gray and Blur
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    k_size = blur_thr if blur_thr % 2 != 0 else blur_thr + 1
    blurred = cv2.GaussianBlur(gray, (k_size, k_size), 0)

    # 2. Apply mask to the blurred image 
    # This zeroes out everything outside the 10m zone
    masked_input = cv2.bitwise_and(blurred, mask)

    # 3. Canny Edge Detection
    edges = cv2.Canny(masked_input, 50, 150)

    # 4. REMOVE THE MASK BORDER
    # Canny will detect the edge of the mask itself. 
    # We shrink the mask slightly (erode) to "cut off" the fake border edges.
    kernel = np.ones((5, 5), np.uint8)
    eroded_mask = cv2.erode(mask, kernel, iterations=1)
    final_edges = cv2.bitwise_and(edges, eroded_mask)

    # 5. Line Detection
    lines = cv2.HoughLines(final_edges, 1, np.pi/180, hough_thr)
    
    return final_edges, lines

def draw_all_hough_lines(frame, lines, color=(255, 255, 255), thickness=1, alpha=0.4):
    """
    Draws every line detected by the Hough transform with transparency.
    """
    if lines is None:
        return frame

    overlay = frame.copy()

    for line in lines:
        rho, theta = line[0]

        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho

        x1 = int(x0 + 2000 * (-b))
        y1 = int(y0 + 2000 * (a))
        x2 = int(x0 - 2000 * (-b))
        y2 = int(y0 - 2000 * (a))

        cv2.line(overlay, (x1, y1), (x2, y2), color, thickness)

    # Blend overlay with original frame
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

    return frame




def add_edge_on_img(frame, edge):
    output = frame.copy()
    edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    edges_color[:, :, 0] = 0   # remove blue
    edges_color[:, :, 1] = 0   # remove green
    output = cv2.addWeighted(output, 1.0, edges_color, 0.7, 0)
    return output


def add_lanes_on_img(frame, ll, rl, mask, mask_pts):
    """
    mask_pts: pixel corners from draw_distance_grid [BL, BR, TR, TL]
    Returns: (frame, endpoints_dict)
    """
    line_layer = np.zeros_like(frame)
    endpoints = {"left": None, "right": None}
    
    # Define the vertical bounds of the mask
    y_bottom = np.max(mask_pts[:, 1]) # Bottom of image/mask
    y_top = np.min(mask_pts[:, 1])    # Top of mask (10m away)

    for i, (best_line, color) in enumerate([(ll, (0, 255, 0)), (rl, (255, 0, 0))]):
        label = "left" if i == 0 else "right"
        
        if best_line:
            rho, theta = best_line
            cos_t, sin_t = np.cos(theta), np.sin(theta)
            
            # Avoid division by zero for horizontal-ish lines
            if abs(cos_t) < 0.001: continue 

            # Calculate intersection points at the mask's Y-boundaries
            # Formula: x = (rho - y * sin(theta)) / cos(theta)
            x_bottom = int((rho - y_bottom * sin_t) / cos_t)
            x_top = int((rho - y_top * sin_t) / cos_t)
            
            p1 = (x_bottom, y_bottom)
            p2 = (x_top, y_top)
            
            # Store endpoints
            endpoints[label] = {"start": p1, "end": p2}
            
            # Draw the line on the overlay
            cv2.line(line_layer, p1, p2, color, 8, cv2.LINE_AA)

    # Apply the mask and blend
    masked_lines = cv2.bitwise_and(line_layer, line_layer, mask=mask)
    cv2.addWeighted(frame, 1.0, masked_lines, 1.0, 0, frame)

    return frame, endpoints


# find the accurate end point within mask
def add_lanes_on_img_with_endpoints(frame, ll, rl, mask, mask_pts, edge_map, conn_thr=1, min_len_thr=50):
    """
    edge_map: The binary image from edge_det_mask_range (final_edges)
    """
    line_layer = np.zeros_like(frame)
    endpoints = {"left": None, "right": None}
    
    y_bottom = np.max(mask_pts[:, 1])
    y_top = np.min(mask_pts[:, 1])

    for i, (best_line, color) in enumerate([(ll, (0, 255, 0)), (rl, (255, 0, 0))]):
        label = "left" if i == 0 else "right"
        if not best_line: continue
        
        rho, theta = best_line
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        if abs(cos_t) < 0.001: continue 

        # 1. Sample along the line to find where edge pixels actually are
        # We walk from bottom to top
        active_points = []
        for y in range(int(y_bottom), int(y_top), -2): # step -2 for speed
            x = int((rho - y * sin_t) / cos_t)
            
            # Bound check for image width
            if 0 <= x < edge_map.shape[1]:
                # 2. Search window: check if there's an edge pixel nearby (±5px)
                try:
                    window = edge_map[y, max(0, x-conn_thr):min(edge_map.shape[1], x+conn_thr)]
                    if np.any(window > 0):
                        active_points.append((x, y))
                except:
                    pass
            
        # print(active_points)
        # 3. Filter for the longest continuous segment
        # If we found enough points, define the new start/end
        if len(active_points) > min_len_thr:
            # Finding the "extremes" of the detected edge clusters
            p_start = active_points[0]  # Closest to bottom
            p_end = active_points[-1]   # Furthest away

            rw_start = pixel_to_world(p_start[0], p_start[1])
            rw_end = pixel_to_world(p_end[0], p_end[1])
            if label=='right':
                global rbp_valid, rtp_valid 
                if rw_start[1]>valid_depth[0]+0.2: # 0.2m above ROI bottom (closer more trustful)
                    rbp_valid = True
                    # print("** btm point in view", rw_start, ROI_RB)
                else:
                    rbp_valid = False

                if rw_end[1]<valid_depth[1]-5: # 2m below ROI top
                    rtp_valid = True
                    # print("** top point in view", rw_end, ROI_RT)
                else:
                    rtp_valid = False

            if label=='left':
                global lbp_valid, ltp_valid 
                if rw_start[1]>valid_depth[0]+0.2: # 0.2m above ROI bottom (closer more trustful)
                    lbp_valid = True
                    # print("** btm point in view", rw_start, ROI_RB)
                else:
                    lbp_valid = False

                if rw_end[1]<valid_depth[1]-5: # 2m below ROI top
                    ltp_valid = True
                    # print("** top point in view", rw_end, ROI_RT)
                else:
                    ltp_valid = False
                
            endpoints[label] = {"start": p_start, "end": p_end}
            
            # 4. Draw only the segment where edges were actually found
            cv2.line(line_layer, p_start, p_end, color, 4, cv2.LINE_AA)

    if not endpoints["right"]:
        rtp_valid = False
        rbp_valid = False
    if not endpoints["left"]:
        ltp_valid = False
        lbp_valid = False


    # Blend with frame
    masked_lines = cv2.bitwise_and(line_layer, line_layer, mask=mask)
    cv2.addWeighted(frame, 1.0, masked_lines, 1.0, 0, frame)

    return frame, endpoints


def add_lanes_on_img_with_endpoints_seg_check(frame, ll, rl, mask, mask_pts, edge_map, conn_thr=2, min_len_thr=50):
    """
    Improved lane segment detection using a density check to prevent 
    endpoints from jumping to noise at the ROI boundaries.
    """
    line_layer = np.zeros_like(frame)
    endpoints = {"left": None, "right": None}
    
    # Define vertical bounds from the mask
    y_bottom = int(np.max(mask_pts[:, 1]))
    y_top = int(np.min(mask_pts[:, 1]))

    for i, (best_line, color) in enumerate([(ll, (0, 255, 0)), (rl, (255, 0, 0))]):
        label = "left" if i == 0 else "right"
        if not best_line: continue
        
        rho, theta = best_line
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        if abs(cos_t) < 0.001: continue 

        active_points = []
        # We search from bottom to top to find the physical start of the line
        # Step -2 for performance; search window helps bridge gaps

        h, w = edge_map.shape[:2]

        y_bottom = min(y_bottom, h - 1)
        y_top = max(y_top, 0)

        for y in range(y_bottom, y_top, -2):
            x = int((rho - y * sin_t) / cos_t)
            
            if 0 <= x < edge_map.shape[1]:
                # Define a localized window on the edge map
                x_min = max(0, x - conn_thr)
                x_max = min(edge_map.shape[1], x + conn_thr)
                window = edge_map[y, x_min:x_max]
                
                if np.any(window > 0):
                    # --- NOISE FILTERING ---
                    # Only accept points if there are other edges nearby vertically
                    # This prevents a single noise speck from being the start point
                    check_y = y - 4 # Look slightly ahead
                    check_x = int((rho - check_y * sin_t) / cos_t)
                    
                    if 0 <= check_x < edge_map.shape[1]:
                        check_win = edge_map[check_y, max(0, check_x-conn_thr):min(edge_map.shape[1], check_x+conn_thr)]
                        if np.any(check_win > 0) or len(active_points) > 0:
                            active_points.append((x, y))

        # Check if the detected segment meets the minimum length requirement
        if len(active_points) > min_len_thr:
            # 1. Use small internal offsets to avoid boundary noise
            # Instead of the absolute first/last, take the 3rd and 3rd-to-last
            p_start = active_points[2]  
            p_end = active_points[-3]   

            # 2. World Coordinate Validation (Consistency Checks)
            rw_start = pixel_to_world(p_start[0], p_start[1])
            rw_end = pixel_to_world(p_end[0], p_end[1])
            
            if label=='right':
                global rbp_valid, rtp_valid 
                if rw_start[1]>valid_depth[0]+0.2: # 0.2m above ROI bottom (closer more trustful)
                    rbp_valid = True
                    # print("** btm point in view", rw_start, ROI_RB)
                else:
                    rbp_valid = False

                if rw_end[1]<valid_depth[1]-5: # 2m below ROI top
                    rtp_valid = True
                    # print("** top point in view", rw_end, ROI_RT)
                else:
                    rtp_valid = False

            if label=='left':
                global lbp_valid, ltp_valid 
                if rw_start[1]>valid_depth[0]+0.2: # 0.2m above ROI bottom (closer more trustful)
                    lbp_valid = True
                    # print("** btm point in view", rw_start, ROI_RB)
                else:
                    lbp_valid = False

                if rw_end[1]<valid_depth[1]-5: # 2m below ROI top
                    ltp_valid = True
                    # print("** top point in view", rw_end, ROI_RT)
                else:
                    ltp_valid = False
            
            endpoints[label] = {"start": p_start, "end": p_end}
            
            # 3. Draw the validated segment
            cv2.line(line_layer, p_start, p_end, color, 8, cv2.LINE_AA)

    # Fallback for missing right line
    if not endpoints["right"]:
        rtp_valid = False
        rbp_valid = False
    if not endpoints["left"]:
        ltp_valid = False
        lbp_valid = False

    # Blend the layer with the original frame
    masked_lines = cv2.bitwise_and(line_layer, line_layer, mask=mask)
    cv2.addWeighted(frame, 1.0, masked_lines, 1.0, 0, frame)

    return frame, endpoints


def put_text_bg(img, text, org, font, font_scale, text_color, bg_color, thickness=2, padding=5):
    (w, h), baseline = cv2.getTextSize(text, font, font_scale, thickness)

    x, y = org
    cv2.rectangle(img, (x - padding, y - h - padding), (x + w + padding, y + baseline + padding), bg_color, -1)
    cv2.putText(frame, text, (x, y), font, font_scale, text_color, thickness)
        


def find_LR2(lines, width, height, angle_thr_deg):
    close_left = None
    close_right = None
    # img_center = width//2
    img_center = int((width)/2)
    eval_y = height//2
    max_x_left = -float('inf')
    min_x_right = float('inf')

    lower_b = np.deg2rad(angle_thr_deg)
    upper_b = np.pi-np.deg2rad(angle_thr_deg)

    if lines is not None:
        for line in lines:
            rho, theta = line[0]
            # vertical is 0, horizental is pi/2 (1.57), dismiss 
            if lower_b < theta < upper_b:
                continue

            cos_t = np.cos(theta)
            sin_t = np.sin(theta)

            if abs(cos_t)<0.01: continue # avoid 0
            x_at_eval_y = (rho-eval_y*sin_t)/cos_t

            if x_at_eval_y<img_center:
                if x_at_eval_y>max_x_left:
                    max_x_left = x_at_eval_y
                    close_left = (rho, theta)
            else:
                if x_at_eval_y<min_x_right:
                    min_x_right = x_at_eval_y
                    close_right = (rho, theta)

    # middle of two line
    mid_p = None
    error_pixel = 1000
    err_heading = 1000
    if close_left and close_right:
        l_rho, l_theta = close_left
        r_rho, r_theta = close_right
        x_left = (l_rho-eval_y*np.sin(l_theta))/np.cos(l_theta)
        x_right = (r_rho-eval_y*np.sin(r_theta))/np.cos(r_theta)
        mid_p = (x_left+x_right)/2

        error_pixel = mid_p-img_center
        
        deg_l = norm_theta(l_theta)
        deg_r = norm_theta(r_theta)
        err_heading = (deg_l+deg_r)/2 # vertical is zero, otherwise, in radians

    if close_right:
        print("++ LR close", close_right)
    
    return close_left, close_right, mid_p, error_pixel, err_heading


def find_PL(lines, width, height, angle_thr_deg):
    """
    Finds the two lines closest to the image center at the horizontal midline.
    """
    close_right = None
    
    eval_y = height / 2  # The horizontal line where we check for intersection
    
    # Initialize trackers for the closest X-coordinates to the center
    min_x_right = float('inf')   # Closest to center from the right (smallest X > center)

    # Thresholds to ignore horizontal-ish lines (around pi/2 or 90 degrees)
    lower_b = np.deg2rad(angle_thr_deg)
    upper_b = np.pi - np.deg2rad(angle_thr_deg)

    if lines is not None:
        for line in lines:
            rho, theta = line[0]

            # 1. Filter out horizontal lines that don't represent lane boundaries
            # In Hough space, vertical is 0, horizontal is pi/2.
            if lower_b < theta < upper_b:
                continue

            cos_t = np.cos(theta)
            sin_t = np.sin(theta)

            # 2. Avoid division by zero for perfectly horizontal lines
            if abs(cos_t) < 1e-6: 
                continue 

            # 3. Calculate X intersection at the middle of the image height
            # Formula derived from: rho = x*cos(theta) + y*sin(theta)
            x_at_eval_y = (rho - eval_y * sin_t) / cos_t

            # 4. Identify lines closest to the center point
            if x_at_eval_y < min_x_right:
                min_x_right = x_at_eval_y
                close_right = (rho, theta)
    
    if close_right:
        print("+++ PL close", close_right)
    
    return close_right


def norm_theta(theta):
    deg = np.rad2deg(theta)
    if deg>90:
        deg -= 180
    return deg



# fixed calibrated distance
def dyn_adj_dis_simple(hfov_deg, img_width, distance, error_pixel):
    
    # focal length in pixels
    hfov_rad = np.deg2rad(hfov_deg)
    fx = img_width / (2 * np.tan(hfov_rad / 2))

    # Horizontal adjustment in meters
    x_adjust_m = (error_pixel / fx) * distance

    return x_adjust_m




## distance estimate by cam fov, mounting loc, heading
def dyn_adj_dis(hfov_deg, vfov_deg, tilt_deg, cam_height, img_width, img_height, error_pixel):
    
    ## estimate cam to target distance respecting to mounting height and tilt angle
    vfov_rad = np.deg2rad(vfov_deg)
    fy = cam_height / (2 * np.tan(vfov_rad / 2))

    cy = cam_height / 2
    alpha = np.arctan((img_height - cy) / fy)

    pitch = np.deg2rad(tilt_deg)
    phi = pitch + alpha
    distance = cam_height / np.tan(phi)
    print("est Z", distance)
    
    # focal length in pixels
    hfov_rad = np.deg2rad(hfov_deg)
    fx = img_width / (2 * np.tan(hfov_rad / 2))

    # Horizontal adjustment in meters
    x_adjust_m = (error_pixel / fx) * distance

    return x_adjust_m


def add_deb_info(frame, mid_p, img_height, img_width, adj):

    cv2.circle(frame, (int(mid_p), img_height//2), 10, (0, 0, 255), -1)
    
    put_text_bg(
        frame,
        "X offset: %.2f" % err_x,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),   # text color (white)
        (0, 0, 0),         # background color (black)
        1
    )

    put_text_bg(
        frame,
        "Err Heading: %.2f" % err_h,
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        (0, 0, 0),
        1
    )

    direc = None
    if adj>0:
        direc = "right"
    else:
        direc = "left"

    put_text_bg(
        frame,
        "move %s: %.2fm" % (direc, adj),
        (10, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        (0, 0, 0),
        1
    )


    


def draw_distance_labels(frame, lane_ep, anchor_points_rw):
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5  # Slightly smaller for stacked lines
    thickness = 2
    line_spacing = 20 # Pixels between the two lines of text
    dot_radius = 10   # As requested
    vertical_margin = 15 # Space between the top of the dot and the bottom of the text

    for side in ['left', 'right']:
        if lane_ep[side] and anchor_points_rw[side]:
            for point_type in ['start', 'end']:
                pixel_pos = lane_ep[side][point_type]
                rw_key = f"{point_type}_meters"
                rw_val = anchor_points_rw[side][rw_key]

                # 1. Create two separate lines of text
                line1 = f"X: {rw_val[0]:.2f}m"
                line2 = f"Z: {rw_val[1]:.1f}m"

                # 2. Calculate position to be ABOVE the dot
                # Shift X to center the text relative to the dot
                text_x = pixel_pos[0] - 60 
                # Shift Y up: dot_radius + margin + total height of both lines
                text_y_base = pixel_pos[1] - dot_radius - vertical_margin - line_spacing

                color = (255, 0, 255) if side == 'left' else (255, 0, 0)

                # 3. Draw Line 1 (X distance)
                # Shadow
                cv2.putText(frame, line1, (text_x, text_y_base), font, font_scale, 
                            (0, 0, 0), thickness + 2, cv2.LINE_AA)
                # Color
                cv2.putText(frame, line1, (text_x, text_y_base), font, font_scale, 
                            color, thickness, cv2.LINE_AA)

                # 4. Draw Line 2 (Y distance)
                # Shadow
                cv2.putText(frame, line2, (text_x, text_y_base + line_spacing), font, font_scale, 
                            (0, 0, 0), thickness + 2, cv2.LINE_AA)
                # Color
                cv2.putText(frame, line2, (text_x, text_y_base + line_spacing), font, font_scale, 
                            color, thickness, cv2.LINE_AA)

                # 5. Draw the Big Anchor Point
                # Black outer ring for contrast
                cv2.circle(frame, pixel_pos, dot_radius, (0, 0, 0), -1, cv2.LINE_AA)
                # Main 
                # if point_type=="start":
                    # cv2.circle(frame, pixel_pos, dot_radius, (255, 0, 0), -1, cv2.LINE_AA)
                # elif point_type=="end":
                cv2.circle(frame, pixel_pos, dot_radius, (0, 0, 0), -1, cv2.LINE_AA)
                # Small colored center dot for side identification
                cv2.circle(frame, pixel_pos, 4, color, -1, cv2.LINE_AA)

    return frame


def rotate_point(x, y, theta):
    """Rotate point by theta (radians)"""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([c*x - s*y, s*x + c*y])


def estimate_parking_center_from_start(rw_start, heading_deg):

    x_s, y_s, _ = rw_start

    # Local offset to parking center
    dx_local = -spot_width / 2
    dy_local =  spot_length / 2

    # Rotate by -heading (world → camera)
    theta = -np.deg2rad(heading_deg)
    c, s = np.cos(theta), np.sin(theta)

    dx = c * dx_local - s * dy_local
    dy = s * dx_local + c * dy_local

    # Translate to camera-ground frame
    x_center = x_s + dx
    y_center = y_s + dy

    return x_center, y_center


def estimate_parking_center_from_start_l(rw_start, heading_deg):

    x_s, y_s, _ = rw_start

    # Local offset to parking center
    dx_local = spot_width / 2
    dy_local =  spot_length / 2

    # Rotate by -heading (world → camera)
    theta = -np.deg2rad(heading_deg)
    c, s = np.cos(theta), np.sin(theta)

    dx = c * dx_local - s * dy_local
    dy = s * dx_local + c * dy_local

    # Translate to camera-ground frame
    x_center = x_s + dx
    y_center = y_s + dy

    return x_center, y_center



def estimate_parking_center_from_end_l(rw_end, heading_deg):

    x_s, y_s, _ = rw_end

    # Local offset to parking center
    dx_local = spot_width / 2
    dy_local = -lane_tape_right+spot_length / 2

    # Rotate by -heading (world → camera)
    theta = -np.deg2rad(heading_deg)
    c, s = np.cos(theta), np.sin(theta)

    dx = c * dx_local - s * dy_local
    dy = s * dx_local + c * dy_local

    # Translate to camera-ground frame
    x_center = x_s + dx
    y_center = y_s + dy

    return x_center, y_center



def estimate_parking_area_from_start(rw_start, heading_deg):
    """
    rw_start: (x, y) right-lane start point in camera-ground frame (meters)
    returns: (x, y, z) parking center relative to camera
    """

    x0, y0, _ = rw_start

    local_pts = np.array([
        [-spot_width, 0],              # p1: left start
        [0, 0],                        # p2: right start
        [0, spot_length],              # p3: right end
        [-spot_width, spot_length]     # p4: left end
    ])

    # Rotate by -heading (world → camera frame)
    theta = -np.deg2rad(heading_deg)

    rotated_pts = np.array([
        rotate_point(px, py, theta)
        for px, py in local_pts
    ])

    # Translate to rw_start
    world_pts = rotated_pts + np.array([x0, y0])

    p1, p2, p3, p4 = world_pts.tolist()
    return p1, p2, p3, p4



def estimate_parking_area_from_start_l(rw_start, heading_deg):
    """
    rw_start: (x, y) right-lane start point in camera-ground frame (meters)
    returns: (x, y, z) parking center relative to camera
    """

    x0, y0, _ = rw_start

    local_pts = np.array([
        [0, 0],     # p1: left start
        [spot_width, 0],     # p2: right start
        [spot_width, spot_length],     # p3: right end
        [0, spot_length]     # p4: left end
    ])

    # Rotate by -heading (world → camera frame)
    theta = -np.deg2rad(heading_deg)

    rotated_pts = np.array([
        rotate_point(px, py, theta)
        for px, py in local_pts
    ])

    # Translate to rw_start
    world_pts = rotated_pts + np.array([x0, y0])

    p1, p2, p3, p4 = world_pts.tolist()
    return p1, p2, p3, p4

def estimate_parking_area_from_end_l(rw_end, heading_deg):
    """
    rw_start: (x, y) right-lane start point in camera-ground frame (meters)
    returns: (x, y, z) parking center relative to camera
    """

    x0, y0, _ = rw_end

    local_pts = np.array([
        [0, -spot_length+lane_tape_right],     # p1: left start
        [spot_width, -spot_length+lane_tape_right],     # p2: right start
        [spot_width, spot_length-lane_tape_right],     # p3: right end
        [0, spot_length-lane_tape_right]     # p4: left end
    ])

    # Rotate by -heading (world → camera frame)
    theta = -np.deg2rad(heading_deg)

    rotated_pts = np.array([
        rotate_point(px, py, theta)
        for px, py in local_pts
    ])

    # Translate to rw_start
    world_pts = rotated_pts + np.array([x0, y0])

    p1, p2, p3, p4 = world_pts.tolist()
    return p1, p2, p3, p4



def estimate_parking_center_from_end(rw_end, heading_deg):
    """
    rw_start: (x, y) right-lane start point in camera-ground frame (meters)
    returns: (x, y, z) parking center relative to camera
    """
    # x_s, y_s, _ = rw_end

    # x_center = x_s - spot_width / 2
    # y_center = y_s - spot_length / 2

    # return x_center, y_center, 0.0

    x_s, y_s, _ = rw_end

    # Local offset to parking center
    dx_local = -spot_width / 2 
    dy_local =  -lane_tape_right+spot_length / 2

    # Rotate by -heading (world → camera)
    theta = -np.deg2rad(heading_deg)
    c, s = np.cos(theta), np.sin(theta)

    dx = c * dx_local - s * dy_local
    dy = s * dx_local + c * dy_local

    # Translate to camera-ground frame
    x_center = x_s + dx
    y_center = y_s + dy

    return x_center, y_center



def estimate_parking_area_from_end(rw_end, heading_deg):
    """
    rw_start: (x, y) right-lane start point in camera-ground frame (meters)
    returns: (x, y, z) parking center relative to camera
    """

    x0, y0, _ = rw_end

    local_pts = np.array([
        [-spot_width, -spot_length+lane_tape_right],              # p1: left start
        [0, -spot_length+lane_tape_right],                        # p2: right start
        [0, spot_length-lane_tape_right],              # p3: right end (anchor)
        [-spot_width, spot_length-lane_tape_right]     # p4: left end
    ])



    # Rotate by -heading (world → camera frame)
    theta = -np.deg2rad(heading_deg)

    rotated_pts = np.array([
        rotate_point(px, py, theta)
        for px, py in local_pts
    ])

    # Translate to rw_start
    world_pts = rotated_pts + np.array([x0, y0])

    p1, p2, p3, p4 = world_pts.tolist()
    return p1, p2, p3, p4





def draw_parking_center(frame, center_px, rx, ry, color=(255, 0, 0), by='left'):
    if center_px is None:
        return frame

    u,v = center_px

    # Draw center point
    cv2.circle(frame, (u, v), radius=10, color=color, thickness=-1)

    # Label
    cv2.putText(
        frame,
        # "sensed by %s (%.2f,%.2f)"%(by,rx,ry),
        "Cen (%.2f,%.2f)"%(rx,ry),
        (u + 8, v - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        2
    )

    return frame



def draw_parking_area(frame, points, color=(255, 0, 0), alpha=0.3):
    if points is None or len(points) < 3: # Need at least 3 points for a poly
        return frame

    h, w = frame.shape[:2]
    
    # 1. Convert to numpy array 
    pts = np.array(points, dtype=np.int32)

    print("\n&&& parking area",pts, "\n")
    if pts[0][0]>5000: # fly out
        return frame
       

    # 2. Check if the polygon is entirely outside the view
    # If all x < 0 or all x > w, etc., we can skip drawing to save processing
    if (np.all(pts[:, 0] < 0) or np.all(pts[:, 0] > w) or 
        np.all(pts[:, 1] < 0) or np.all(pts[:, 1] > h)):
        return frame

    # 3. Create a mask for the semi-transparent overlay
    # This is often safer than copying the whole frame if the frame is 4K
    overlay = frame.copy()
    
    # Reshape for OpenCV requirements
    pts_reshaped = pts.reshape((-1, 1, 2))

    # Draw the filled area on the overlay
    cv2.fillPoly(overlay, [pts_reshaped], color)
    
    # Apply transparency
    frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

    # 4. Draw the outline
    # cv2.polylines handles points outside the image gracefully by clipping 
    # the lines at the edge of the canvas.
    cv2.polylines(frame, [pts_reshaped], isClosed=True, color=color, thickness=2)

    return frame

def clip_polygon_to_image(pts, w, h):
    # Image rectangle
    rect = np.array([
        [0, 0],
        [w - 1, 0],
        [w - 1, h - 1],
        [0, h - 1]
    ])

    pts = pts.astype(np.float32)
    clipped = cv2.intersectConvexConvex(pts, rect)[1]

    if clipped is None:
        return None

    return clipped.astype(np.int32)

def draw_parking_center2(frame, center_px):
    if center_px is None:
        return frame

    u,v = center_px
    color=(0, 0, 0)

    # Draw center point
    cv2.circle(frame, (u, v), radius=10, color=color, thickness=-1)

    # Label
    cv2.putText(
        frame,
        "Cam sensed target by far right point",
        (u + 8, v - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        2
    )

    return frame


def draw_ref_indicator(frame, ind1, ind2, ind3, ind4):
    # if not ind1 and not ind2 and not ind3 and not ind4:
        # return frame

    color=(0, 0, 255)

    if ind1:
        cv2.putText(
            frame,
            "R-C ref valid",
            (w - 120, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2
        )

    if ind2:
        cv2.putText(
            frame,
            "R-F ref valid",
            (w - 120, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2
        )
    
    if ind3:
        cv2.putText(
            frame,
            "L-C ref valid",
            (w - 240, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2
        )

    if ind4:
        cv2.putText(
            frame,
            "L-F ref valid",
            (w - 240, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2
        )
    
    return frame


# spot in vehicle-center world coord: -0.27687143885717225 4.849720453516838 -0.27687143885717225 7.349220453516837

def parking_center_to_vehicle(x, y, veh_heading):
    """
    Input:
        (x, y): parking center relative to camera (meters)
                x = right, y = forward

    Output:
        (E, N): parking center relative to vehicle center in world frame
    """

    # Step 1: Camera → Vehicle-center (translation)
    x_v = x
    y_v = y + vehicle_length / 2

    # Step 2: Vehicle → World (rotation by heading)
    h = np.deg2rad(veh_heading)

    E =  np.sin(h) * y_v + np.cos(h) * x_v
    N =  np.cos(h) * y_v - np.sin(h) * x_v

    return E, N


def rotate_world(x, y, veh_heading):
  
    # Step 1: Camera → Vehicle-center (translation)
    x_v = x
    y_v = y + vehicle_length / 2

    # Step 2: Vehicle → World (rotation by heading)
    h = np.deg2rad(veh_heading)

    E =  np.sin(h) * y_v + np.cos(h) * x_v
    N =  np.cos(h) * y_v - np.sin(h) * x_v

    return E, N

def target_wrt_initial_point(E, N, vehicle_pos):
    Ev, Nv = vehicle_pos
    return Ev + E, Nv + N




# def gps_to_meter(gps0, gps1):
#     """
#     Convert GPS to local ENU coordinates (meters)
#     East = x, North = y
#     """
#     # R = 6371000  # Earth radius (meters)
#     R = 6378137.0 # Earth radius in meters
    

#     lat0, lon0 = np.deg2rad(gps0)
#     lat1, lon1 = np.deg2rad(gps1)

#     dlat = lat1 - lat0
#     dlon = lon1 - lon0

#     x_east  = R * dlon * np.cos(lat0)
#     y_north = R * dlat

#     return x_east, y_north


def gps_to_meter(gps0, gps1):
    """
    Translates gps1 relative to gps0.
    gps format: [latitude, longitude]
    Returns: [x_meters (East), y_meters (North)]
    """
    R = 6378137.0 # Earth radius in meters
    
    lat0, lon0 = np.radians(gps0[0]), np.radians(gps0[1])
    lat1, lon1 = np.radians(gps1[0]), np.radians(gps1[1])
    
    # Calculate differences
    d_lat = lat1 - lat0
    d_lon = lon1 - lon0
    
    # North (Y) displacement
    y = d_lat * R
    
    # East (X) displacement
    # We use the average latitude to compensate for longitude convergence
    x = d_lon * R * np.cos((lat0 + lat1) / 2)
    
    return [x, y]


def estimate_v2p_heading_by_sense_line(p1, p2):
    """
    p1, p2: (x, y) tuples or arrays
    Returns heading in degrees where:
    North/Forward (+Y) = 0°
    East/Right (+X) = 90°
    South/Backward (-Y) = 180°
    West/Left (-X) = -90° (or 270°)
    """
    dx = p2[0] - p1[0] # Right/Left
    dy = p2[1] - p1[1] # Forward/Backward

    # In standard math: atan2(y, x) -> 0 is +X
    # For Navigation (0 is +Y): use atan2(x, y)
    orientation_rad = np.arctan2(dx, dy)

    # Convert to degrees
    orientation_deg = np.degrees(orientation_rad)
    
    return orientation_deg



## compare two estimation
def comp_lr_loc_est(park_center_r, park_cen_l):
    
    def extract_xy(data):
        xs, ys = [], []
        for p in data:
            if p is not None:
                xs.append(p[0])
                ys.append(p[1])
            else:
                xs.append(np.nan)
                ys.append(np.nan)
        return np.array(xs), np.array(ys)

    # Extract coordinates
    x1, y1 = extract_xy(park_center_r)
    x2, y2 = extract_xy(park_center_l)

    plt.figure(figsize=(8, 8))

    plt.plot(x1, y1, 'r.-', label='park_center_by_right')
    plt.plot(x2, y2, 'b.-', label='park_center_by_left')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Park Center est R vs L')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)

    plt.show()


def comp_lr_h_est(park_h_r, park_h_l):
    
    def extract_h(data):
        hs = []
        for p in data:
            if p is not None:
                if p>180:
                    p -= 360
                hs.append(p)
            else:
                hs.append(np.nan)
                
        return np.array(hs)

    # Extract coordinates
    h1 = extract_h(park_h_r)
    h2 = extract_h(park_h_l)

    plt.figure(figsize=(8, 8))

    plt.plot(h1, 'r.-', label='park_head_by_right')
    plt.plot(h2, 'b.-', label='park_head_by_left')

    plt.xlabel('T')
    plt.ylabel('Head')
    plt.title('Park heading est R vs L')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)

    plt.show()


def draw_middle_line_pixels(frame, left_lane_px, right_lane_px):
    """
    Draw a virtual middle line based purely on pixel coordinates.

    left_lane_px, right_lane_px: dict with keys 'start' and 'end', each as (x, y) pixel coordinates
    """
    
    if left_lane_px is None or right_lane_px is None:
        print("Cannot draw middle line; one lane is missing.")
        return frame

    # Compute middle points at start and end
    middle_start = (
        int((left_lane_px['start'][0] + right_lane_px['start'][0]) / 2),
        int((left_lane_px['start'][1] + right_lane_px['start'][1]) / 2)
    )
    middle_end = (
        int((left_lane_px['end'][0] + right_lane_px['end'][0]) / 2),
        int((left_lane_px['end'][1] + right_lane_px['end'][1]) / 2)
    )

    # Draw middle line (green, thickness=2)
    frame_with_line = frame.copy()
    cv2.line(frame_with_line, middle_start, middle_end, (0, 255, 0), 2)

    return frame_with_line



def load_veh_status():
    ## load real heading
    with open(can_h_fn, 'r') as f:
        line = f.read().strip()
        # Remove brackets and split by comma
        data = line.replace('[', '').replace(']', '').split(',')
        # h is the third element (index 2)
        v2w_heading = float(data[2])

    ## load ref gps
    with open(gps_ref_fn, 'r') as f:
        line = f.read().strip()
        # Remove brackets and split by comma
        data = line.replace('[', '').replace(']', '').split(',')
        # h is the third element (index 2)
        gps0 = [float(data[0]), float(data[1])]


    ## load realtime gps
    with open(gps_fn, 'r') as f:
        line = f.read().strip()
        # Remove brackets and split by comma
        data = line.replace('[', '').replace(']', '').split(',')
        # h is the third element (index 2)
        current_gps = [float(data[0]), float(data[1])]

    return v2w_heading, gps0, current_gps


## !! if not stop, need to predict/interpulate the current vehicle heading
def target_re_estimation(tracker):
    v2w_heading, gps0, current_gps = load_veh_status()
    vehicle_cur_pos_meter = gps_to_meter(gps0, current_gps)
    target = tracker.get_recent_est(v2w_heading, vehicle_cur_pos_meter)
    return target

def target_re_estimation_by_both(tracker):
    v2w_heading, gps0, current_gps = load_veh_status()
    vehicle_cur_pos_meter = gps_to_meter(gps0, current_gps)
    target = tracker.get_recent_est_by_both(v2w_heading, vehicle_cur_pos_meter)
    return target


def update_pathx5_by_cam(new_target):

    with open(path_fn, 'r') as f:
        line = f.readline()
    path = json.loads(line)
    target = path[-1]

    print('old path', path)

    ## change location first (x(East) first, then y(North)), heading not accurate yet
    path[-2][0] = new_target[0]
    # path[-2][1] = new_target[1] ## keep the y same, only change x for demo
    new_target.append(path[-2][2])

    ## also change the extension point
    ref = path[-2]
    new_x, new_y = forward_point(ref[0], ref[1], ref[2], 5)
    path[-1][0] = new_x 
    # path[-1][1] = new_y ## keep the y same, only change x for demo

    print('new path', path)

    with open(path_fn, 'w') as file:
        json.dump(path, file)

    return target, new_target
   




def forward_point(x, y, heading_deg, distance):
    theta = math.radians(heading_deg)

    dx = distance * math.sin(theta)
    dy = distance * math.cos(theta)

    return x + dx, y + dy



#### communication with current web server ########
def call_stop_service():
    url = "https://localhost:5001/stop_exc_for_replan"

    try:
        response = requests.post(url, verify=False)  # verify=False if self-signed cert
        
        if response.status_code == 200:
            print("Success:", response.json())
        else:
            print("Failed:", response.status_code, response.text)

    except requests.exceptions.RequestException as e:
        print("Error calling service:", e)


def call_resume_service():
    url = "https://localhost:5001/start_exc_for_replan"

    try:
        response = requests.post(url, verify=False)  # verify=False if self-signed cert
        
        if response.status_code == 200:
            print("Success:", response.json())
        else:
            print("Failed:", response.status_code, response.text)

    except requests.exceptions.RequestException as e:
        print("Error calling service:", e)


######################################################################3

# for rec_mid_in, emulate the turning
# Define the index range (60 to 220 inclusive)

ts = 30 # for left in
te = 150
# ts = 60 # for middle in
# te = 200
# ts = 55 # for right in
# te = 220
# ts = 90 # test0205
# te = 430
indices = np.arange(ts, te+1)
# Generate values that change linearly from 90 to 0
values = np.linspace(90, 0, len(indices))
# Combine into a dictionary or DataFrame
mapping = dict(zip(indices, values))



## streaming

## offline
# video_path = "rec1.mp4" 
# video_path = "rec2c.mp4" 
video_path = "rec_left_in.mp4" 
# video_path = "rec_mid_in.mp4" 
# video_path = "rec_right_in.mp4"
# video_path = "off_close_line.mp4" 
# video_path = "off_mid_line.mp4" 
# video_path = "off_far_line.mp4" 
# video_path = "test11.mp4"
# video_path = "test2.mp4"
# video_path = "test0205.mp4" # 534 frame, 157 gps/heading read


# video_path = "./0209/g1/cam_rec1.mp4" # 1986 frame, 230~1636 move
# video_path = "./0209/g2/cam_rec2.mp4" # 3106 frame , 280~2580 move

cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture(video_path)


## not good at fixing the buffer caused frame jump issue
# cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
# cap.set(cv2.CAP_PROP_FPS, 15)
# for _ in range(3):
#     cap.grab()
# ret, frame = cap.retrieve()


# Set lower resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

# Optional: lower FPS
cap.set(cv2.CAP_PROP_FPS, 30)


if not cap.isOpened():
    print("no camera found")
    exit()

frame_skip = 2 #2
count = 0
tracker = LaneTracker(max_history=500) 


## output mp4 format
fps = cap.get(cv2.CAP_PROP_FPS) or 30.0  # Default to 30 if metadata is missing
fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
video_output = cv2.VideoWriter('lane_detection_output.mp4', fourcc, fps, (w, h))
# video_output = cv2.VideoWriter('lane_detection_output.mp4', fourcc, fps, (1920, 1080))

valid_count = 0
valid_lr_count = 0

while True:

    ## read/load stream
    ret, frame = cap.read()

    if not ret:
        print("stream reading fail")
        break

        # print("Stream lost. Reconnecting...")
        # cap.release()
        # time.sleep(1)
        # cap = cv2.VideoCapture(0)  # or your stream URL
        # continue

    count += 1
    if count % frame_skip != 0:
        continue   # skip processing

    print("frame", count)
    # img_height, img_width = frame.shape[:2]
    # print(img_height, img_width)

    # exit()
    

    
    ## vehicle status
    v2w_heading = 0 # v2p for camera stream ui, v2w for execution
    current_gps = None
    gps0 = None
 
    ## remove, only for testing
    gps0 = (42.517299, -83.045387) ## ref (e.g. init point)
    current_gps = (42.517319, -83.045271)
    v2w_heading = 0
    vehicle_cur_pos_meter = None

    try:
        v2w_heading, gps0, current_gps = load_veh_status()
        vehicle_cur_pos_meter = gps_to_meter(gps0, current_gps)
            
        print("veh status", v2w_heading, gps0, current_gps, vehicle_cur_pos_meter)
    
    except:
        pass
        
    # exit()


                

    ## only detect within predefined range
    ## only detect when reaching the predefined range: stop, plan, then resume
    

    # frame = cv2.resize(frame, (w, h))

    # Undistort
    frame = cv2.undistort(frame, K, dist_coeffs)

    
    ### JJJJJ ###

    # exit()
    ori_img = frame.copy()

    ## basic add center of image point     
    cv2.circle(frame, (w//2, h//2), 10, (0,0,0), -1)
    # pt1 = (0, h//2)
    # pt2 = (w, h//2)
    # cv2.line(frame, pt1, pt2, (0,0,0), 2)


    #################
    
    # frame
    frame, project_points, mask = est_distance_grid(frame)

    
    ## 10m mask
    mask_colored = np.zeros_like(frame)
    mask_colored[:] = (0, 255, 0)  # BGR for Green
    mask_overlay = cv2.bitwise_and(mask_colored, mask_colored, mask=mask)
    frame = cv2.addWeighted(frame, 1.0, mask_overlay, 0.2, 0)
    
    ##########################


    # if v2w_heading>Det_heading_trigger[0] and v2w_heading>Det_heading_trigger[0]:
    if True:
    # !! based on the current vehicle gps to the target_static, and the current heading to the target heading, to find when to start lane detection 

        ## edge & Lane detection
        # edges, lines = edge_det(frame, hough_thr, blur_thr)
        edges, lines = edge_det_mask_range(ori_img, hough_thr, blur_thr, mask)
        
        frame = add_edge_on_img(frame, edges)
        frame = draw_all_hough_lines(frame, lines) ## deb
        
        ll,rl,mid_p,err_x, err_h = find_LR2(lines, w, h, angle_thr_deg)

        ## focuse on right line only
        # ll = None
        # rl = find_PL(lines, w, h, angle_thr_deg)

        ## add left/right parking lane to UI
        # frame, lane_ep = add_lanes_on_img(frame, ll, rl, mask, project_points)
        ## !!! try find connected max support line after find the intersection points between line equation and mask border 
        # frame, lane_ep = add_lanes_on_img_with_endpoints(frame, ll, rl, mask, project_points, edges, 1, 15)
        frame, lane_ep = add_lanes_on_img_with_endpoints_seg_check(frame, ll, rl, mask, project_points, edges, 1, 15)
                # print(lane_ep)

        anchor_points_rw = find_anchor_points(lane_ep)
        
        # for side, coords in anchor_points_rw.items():
        #     if coords:
        #         print(f"--- {side.upper()} LANE ---")
        #         print(f"Start (1m mark): X={coords['start_meters'][0]:.2f}m, Y={coords['start_meters'][1]:.2f}m")
        #         print(f"End (10m mark):  X={coords['end_meters'][0]:.2f}m, Y={coords['end_meters'][1]:.2f}m")

        # Assuming real_world_coords is the output from our previous step
        
        # --- Integration Example ---
        frame = draw_distance_labels(frame, lane_ep, anchor_points_rw)

        pc = None
        pc2veh = None
        pc2world = None
        target_heading = None
    
        pc_l = None
        pc2veh_l = None
        pc2world_l = None
        target_heading_l = None

        ## parking spot center estimation #####
        v2p_heading = 0 ## also test e.g. 359!!!! 



           
        ## 1st start with right lane close point
        if anchor_points_rw["right"]:

            ## estimate heading by start/end point on image
            p1 = anchor_points_rw["right"]['start_meters']
            p2 = anchor_points_rw["right"]['end_meters']
            v2p_heading_sen = estimate_v2p_heading_by_sense_line(p1, p2)
            print(f"Orientation est by right lane:({v2p_heading_sen:.2f}°)")

            if rbp_valid:
                ##
                ref_p =  anchor_points_rw["right"]['start_meters']
                x,y = estimate_parking_center_from_start(ref_p, v2p_heading_sen) # -15 earlier stage (v2p heading)
                print("parking2Cam est by right lane start point:", x, y)
                p1, p2, p3, p4 = estimate_parking_area_from_start(ref_p, v2p_heading_sen)
                pc = [x,y]
                
                try:
                    ## parking center
                    u,v = project_point_with_fov_to_img(x,y) 
                    # print("est center img", u,v, "realworld", x,y, "referring to start point of right lane")
                    
                    ## parking area
                    u1,v1 = project_point_with_fov_to_img(p1[0], p1[1])
                    u2,v2 = project_point_with_fov_to_img(p2[0], p2[1])
                    u3,v3 = project_point_with_fov_to_img(p3[0], p3[1])
                    u4,v4 = project_point_with_fov_to_img(p4[0], p4[1])
                   
                    # print("-(est by right)----",u,v,u1,v1,u2,v2,u3,v3,u4,v4)

                    frame = draw_parking_center(frame, (u,v), x, y, color=(255, 0, 0), by='right')
                    frame = draw_parking_area(frame, [(u1,v1), (u2,v2), (u3,v3), (u4,v4)], color=(255, 0, 0))
                    
                except:
                    # print("fail projection")
                    # exit()
                    pass

                ## (x,y) is the center of the parking lot regarding camera 
                ## world_x, world_y is the center of the parking lot (in East/North) regarding to veh heading can cam2veh mount
                ## finally also need to transform to the world coordinate regarding to the initial point of vehicle
                # world_x, world_y = parking_center_to_vehicle(x, y, v2p_heading_sen+v2w_heading)
                world_x, world_y = parking_center_to_vehicle(x, y, v2w_heading) ## doesn't matter with the sensed heading
                pc2veh = [world_x, world_y]
                # vehicle_cur_pos_meter = [10,10] ## need to transfer by gps
                world_x0, world_y0 = target_wrt_initial_point(world_x, world_y, vehicle_cur_pos_meter)
                pc2world = [world_x0, world_y0]
                
                ## from current vehicle position [0,0, CAN_h] to target [x,y,CAN_h+sensed_heading_off]
                target_heading = v2p_heading_sen+v2w_heading
                if target_heading<0:
                    target_heading+= 360
                start = [0, 0, v2w_heading]
                target = [world_x, world_y, target_heading]
                
                print("** spot in cam, vehicle, world x3 coord:", x, y, ", ", world_x, world_y, ", ", world_x0, world_y0, "heading x2", v2p_heading_sen, target_heading)

                ## compare and use the current (world_x0, world_y0) to replace the static planned target (sx,sy), 
                data_to_save = [start, target]

                # Save to a text file use current as [0,0]
                
                with open(v2spot_file_path, 'a') as f:
                    # Option 1: Save as a string representation of the list
                    f.write(str(data_to_save)+"\n")

                ## to test and compare with the pathx5
                ## compare and use the current (world_x0, world_y0) to replace the static planned target (sx,sy), 
                # Save to a text file use initial point as [0,0]
              
                start0 = [vehicle_cur_pos_meter[0], vehicle_cur_pos_meter[1], v2w_heading]
                target0 = [vehicle_cur_pos_meter[0]+world_x, vehicle_cur_pos_meter[1]+world_y, target_heading]
                data_to_save0 = [start0, target0]

                
                with open(v2spot_file_path_world, 'a') as f:
                    # Option 1: Save as a string representation of the list
                    f.write(str(data_to_save0)+"\n")


                print("new path obtain current fr:", start0, "to", target0)
                valid_count += 1

                ## problem: v2w heading delay, world coordinate won't be stable
                ## when stop the vehicle, wait to update the recent vehicle heading, re-estimate the target
                ## if FOV good enough, wait till both lane shows and consist with each other, then stop and replan 
                if valid_count==decision_point:
                    # 1. stop veh
                    print("send stop cmd")
                    call_stop_service()
                    
                    # 2. reload heading and get current path
                    # print("valid", valid_count)
                    # time.sleep(2.0) ## wait till heading converge
                    new_target = target_re_estimation(tracker)
                    print("new planned target", new_target)

                     # 3. resume (update pathx5, resume task)
                    if new_target:
                        # pass
                        t0, t1 = update_pathx5_by_cam(new_target)
                        print("!! path succ switched target fr", t0, "to", t1)

                    else:
                        print("!! not enough cam info to replan path")

                    
                    call_resume_service()
                       
                    # exit()
                

        
        # ## 2nd when close point out of view, est with top point (better not using the far point, error large)
        # if anchor_points_rw["right"]:
        #     ref_p =  anchor_points_rw["right"]['end_meters']
        #     x,y = estimate_parking_center_from_end(ref_p, v2p_heading_sen)
        #     p1, p2, p3, p4 = estimate_parking_area_from_end(ref_p, v2p_heading_sen)
            
        #     try:
        #         u,v = project_point_with_fov_to_img(x,y)
        #         # print("est center img", u,v, "realworld", x,y, "referring to end point of right lane")
                
        #         u1,v1 = project_point_with_fov_to_img(p1[0], p1[1])
        #         u2,v2 = project_point_with_fov_to_img(p2[0], p2[1])
        #         u3,v3 = project_point_with_fov_to_img(p3[0], p3[1])
        #         u4,v4 = project_point_with_fov_to_img(p4[0], p4[1])
                
        #         frame = draw_parking_center2(frame, (u,v))
        #         # frame = draw_parking_area(frame, [(u1,v1), (u2,v2), (u3,v3), (u4,v4)], color=(0, 0, 0))
                
        #     except:
        #         pass
       

        ## 3rd start with left lane close point (if left lane visible in fov, close point is better for ref est)
        if anchor_points_rw["left"]:

            # estimate heading by start/end point on image
            p1_l = anchor_points_rw["left"]['start_meters']
            p2_l = anchor_points_rw["left"]['end_meters']
            v2p_heading_sen_l = estimate_v2p_heading_by_sense_line(p1_l, p2_l)
            print(f"Orientation est by left lane:({v2p_heading_sen_l:.2f}°)")

            if lbp_valid:
                ##
                ref_p =  anchor_points_rw["left"]['start_meters']
                xl,yl = estimate_parking_center_from_start_l(ref_p, v2p_heading_sen_l) # -15 earlier stage (v2p heading)
                print("parking2Cam est by left lane start point:", xl, yl)
                p1, p2, p3, p4 = estimate_parking_area_from_start_l(ref_p, v2p_heading_sen_l)
                pc_l = [xl,yl]
                
                try:
                    ## parking center
                    u,v = project_point_with_fov_to_img(xl,yl) 
                    # print("est center img", u,v, "realworld", x,y, "referring to start point of right lane")
                    
                    ## parking area
                    u1,v1 = project_point_with_fov_to_img(p1[0], p1[1])
                    u2,v2 = project_point_with_fov_to_img(p2[0], p2[1])
                    u3,v3 = project_point_with_fov_to_img(p3[0], p3[1])
                    u4,v4 = project_point_with_fov_to_img(p4[0], p4[1])
                   
                    # print("-(est by left)----",u,v,u1,v1,u2,v2,u3,v3,u4,v4)

                    frame = draw_parking_center(frame, (u,v), xl, yl, color=(255, 0, 255), by='left')
                    frame = draw_parking_area(frame, [(u1,v1), (u2,v2), (u3,v3), (u4,v4)], color=(255, 0, 255))
                    
                except:
                    # print("fail projection")
                    # exit()
                    pass

                ## (x,y) is the center of the parking lot regarding camera 
                ## world_x, world_y is the center of the parking lot (in East/North) regarding to veh heading can cam2veh mount
                ## finally also need to transform to the world coordinate regarding to the initial point of vehicle
                world_x_l, world_y_l = parking_center_to_vehicle(xl, yl, v2w_heading)
                pc2veh_l = [world_x_l, world_y_l]
                
                # print(">>",vehicle_cur_pos_meter)
                # vehicle_cur_pos_meter = [10,10] ## need to transfer by gps
                world_x0_l, world_y0_l = target_wrt_initial_point(world_x_l, world_y_l, vehicle_cur_pos_meter)
                pc2world_l = [world_x0_l, world_y0_l]

                
                ## from current vehicle position [0,0, CAN_h] to target [x,y,CAN_h+sensed_heading_off]
                target_heading_l = v2p_heading_sen_l+v2w_heading
                if target_heading_l<0:
                    target_heading_l+= 360
                start = [0, 0, v2w_heading]
                target = [world_x_l, world_y_l, target_heading_l]
                
                print("** by left, spot in cam, vehicle, world x3 coord:", xl, yl, v2p_heading_sen_l, ", ", world_x_l, world_y_l, target_heading_l, ", ", world_x0_l, world_y0_l, target_heading_l)



            ## 4th end with left lane end point
            # p1_l = anchor_points_rw["left"]['start_meters']
            # p2_l = anchor_points_rw["left"]['end_meters']
            # v2p_heading_sen_l = estimate_v2p_heading_by_sense_line(p1_l, p2_l)
            # print(f"Orientation est by left lane:({v2p_heading_sen_l:.2f}°)")

            # if ltp_valid:
            #     ##
            #     ref_p =  anchor_points_rw["left"]['end_meters']
            #     xl,yl = estimate_parking_center_from_end_l(ref_p, v2p_heading_sen_l) # -15 earlier stage (v2p heading)
            #     print("parking2Cam est by left lane end point:", xl, yl)
            #     p1, p2, p3, p4 = estimate_parking_area_from_end_l(ref_p, v2p_heading_sen_l)
            #     pc_l = [xl,yl]
                
            #     try:
            #         ## parking center
            #         u,v = project_point_with_fov_to_img(xl,yl) 
            #         # print("est center img", u,v, "realworld", x,y, "referring to start point of right lane")
                    
            #         ## parking area
            #         u1,v1 = project_point_with_fov_to_img(p1[0], p1[1])
            #         u2,v2 = project_point_with_fov_to_img(p2[0], p2[1])
            #         u3,v3 = project_point_with_fov_to_img(p3[0], p3[1])
            #         u4,v4 = project_point_with_fov_to_img(p4[0], p4[1])
                   
            #         # print("-(est by left)----",u,v,u1,v1,u2,v2,u3,v3,u4,v4)
            #         frame = draw_parking_center(frame, (u,v), xl, yl, color=(255, 0, 255), by='left')
            #         frame = draw_parking_area(frame, [(u1,v1), (u2,v2), (u3,v3), (u4,v4)], color=(255, 0, 255))
                    
            #     except:
            #         # print("fail projection")
            #         # exit()
            #         pass

            #     ## (x,y) is the center of the parking lot regarding camera 
            #     ## world_x, world_y is the center of the parking lot (in East/North) regarding to veh heading can cam2veh mount
            #     ## finally also need to transform to the world coordinate regarding to the initial point of vehicle
            #     world_x_l, world_y_l = parking_center_to_vehicle(xl, yl, v2w_heading)
            #     pc2veh_l = [world_x_l, world_y_l]
                
            #     # print(">>",vehicle_cur_pos_meter)
            #     # vehicle_cur_pos_meter = [10,10] ## need to transfer by gps
            #     world_x0_l, world_y0_l = target_wrt_initial_point(world_x_l, world_y_l, vehicle_cur_pos_meter)
            #     pc2world_l = [world_x0_l, world_y0_l]

                
            #     ## from current vehicle position [0,0, CAN_h] to target [x,y,CAN_h+sensed_heading_off]
            #     target_heading_l = v2p_heading_sen_l+v2w_heading
            #     if target_heading_l<0:
            #         target_heading_l+= 360
            #     start = [0, 0, v2w_heading]
            #     target = [world_x_l, world_y_l, target_heading_l]
                
            #     print("** by left, spot in cam, vehicle, world x3 coord:", xl, yl, v2p_heading_sen_l, ", ", world_x_l, world_y_l, target_heading_l, ", ", world_x0_l, world_y0_l, target_heading_l)


        tracker.update(anchor_points_rw, pc, pc_l, pc2veh, pc2veh_l, pc2world, pc2world_l, current_gps, v2w_heading, target_heading, target_heading_l)

        tracker.check_consistance(window_size=10, std_threshold=0.15)

        ## tracker current past window



        ## both lane partically visible
        if ltp_valid and rbp_valid and pc and pc_l:
        # if anchor_points_rw["right"] and anchor_points_rw["left"]: 
            print("####### both lanes visible, heading sensed", target_heading, target_heading_l)
            print("####### both lanes visible, location sensed", pc, pc_l)
            dis_vary = ((pc[0]-pc_l[0])**2+(pc[1]-pc_l[1])**2)**0.5
            if dis_vary<LR_vary_thr:
                valid_lr_count += 1


        if valid_lr_count==decision_point2:
            time.sleep(2.0)
            print("left/right agree 5 times")

            # frame = draw_middle_line_pixels(frame, anchor_points_rw["left"], anchor_points_rw["right"])

            # # 1. stop veh
            # print("send stop cmd")
            # call_stop_service()
            
            # # 2. reload heading and get current path
            # # print("valid", valid_count)
            # time.sleep(2.0) ## wait till heading converge
            # new_target = target_re_estimation_by_both(tracker)
            # print("new planned target", new_target)

            #  # 3. resume (update pathx5, resume task)
            # if new_target:
            #     # pass
            #     t0, t1 = update_pathx5_by_cam(new_target)
            #     print("!! path succ switched target fr", t0, "to", t1)

            # else:
            #     print("!! not enough cam info to replan path")

            
            # call_resume_service()



        ## debug show ref point
        frame = draw_ref_indicator(frame, rbp_valid, rtp_valid, lbp_valid, ltp_valid)
       
        ### JJJJJ ###

        ## if mid of 2 lanes detected, add target point on image, add txt UI
        # if mid_p:

        #     goal = int(mid_p)
        #     curr = int(img_width//2)
            
        #     # adjustment in pixel
        #     error_pixel = goal-curr
            
        #     ## adjustment map to meter
        #     adj = dyn_adj_dis_simple(hfov_deg, img_width, distance, error_pixel)
        #     # adj = dyn_adj_dis(hfov_deg, vfov_deg, tilt_deg, cam_height, img_width, img_height, error_pixel)
            
        #     add_deb_info(frame, mid_p, img_height, img_width, adj)
    

    cv2.imshow('stream', frame)
    video_output.write(frame)
    
    # if cv2.waitKey(1) & 0xFF==ord('q'):
    if cv2.waitKey(100) & 0xFF==ord('q'): ## 10ms per frame
        break


cap.release()
cv2.destroyAllWindows()

# Access the history tracking data of anchor points
full_path = tracker.get_history_list()
# print(f"Tracked {len(full_path['right_start'])} frames of data.")
# print(full_path)


# with open(track_fn, 'w') as f:
#     json.dump(full_path, f)

# plot_lane_history(full_path)
# # plot_pc_history(full_path, 'park_cen')
# plot_pc_history(full_path, 'park_cen_veh')
# # plot_pc_history(full_path, 'park_cen_world')

# filter_full_path = lane_history_filtered(full_path)
# # plot_lane_history(filter_full_path)
# plot_pc_history(filter_full_path, 'park_cen_veh')


# ## compare by camera coordinate first
# # park_center_r = full_path["park_cen"]     
# # park_center_l = full_path["park_cen_l"]  
# park_center_r = full_path["park_cen_veh"]     
# park_center_l = full_path["park_cen_veh_l"]   
# comp_lr_loc_est(park_center_r, park_center_l)


# park_h_r = full_path["park_heading_world"]    
# park_h_l = full_path["park_heading_world_l"]   
# comp_lr_h_est(park_h_r, park_h_l)

print("valid right count", valid_count)
print("valid both count", valid_lr_count)



