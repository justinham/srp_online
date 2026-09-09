# ffplay /dev/video0

import cv2
import numpy as np
import json
from collections import deque
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter
import pandas as pd


## vehicle

vehicle_width = 2.196 # in meter
vehicle_height = 1.976
vehicle_length = 4.999

lane_length = 5.0  # meters
lane_offset = 1.5  # right of target
lane_width = 0.2   # optional lane width

############## 70 camera ##############

# fov
# hfov_deg = 70
# vfov_deg = 70 

# # fixed look-ahead distance
# distance = 0.25 

# # look-ahead distance by tilt degree and mount height and fov
# cam_height = 0.1
# tilt_deg = 20

# ##
# blur_thr = 11
# hough_thr = 200
# angle_thr_deg = 75


############# 170 camera ##############

# fov
hfov_deg = 170
vfov_deg = 120 
adj_fov = 10

# fixed look-ahead distance
distance = 5.0 

# look-ahead distance by tilt degree and mount height and fov
cam_height = 1.0
tilt_deg = 70

##
blur_thr = 9 # 15
hough_thr = 200 # 200
angle_thr_deg = 75


## test streaming screen recording W,H
# 2592(H) x 1944(V) @ 15fps 170 Degree Fisheye Lens
# w = 1916
# h = 1072

w = 2592
h = 1944

# --- Camera Calibration Parameters ---
# Adjust based on specific camera hardware
focal_length = w * 0.8 #113 for 170 # w * 0.8 for 64 degree only       # Focal length in pixels
camera_height = 0.8          # Height of camera from ground (meters)
pitch_angle = np.radians(10) # Tilt down (10 degrees)


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
        ('left_start', 'lightgreen', 'L-Start'), ('right_start', 'salmon', 'R-Start'),
        ('left_end', 'darkgreen', 'L-End'), ('right_end', 'red', 'R-End')
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
    
    # --- Top-Down Path Plot ---
    plt.figure(figsize=(6, 8))
    for key, color in [('left_start', 'Greens'), ('left_end', 'YlGn'), 
                       ('right_start', 'Reds'), ('right_end', 'OrRd')]:
        data = np.array([pt for pt in history_data[key] if pt is not None])
        if len(data) > 0:
            plt.scatter(data[:,0], data[:,1], c=np.arange(len(data)), cmap=color, s=15)
            
    plt.axvline(0, color='black', linestyle='--') # Car center
    plt.title("Top-Down Trajectory (Path Map)")
    plt.xlabel("Lateral (m)")
    plt.ylabel("Forward (m)")
    plt.xlim(-4, 4)
    # plt.legend()
    plt.grid(True, alpha=0.3)
    plt.savefig('top_down_trajectory.png')



def lane_history_filtered(history_data):
    filtered_history = {}
    
    for key in ['left_start', 'left_end', 'right_start', 'right_end']:
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
    grid_depth = 15

    # Draw Depth Lines (Lane width markers)
    # Using the standard 9ft (~2.7m) parking width as the outer bounds
    for xw in [-1.35, -0.5, 0.0, 0.5, 1.35]: 
        pts = []
        for yw in np.linspace(1.0, grid_depth, 20):
            pt = project_point_with_fl(xw, yw)
            # pt = project_point_with_fov(xw, yw)
            if pt and 0 <= pt[0] < w and 0 <= pt[1] < h:
                pts.append(pt)
        
        if len(pts) > 1:
            cv2.polylines(frame, [np.array(pts)], False, (0, 255, 255), 2)

            # Labeling
            label_pt = pts[0]
            cv2.putText(frame, f"{xw}m", (label_pt[0]-45, label_pt[1]+30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)


    # Draw Distance Markers (1m to 9m)
    for yw in range(1, grid_depth+1):
        pts = []
        for xw in np.linspace(-1.35, 1.35, 10):
            pt = project_point_with_fl(xw, yw)
            # pt = project_point_with_fov(xw, yw)
            if pt and 0 <= pt[0] < w and 0 <= pt[1] < h:
                pts.append(pt)
        
        if len(pts) > 1:
            # Red for close-range (<2m), Yellow otherwise
            # color = (0, 0, 255) if yw <= 2 else (0, 255, 255)
            color = (0, 255, 255)
            cv2.polylines(frame, [np.array(pts)], False, color, 2)
            
            # Labeling
            label_pt = pts[0]
            cv2.putText(frame, f"{yw}m", (label_pt[0]-70, label_pt[1]+5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

    # Generate final mask and corners
    # mask, corners_10m = get_10m_mask(h, w)
    mask, corners_10m = get_15m_mask(h, w)

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

# def project_point_with_fov(xw, yw):
#     # 1. Convert Horizontal FOV (degrees) to Focal Length (pixels)
#     fov_rad = np.deg2rad(hfov_deg)
#     f_pixels = (w / 2) / np.tan(fov_rad / 2)
    
#     # 2. Transform Ground (xw, yw, 0) to Camera Coordinates
#     # Z is depth/forward, Y is vertical/down
#     z_cam = yw * np.cos(pitch_angle) + camera_height * np.sin(pitch_angle)
#     y_cam = camera_height * np.cos(pitch_angle) - yw * np.sin(pitch_angle)
#     x_cam = xw
    
#     # 3. Clipping
#     if z_cam <= 0.1: 
#         return None
    
#     # 4. Project using f_pixels derived from FOV
#     u = int((x_cam * f_pixels / z_cam) + w / 2)
#     v = int((y_cam * f_pixels / z_cam) + h / 2)
    
#     # 5. Out-of-bounds check (Optional)
#     if not (0 <= u < w and 0 <= v < h):
#         return None
        
#     return (u, v)

def project_point_with_fov(xw, yw):
    fov_x = np.deg2rad(hfov_deg)
    fx = (w / 2) / np.tan(fov_x / 2)*adj_fov
    fov_y = 2 * np.arctan((h / w) * np.tan(fov_x / 2))
    fy = (h / 2) / np.tan(fov_y / 2)*adj_fov

    z_cam = yw * np.cos(pitch_angle) + camera_height * np.sin(pitch_angle)
    y_cam = camera_height * np.cos(pitch_angle) - yw * np.sin(pitch_angle)
    x_cam = xw

    if z_cam <= 0.1:
        return None

    u = int(x_cam * fx / z_cam + w / 2)
    v = int(y_cam * fy / z_cam + h / 2)

    if not (0 <= u < w and 0 <= v < h):
        return None

    return (u, v)


def get_10m_mask(h, w):
        mask = np.zeros((h, w), dtype=np.uint8)
        # 19ft x 9ft area (~5.8m x 2.7m) or your custom 10m zone
        corners_world = [(-2.0, 2.0), (2.0, 2.0), (2.0, 10.0), (-2.0, 10.0)]
        
        pixel_pts = []
        for xw, yw in corners_world:
            pt = project_point_with_fl(xw, yw)
            # pt = project_point_with_fov(xw, yw)
            if pt: pixel_pts.append(pt)
            
        if len(pixel_pts) >= 3:
            cv2.fillPoly(mask, [np.array(pixel_pts, dtype=np.int32)], 255)
        return mask, pixel_pts

def get_15m_mask(h, w):
        mask = np.zeros((h, w), dtype=np.uint8)
        # 19ft x 9ft area (~5.8m x 2.7m) or your custom 10m zone
        corners_world = [(-2.0, 2.0), (2.0, 2.0), (2.0, 15.0), (-2.0, 15.0)]
        
        pixel_pts = []
        for xw, yw in corners_world:
            pt = project_point_with_fl(xw, yw)
            # pt = project_point_with_fov(xw, yw)
            if pt: pixel_pts.append(pt)
            
        if len(pixel_pts) >= 3:
            cv2.fillPoly(mask, [np.array(pixel_pts, dtype=np.int32)], 255)
        return mask, pixel_pts



def pixel_to_world(u, v, w, h):
    
    # 1. Translate pixel to optical center coordinate system
    # (x_c, y_c) are coordinates relative to the lens center
    x_c = u - w / 2
    y_c = v - h / 2
    
    # 2. Rotation / Trig approach
    # We need the angle of the ray relative to the optical axis
    # alpha is the vertical angle of the pixel relative to the center
    alpha = np.arctan2(y_c, focal_length)
    
    # Total angle relative to the ground
    # If the camera is pitched down, the ray angle is (pitch + alpha)
    total_angle = pitch_angle + alpha
    
    # 3. Calculate Ground Distance (yw)
    # yw = height / tan(total_angle)
    # If total_angle <= 0, the pixel is looking at or above the horizon
    if total_angle <= 0:
        return None # Looking at the sky
        
    yw = camera_height / np.tan(total_angle)
    
    # 4. Calculate Lateral Distance (xw)
    # xw is proportional to the distance yw and the horizontal pixel offset
    # We use the hypotenuse (distance from camera to ground point)
    dist_to_point = np.sqrt(yw**2 + camera_height**2)
    xw = (x_c * dist_to_point) / np.sqrt(focal_length**2 + y_c**2)

    return xw, yw


def find_anchor_points(endpoints):
    real_world_coords = {}

    width, height = w, h

    for side in ['left', 'right']:
        if endpoints[side] is not None:
            start_px = endpoints[side]['start']
            end_px = endpoints[side]['end']
            
            # Map Start Point (usually the point closer to the car)
            rw_start = pixel_to_world(start_px[0], start_px[1], width, height)
            
            # Map End Point (usually the point further away)
            rw_end = pixel_to_world(end_px[0], end_px[1], width, height)
            
            real_world_coords[side] = {
                'start_meters': rw_start,
                'end_meters': rw_end
            }
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
            'right_end':  deque(maxlen=max_history)
        }

    def update(self, real_world_coords):
        """
        Expects: {'left': {'start_meters': (x,y), 'end_meters': (x,y)}, 'right': ...}
        or None values.
        """
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

    def get_history_list(self):
        """Returns the history as a standard Python list for saving/JSON"""
        return {key: list(val) for key, val in self.history.items()}



def edge_det(frame, hough_thr, blur_thr):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (blur_thr,blur_thr), 0)
    edges = cv2.Canny(blurred, 50, 150)
    # thr: minimum number of votes needed to detect a line
    lines = cv2.HoughLines(edges, 1, np.pi/180, hough_thr)
    return edges, lines





def edge_det_mask_range(frame, hough_thr, blur_thr, mask):
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



def edge_det_mask_range_conn(frame, hough_thr, blur_thr, mask):
    # 1. Pre-processing
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    k_size = blur_thr if blur_thr % 2 != 0 else blur_thr + 1
    blurred = cv2.GaussianBlur(gray, (k_size, k_size), 0)

    # 2. Masked Canny
    masked_input = cv2.bitwise_and(blurred, mask)
    edges = cv2.Canny(masked_input, 50, 150)

    # 3. Connect Edges (Dilation & Erosion)
    # Using a vertical kernel helps connect lane dashes without blurring them sideways
    connection_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 7))
    connected_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, connection_kernel)

    # 4. Remove Spread Noise (Area Filtering)
    # This finds every "blob" of white pixels and deletes those that are too small
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(connected_edges, connectivity=8)
    
    # Create blank canvas for cleaned edges
    clean_edges = np.zeros_like(connected_edges)
    min_area = 40  # Adjust based on how far away your 10m range is
    
    for i in range(1, num_labels): # Skip background label 0
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            clean_edges[labels == i] = 255

    # 5. Remove Mask Border
    # We still erode the mask to ensure the trapezoid border isn't counted as a line
    border_kernel = np.ones((5, 5), np.uint8)
    eroded_mask = cv2.erode(mask, border_kernel, iterations=1)
    final_edges = cv2.bitwise_and(clean_edges, eroded_mask)

    # 6. Line Detection
    lines = cv2.HoughLines(final_edges, 1, np.pi/180, hough_thr)
    
    return final_edges, lines


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
                window = edge_map[y, max(0, x-conn_thr):min(edge_map.shape[1], x+conn_thr)]
                if np.any(window > 0):
                    active_points.append((x, y))
        
        # print(active_points)
        # 3. Filter for the longest continuous segment
        # If we found enough points, define the new start/end
        if len(active_points) > min_len_thr:
            # Finding the "extremes" of the detected edge clusters
            p_start = active_points[0]  # Closest to bottom
            p_end = active_points[-1]   # Furthest away
            
            endpoints[label] = {"start": p_start, "end": p_end}
            
            # 4. Draw only the segment where edges were actually found
            cv2.line(line_layer, p_start, p_end, color, 8, cv2.LINE_AA)

    # Blend with frame
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
    img_center = width//2
    eval_y = height//2
    max_x_left = -float('inf')
    min_x_right = float('inf')

    lower_b = np.deg2rad(angle_thr_deg)
    upper_b = np.pi-np.deg2rad(angle_thr_deg)

    ## 45 degree
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

    return close_left, close_right, mid_p, error_pixel, err_heading



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
    font_scale = 0.6  # Slightly smaller for stacked lines
    thickness = 2
    line_spacing = 25 # Pixels between the two lines of text
    dot_radius = 20   # As requested
    vertical_margin = 15 # Space between the top of the dot and the bottom of the text

    for side in ['left', 'right']:
        if lane_ep[side] and anchor_points_rw[side]:
            for point_type in ['start', 'end']:
                pixel_pos = lane_ep[side][point_type]
                rw_key = f"{point_type}_meters"
                rw_val = anchor_points_rw[side][rw_key]

                # 1. Create two separate lines of text
                line1 = f"X: {rw_val[0]:.2f}m"
                line2 = f"Y: {rw_val[1]:.1f}m"

                # 2. Calculate position to be ABOVE the dot
                # Shift X to center the text relative to the dot
                text_x = pixel_pos[0] - 60 
                # Shift Y up: dot_radius + margin + total height of both lines
                text_y_base = pixel_pos[1] - dot_radius - vertical_margin - line_spacing

                color = (0, 255, 0) if side == 'left' else (255, 0, 0)

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
                cv2.circle(frame, pixel_pos, dot_radius + 2, (0, 0, 0), -1, cv2.LINE_AA)
                # Main White circle
                cv2.circle(frame, pixel_pos, dot_radius, (255, 0, 0), -1, cv2.LINE_AA)
                # Small colored center dot for side identification
                cv2.circle(frame, pixel_pos, 5, color, -1, cv2.LINE_AA)

    return frame






######################################################################3
## streaming
# cap = cv2.VideoCapture(0)

## offline
video_path = "rec1.mp4" 
# video_path = "rec2c.mp4" 
cap = cv2.VideoCapture(video_path)


## not good at fixing the buffer caused frame jump issue
# cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
# cap.set(cv2.CAP_PROP_FPS, 15)
# for _ in range(3):
#     cap.grab()
# ret, frame = cap.retrieve()


if not cap.isOpened():
    print("no camera found")
    exit()

frame_skip = 2
count = 0
tracker = LaneTracker(max_history=500) 


## output mp4 format
fps = cap.get(cv2.CAP_PROP_FPS) or 30.0  # Default to 30 if metadata is missing
fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
video_output = cv2.VideoWriter('lane_detection_output.mp4', fourcc, fps, (w, h))


while True:

    ## read/load stream
    ret, frame = cap.read()

    if not ret:
        print("stream reading fail")
        break

    count += 1
    if count % frame_skip != 0:
        continue   # skip processing

    frame = cv2.resize(frame, (w, h))

    img_height, img_width = frame.shape[:2]
    
    ori_img = frame.copy()

    ## basic add center of image point     
    cv2.circle(frame, (img_width//2, img_height//2), 10, (0,0,0), -1)
    pt1 = (0, img_height//2)
    pt2 = (img_width, img_height//2)
    cv2.line(frame, pt1, pt2, (0,0,0), 2)


    #################
    
    # frame
    frame, project_points, mask = est_distance_grid(frame)

    
    ## 10m mask
    mask_colored = np.zeros_like(frame)
    mask_colored[:] = (0, 255, 0)  # BGR for Green
    mask_overlay = cv2.bitwise_and(mask_colored, mask_colored, mask=mask)
    frame = cv2.addWeighted(frame, 1.0, mask_overlay, 0.2, 0)
    

    
    ##########################

    ## edge & Lane detection
    # edges, lines = edge_det(frame, hough_thr, blur_thr)
    edges, lines = edge_det_mask_range(ori_img, hough_thr, blur_thr, mask)
    # edges, lines = edge_det_mask_range_conn(ori_img, hough_thr, blur_thr, mask)
    
    frame = add_edge_on_img(frame, edges)
    
    ll,rl,mid_p,err_x, err_h = find_LR2(lines, img_width, img_height, angle_thr_deg)
    ## add left/right parking lane to UI
    # frame, lane_ep = add_lanes_on_img(frame, ll, rl, mask, project_points)
    ## !!! try find connected max support line after find the intersection points between line equation and mask border 
    frame, lane_ep = add_lanes_on_img_with_endpoints(frame, ll, rl, mask, project_points, edges, 1, 50)
    print(lane_ep)

    anchor_points_rw = find_anchor_points(lane_ep)
    
    for side, coords in anchor_points_rw.items():
        if coords:
            print(f"--- {side.upper()} LANE ---")
            print(f"Start (1m mark): X={coords['start_meters'][0]:.2f}m, Y={coords['start_meters'][1]:.2f}m")
            print(f"End (10m mark):  X={coords['end_meters'][0]:.2f}m, Y={coords['end_meters'][1]:.2f}m")

    # 2. Inside your frame-by-frame loop:
    # Assuming real_world_coords is the output from our previous step
    tracker.update(anchor_points_rw)

    # --- Integration Example ---
    frame = draw_distance_labels(frame, lane_ep, anchor_points_rw)


    
    ## if mid of 2 lanes detected, add target point on image, add txt UI
    if mid_p:

        goal = int(mid_p)
        curr = int(img_width//2)
        
        # adjustment in pixel
        error_pixel = goal-curr
        
        ## adjustment map to meter
        adj = dyn_adj_dis_simple(hfov_deg, img_width, distance, error_pixel)
        # adj = dyn_adj_dis(hfov_deg, vfov_deg, tilt_deg, cam_height, img_width, img_height, error_pixel)
        
        add_deb_info(frame, mid_p, img_height, img_width, adj)


    cv2.imshow('stream', frame)
    video_output.write(frame)
    
    if cv2.waitKey(1) & 0xFF==ord('q'):
    # if cv2.waitKey(100) & 0xFF==ord('q'): ## 10ms per frame
        break


cap.release()
cv2.destroyAllWindows()

# Access the history tracking data of anchor points
full_path = tracker.get_history_list()
print(f"Tracked {len(full_path['right_start'])} frames of data.")
print(full_path)

with open("track_test.txt", 'w') as f:
    json.dump(full_path, f)

# plot_lane_history(full_path)

filter_full_path = lane_history_filtered(full_path)
plot_lane_history(filter_full_path)



