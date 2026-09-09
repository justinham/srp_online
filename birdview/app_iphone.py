
from flask import Flask, render_template, request, jsonify
import json
import math
import ast



import numpy as np
from scipy.interpolate import CubicHermiteSpline

from scipy.interpolate import CubicSpline, make_interp_spline

from pynmeagps import NMEAReader

from scipy.interpolate import UnivariateSpline

# import pandas as pd
import os
import logging

processes = []
processes_bg = []




## trailer

def intersection_tra(p0_3d, heading_start_deg, p3_3d, heading_end_deg, n=200):
    # Extract 2D positions (ignoring the 3rd dimension/heading component from your input array)
    p0 = p0_3d[:2]
    p3 = p3_3d[:2]
    
    # 1. Convert headings to 2D unit direction vectors
    # Based on your code's convention: 0 deg = +X, 90 deg = +Y (Counter-Clockwise)
    rad_start = np.radians(heading_start_deg)
    rad_end = np.radians(heading_end_deg)
    
    v_start = np.array([np.cos(rad_start), np.sin(rad_start)])
    v_end = np.array([np.cos(rad_end), np.sin(rad_end)])

    intersection_point = (p0 + p3) / 2.0   
    
    try:
        # 2. Find the intersection of the two heading lines: 
        # Line 1: p0 + t * v_start
        # Line 2: p3 + s * v_end
        # Set them equal: t * v_start - s * v_end = p3 - p0
        A = np.column_stack((v_start, -v_end))
        b = p3 - p0

        scales = np.linalg.solve(A, b)
        t, s = scales[0], scales[1]
        intersection_point = p0 + t * v_start


     
        is_ahead_of_p0 = t > 0
        is_between_points = (t > 0) and (s < 0) 
        # print("Is it ahead of p0?", t, s)    
            
        if is_ahead_of_p0:
            intersection_point = (p0 + p3) / 2.0   
            print("intersection ahead (0,0)")   
        elif is_between_points:
            intersection_point = p0 + scales[0] * v_start
            print("intersection in between")
        else:
            intersection_point = p0 + scales[0] * v_start ## avoid sharp change
            # intersection_point = (p0 + p3) / 2.0  ## avoid too far
            print("intersection behind target")

    
    except np.linalg.LinAlgError:
        # Handle parallel headings by placing control points halfway between endpoints
        intersection_point = (p0 + p3) / 2.0
        print("Warning: Headings are parallel. Using midpoint fallback.")

    return intersection_point




def projection_tra(p0_3d, heading_start_deg, p3_3d, heading_end_deg=None, n=200):
    # Extract 2D positions (ignoring 3rd dimension/heading)
    p0 = p0_3d[:2]
    p3 = p3_3d[:2]
    
    # 1. Convert p0 heading to a 2D unit direction vector
    # 0 deg = +X, 90 deg = +Y (Counter-Clockwise)
    rad_start = np.radians(heading_start_deg)
    v_start = np.array([np.cos(rad_start), np.sin(rad_start)])
    
    # 2. Vector from p0 to p3
    vec_to_p3 = p3 - p0
    
    # 3. Scalar projection using dot product
    # This gives the signed distance 't' along v_start where p3 projects
    t = np.dot(vec_to_p3, v_start)
    
    # 4. Calculate the actual 2D projection point coordinates
    projection_point = p0 + t * v_start
    
    # Directional checks (similar to your original debug prints)
    if t < 0:
        print("Projection is behind p0")
    elif t > np.linalg.norm(vec_to_p3): 
        # If t is greater than the distance to p3, it's out in front
        print("Projection is ahead of target")
    else:
        print("Projection is between p0 and target alignment")
        
    return projection_point




def intersection_tra_J_coor(p0_3d, heading_start_deg, p3_3d, heading_end_deg, n=200):
    # Extract 2D positions (ignoring the 3rd dimension/heading component)
    p0 = p0_3d[:2]
    p3 = p3_3d[:2]
    
    # 1. Convert headings to 2D unit direction vectors using custom system:
    # 0 deg = +Y (Up), 90 deg = +X (Left), clockwise accumulation
    rad_start = np.radians(heading_start_deg)
    rad_end = np.radians(heading_end_deg)
    
    v_start = np.array([np.sin(rad_start), np.cos(rad_start)])
    v_end = np.array([np.sin(rad_end), np.cos(rad_end)])
    
    # Default fallback initialization
    intersection_point = (p0 + p3) / 2.0
    
    try:
        # 2. Find the intersection of the two heading lines:
        # Line 1: p0 + t * v_start
        # Line 2: p3 + s * v_end
        # System: t * v_start - s * v_end = p3 - p0
        A = np.column_stack((v_start, -v_end))
        b = p3 - p0
        scales = np.linalg.solve(A, b)
        t, s = scales[0], scales[1]
        
        # Calculate possible intersection coordinates
        calculated_intersection = p0 + t * v_start
        
        # Validation checks based on your original flow parameters
        is_ahead_of_p0 = t > 0
        is_between_points = (t > 0) and (s < 0)  
        
        if is_ahead_of_p0:
            intersection_point = (p0 + p3) / 2.0
            print("intersection ahead (0,0)")
        elif is_between_points:
            intersection_point = calculated_intersection
            print("intersection in between")
        else:
            intersection_point = calculated_intersection
            print("intersection behind target")
            
    except np.linalg.LinAlgError:
        # Handle parallel headings cleanly
        intersection_point = (p0 + p3) / 2.0
        print("Warning: Headings are parallel. Using midpoint fallback.")
        
    return intersection_point


def projection_tra_J_coor(p0_3d, heading_start_deg, p3_3d, heading_end_deg=None, n=200):
    # Extract 2D positions (ignoring 3rd dimension/heading component)
    p0 = p0_3d[:2]
    p3 = p3_3d[:2]
    
    # 1. Convert heading to 2D unit vector using custom system:
    # 0 deg = +Y (Up), 90 deg = +X (Left), clockwise accumulation
    rad_start = np.radians(heading_start_deg)
    v_start = np.array([np.sin(rad_start), np.cos(rad_start)])
    
    # 2. Vector from p0 to p3
    vec_to_p3 = p3 - p0
    
    # 3. Scalar projection using dot product
    # Gives the signed distance 't' along p0's heading vector
    t = np.dot(vec_to_p3, v_start)
    
    # 4. Calculate the 2D projection coordinates
    projection_point = p0 + t * v_start
    
    # Directional checks matching your validation logic
    if t < 0:
        print("Projection is behind p0")
    elif t > np.linalg.norm(vec_to_p3):
        print("Projection is ahead of target alignment")
    else:
        print("Projection is between p0 and target alignment")
        
    return projection_point


def get_heading_p_to_p0(p, p0, degrees=True):
	x, y = p
	x0, y0 = p0

	dx = x0 - x
	dy = y0 - y

	# Angle in radians in range [-pi, pi]
	rad = math.atan2(dy, dx)

	# Normalize to [0, 2*pi)
	rad_normalized = rad % (2 * math.pi)

	if degrees:
		return math.degrees(rad_normalized)  # Returns [0, 360)
	return rad_normalized





def compute_bezier_tra(p0, p1, p2, p3, n=200):
	t = np.linspace(0, 1, n)[:, None]

	# Position interpolation
	curve = (1 - t)**3 * p0 + \
			3 * (1 - t)**2 * t * p1 + \
			3 * (1 - t) * t**2 * p2 + \
			t**3 * p3

	# Corrected analytical derivative direction vectors
	dcurve = 3 * (1 - t)**2 * (p1 - p0) + \
			 6 * (1 - t) * t * (p2 - p1) + \
			 3 * t**2 * (p3 - p2)

	# Extract X (front) and Y (left) components
	dx = dcurve[:, 0]
	dy = dcurve[:, 1]

	# Calculate heading: 0 deg at +X, 90 deg at +Y (counter-clockwise)
	# headings = np.degrees(np.arctan2(dy, dx))
	headings = (np.degrees(np.arctan2(dcurve[:, 0], dcurve[:, 1])) + 360) % 360
	
	headings = (headings + 360) % 360

	return curve, headings


def compute_bezier_tra_j(p0, p1, p2, p3, n=200):
    t = np.linspace(0, 1, n)[:, None]
    
    # Position interpolation (remains valid)
    curve = (1 - t)**3 * p0 + \
            3 * (1 - t)**2 * t * p1 + \
            3 * (1 - t) * t**2 * p2 + \
            t**3 * p3
            
    # Analytical derivative direction vectors (remains valid)
    dcurve = 3 * (1 - t)**2 * (p1 - p0) + \
             6 * (1 - t) * t * (p2 - p1) + \
             3 * t**2 * (p3 - p2)
    
    # Extract components based on your system: index 0 is X (Left), index 1 is Y (Up)
    dx = dcurve[:, 0]
    dy = dcurve[:, 1]
    
    # CORRECTED HEADING CALCULATION:
    # Under standard trig, arctan2(y, x) targets 0° at +X and increases CCW.
    # To target 0° at +Y and increase CW towards +X, we use arctan2(dx, dy).
    headings = (np.degrees(np.arctan2(dx, dy)) + 360) % 360
    
    return curve, headings


def generate_safe_bezier_deg(p0, p3, heading0_deg, heading3_deg, min_radius=10.0):
    """
    Computes P1 and P2 for REVERSE motion trajectory.
    Frame: +X is Up, +Y is Left.
    Heading: 0 deg = +X (Up), 90 deg = +Y (Left), Counter-Clockwise orientation.
    """
    p0 = np.array(p0, dtype=float)
    p3 = np.array(p3, dtype=float)

    rad0 = np.radians(heading0_deg)
    rad3 = np.radians(heading3_deg)

    # SWAPPED COMPONENTS: cos for X (Up), sin for Y (Left)
    dir0 = np.array([np.cos(rad0), np.sin(rad0)])
    dir3 = np.array([np.cos(rad3), np.sin(rad3)])

    dist = np.linalg.norm(p3 - p0)
    handle_len = min(dist * 0.4, max(0.552 * min_radius, dist * 0.35))

    # REVERSE MOTION:
    # P1 extends out the back of start point (-dir0)
    p1 = p0 - handle_len * dir0
    
    # P2 sits in front of destination (+dir3)
    p2 = p3 + handle_len * dir3

    return p0, p1, p2, p3

def generate_safe_bezier_deg_j(p0, p3, heading0_deg, heading3_deg, min_radius=10.0):
    """
    Computes P1 and P2 for REVERSE motion trajectory.
    Frame: +X is Left, +Y is Up. 0 deg = +Y (Up), CCW towards +X.
    """
    p0 = np.array(p0, dtype=float)
    p3 = np.array(p3, dtype=float)

    rad0 = np.radians(heading0_deg)
    rad3 = np.radians(heading3_deg)

    # Unit direction vectors facing the FRONT of the vehicle
    dir0 = np.array([np.sin(rad0), np.cos(rad0)])
    dir3 = np.array([np.sin(rad3), np.cos(rad3)])

    dist = np.linalg.norm(p3 - p0)
    handle_len = min(dist * 0.4, max(0.552 * min_radius, dist * 0.35))

    # REVERSE MOTION FLIPS THE SIGNS:
    # P1 goes OPPOSITE to front-facing heading (moving out the rear of start point)
    p1 = p0 - handle_len * dir0
    
    # P2 sits IN FRONT of destination (vehicle backs into P3 from P2)
    p2 = p3 + handle_len * dir3

    return p0, p1, p2, p3


def rotate_by_0(deg):

	fn = "pathx5_can_heading"
	with open("hummer_path/%s.txt"%fn, "r") as f:
		message = json.loads(f.readline())

	pts = message
	theta = np.deg2rad(deg)

	theta_deg = deg
	theta = math.radians(theta_deg)

	x0, y0, _ = pts[0]


	rotated = []
	for x, y, h in pts:
		dx = x - x0
		dy = y - y0

		xr = x0 + dx * math.cos(theta) - dy * math.sin(theta)
		yr = y0 + dx * math.sin(theta) + dy * math.cos(theta)

		hr = (h - theta_deg) % 360   # or just h - 1.0 if you don't want wrapping

		rotated.append([xr, yr, hr])

	print("after rotation", rotated)
	return rotated


def get_average_coordinates(file_path):
	x_sum = 0
	y_sum = 0
	count = 0

	try:
		with open(file_path, 'r') as file:
			for line in file:
				line = line.strip()
				if not line:
					continue
				
				# Safely parse the string "[x, y]" into a Python list
				try:
					coords = ast.literal_eval(line)
					if isinstance(coords, list) and len(coords) == 2:
						x_sum += coords[0]
						y_sum += coords[1]
						count += 1
				except (ValueError, SyntaxError) as e:
					print(f"Skipping malformed line: {line}")
				print(line)	

		if count > 0:
			avg_x = x_sum / count
			avg_y = y_sum / count
			return avg_x, avg_y, count
		else:
			return None, None, 0

	except FileNotFoundError:
		print("The file was not found.")
		return None, None, 0


def launch_process(cmd, shell=False):

	# password = "ConnAu\n"
	# p = subprocess.run(cmd, shell=True, executable="/bin/bash", input=password.encode(), check=True)
	p = subprocess.Popen(cmd, shell=shell, preexec_fn=os.setsid)

	processes.append(p)


def cleanup():
	print("\nStopping all ROS nodes and scripts...")
	for p in processes:
		try:
			os.killpg(os.getpgid(p.pid), signal.SIGTERM)
		except Exception as e:
			print(f"Error stopping process: {e}")



# Suppress all HTTP request logs (including 200 responses)
log = logging.getLogger('werkzeug')
log.setLevel(logging.WARNING)



# import matplotlib.pyplot as plt
show_fig = False


###############


## find p2 based on intersection
def find_sta_turn_p_bri(points):

	p2 = [0,0,0]
	start_turn_adv_dis = 8.0
	
	# Extract p1, p3, and p4
	p1 = points[0, :2]  # (x, y)
	p3 = points[2, :2]
	p4 = points[3, :2]
	p5 = points[4, :2]
	# print(p1)
	xs = [p1[0], p3[0], p4[0], p5[0]]
	ys = [p1[1], p3[1], p4[1], p5[1]]

	## init with p0
	p2 = points[0].tolist()

	heading_rad = np.radians(points[0][2])
	dir_vector = np.array([np.sin(heading_rad), np.cos(heading_rad)])  # [x, y] direction

	# Set up linear system: p1 + t*d = p3 + s*(p4 - p3)
	A = np.column_stack((dir_vector, -(p4 - p3)))
	b = p3 - p1[:2]

	# Solve for [t, s] if possible
	try:
		ts = np.linalg.lstsq(A, b, rcond=None)[0]
		t, s = ts

		# Calculate intersection point
		intersection = p1[:2] + t * dir_vector
		base_dis = ((intersection[0]-p1[0])**2+(intersection[1]-p1[1])**2)**0.5

		if base_dis>start_turn_adv_dis:
			
			to_p1_vector = p1[:2] - intersection
			distance = np.linalg.norm(to_p1_vector)

			# Check to avoid division by zero
			if distance == 0:
				moved_point = intersection  # Already at p1
			else:
				direction = to_p1_vector / distance  # Normalize
				moved_point = intersection + direction * start_turn_adv_dis  # Move 8 meters toward p1

			# Output the adjusted point
			# print("Moved intersection point (8m toward p1):", moved_point)

			p2 = [moved_point[0],moved_point[1],points[0][2]]


	except np.linalg.LinAlgError:
		print("No unique intersection found — lines may be parallel or ill-defined.")

	return p2


def find_sta_turn_p_bri_trailer(points):

	p2 = [0,0,0]
	start_turn_adv_dis = 8.0
	
	# Extract p1, p3, and p4
	p1 = points[0, :2]  # (x, y)
	p3 = points[2, :2]
	p4 = points[3, :2]
	p5 = points[4, :2]
	# print(p1)
	xs = [p1[0], p3[0], p4[0], p5[0]]
	ys = [p1[1], p3[1], p4[1], p5[1]]

	## init with p0
	p2 = points[0].tolist()

	heading_rad = np.radians(points[0][2])
	dir_vector = np.array([np.sin(heading_rad), np.cos(heading_rad)])  # [x, y] direction

	# Set up linear system: p1 + t*d = p3 + s*(p4 - p3)
	A = np.column_stack((dir_vector, -(p4 - p3)))
	b = p3 - p1[:2]

	# Solve for [t, s] if possible
	try:
		ts = np.linalg.lstsq(A, b, rcond=None)[0]
		t, s = ts

		# Calculate intersection point
		intersection = p1[:2] + t * dir_vector
		base_dis = ((intersection[0]-p1[0])**2+(intersection[1]-p1[1])**2)**0.5

		if base_dis>start_turn_adv_dis:
			
			to_p1_vector = p1[:2] - intersection
			distance = np.linalg.norm(to_p1_vector)

			# Check to avoid division by zero
			if distance == 0:
				moved_point = intersection  # Already at p1
			else:
				direction = to_p1_vector / distance  # Normalize
				moved_point = intersection + direction * start_turn_adv_dis  # Move 8 meters toward p1

			# Output the adjusted point
			# print("Moved intersection point (8m toward p1):", moved_point)

			p2 = [moved_point[0],moved_point[1],points[0][2]]


	except np.linalg.LinAlgError:
		print("No unique intersection found — lines may be parallel or ill-defined.")

	return p2

def move_point_towards(a, b, distance=5.0):
	"""
	a: tuple (x, y) - Starting point
	b: tuple (x, y) - Target point
	distance: float - How far to move (e.g., 5m)
	"""
	dx = b[0] - a[0]
	dy = b[1] - a[1]
	
	current_dist = math.sqrt(dx**2 + dy**2)
	
	# Avoid division by zero if points are at the same location
	if current_dist == 0:
		return a
	
	# Calculate the new position
	# If distance > current_dist, a will move past b
	new_x = a[0] + (dx / current_dist) * distance
	new_y = a[1] + (dy / current_dist) * distance
	
	return (new_x, new_y)

def smooth_component(arr, t, t_smooth, k=2, smooth_factor=None):
	"""
	Smooth a data sequence using UnivariateSpline with specified smoothness.
	"""
	s = smooth_factor or (len(arr) * np.var(arr) * 0.001)  # baseline smoothing
	# print("sf", s)
	spline = UnivariateSpline(t, arr, k=k, s=s)  # s>0 for smoothing :contentReference[oaicite:2]{index=2}
	return spline(t_smooth)


def path_gen_for_eh(pts_deg, tot_point):

	latlogs = [[x[0], x[1]] for x in pts_deg]
	pts = np.array(latlogs)  # lat/lon list
	# pts = np.array(pts_deg)  # lat/lon list
	lat0, lon0 = pts[0]
	R = 6371000
	cosLat0 = np.cos(np.radians(lat0))


	xyh = np.zeros((len(pts_deg), 3))
	for i, (lat, lon, head) in enumerate(pts_deg):
		dlat = np.radians(lat - lat0)
		dlon = np.radians(lon - lon0)
		x = R * dlon * cosLat0
		y = R * dlat
		xyh[i] = [x, y, head]

	xyh[:, 2] = np.unwrap(xyh[:, 2])  # ensures continuous heading signal :contentReference[oaicite:1]{index=1}

	t = np.arange(len(xyh))
	t_smooth = np.linspace(0, len(xyh) - 1, tot_point)


	x_s = smooth_component(xyh[:, 0], t, t_smooth)
	y_s = smooth_component(xyh[:, 1], t, t_smooth)
	h_s = smooth_component(xyh[:, 2], t, t_smooth)

	lat_s = lat0 + np.degrees(y_s / R)
	lon_s = lon0 + np.degrees(x_s / (R * cosLat0))
	h_s = (h_s + np.pi) % (2 * np.pi) - np.pi  # wrap into [-π,π)

	return lat_s, lon_s, h_s, x_s, y_s




def path_antenna_cali(ant_offset):

	with open("hummer_path/pathx5.txt", "r") as f:
		line = f.readline()
	path_loc = json.loads(line)
	# f.close()
	pointx5 = path_loc

	new_pointx5 = []
	center_offset = []
	center_offset_uni = [0,0]
	for idx, p in enumerate(pointx5):
		h = p[2] * 3.1416 / 180
		dx = -ant_offset * math.sin(h) 
		dy = -ant_offset * math.cos(h) 
		new_pointx5.append([p[0]+dx, p[1]+dy, p[2]])
		if idx==0:
			center_offset_uni = [dx,dy]
		center_offset.append([dx, dy])

	xs = [x[0] for x in pointx5]
	ys = [x[1] for x in pointx5]
	xs_new = [x[0] for x in new_pointx5]
	ys_new = [x[1] for x in new_pointx5]

	# new coordinate use antenna as center
	pointx5_ac = []
	new_pointx5_ac = [] 
	for i in range(len(pointx5)):
		# pointx5_ac.append([pointx5[i][0]-center_offset[i][0], pointx5[i][1]-center_offset[i][1], pointx5[i][2]])
		# new_pointx5_ac.append([new_pointx5[i][0]-center_offset[i][0], new_pointx5[i][1]-center_offset[i][1], new_pointx5[i][2]])
		pointx5_ac.append([pointx5[i][0]-center_offset_uni[0], pointx5[i][1]-center_offset_uni[1], pointx5[i][2]])
		new_pointx5_ac.append([new_pointx5[i][0]-center_offset_uni[0], new_pointx5[i][1]-center_offset_uni[1], new_pointx5[i][2]])

	xs = [x[0] for x in pointx5_ac]
	ys = [x[1] for x in pointx5_ac]
	xs_new = [x[0] for x in new_pointx5_ac]
	ys_new = [x[1] for x in new_pointx5_ac]

	with open("hummer_path/pathx5_antenna_cali.txt", "w") as f:
		json.dump(new_pointx5_ac, f)





def show_path(points, title):

	points = np.array(points)
	x = points[:, 0]
	y = points[:, 1]
	headings_deg = points[:, 2]

	# Convert heading to radians
	headings_rad = np.radians(headings_deg)

	# Compute direction vectors (unit vector scaled)
	arrow_length = 2.0
	u = np.sin(headings_rad) * arrow_length   # X direction (East)
	v = np.cos(headings_rad) * arrow_length   # Y direction (North)

	# Plot points and heading arrows
	plt.figure(figsize=(6, 6))
	plt.quiver(x, y, u, v, angles='xy', scale_units='xy', scale=1, color='blue')

	# Mark the points
	plt.scatter(x, y, color='red')
	for i in range(len(x)):
		plt.text(x[i] + 0.5, y[i] + 0.5, f"{round(headings_deg[i],2)}°", color='black')

	# Labels and grid
	plt.xlabel("East (meters)")
	plt.ylabel("North (meters)")
	plt.title("%s (0° = North, Clockwise)"%title)
	plt.grid(True)
	plt.axis('equal')
	plt.xlim(-5, 30)
	plt.ylim(-15, 20)

	plt.show()


def move_back(point, back_distance):
	
	heading_deg = point[2]
	heading_rad = math.radians(heading_deg)
	dx = -back_distance * math.sin(heading_rad)  # East-West offset
	dy = -back_distance * math.cos(heading_rad)  # North-South offset

	# move down
	new_x = point[0] + dx
	new_y = point[1] + dy
	new_h = point[2]
	p_back = [new_x, new_y, new_h]
	return p_back


def move_forward(point, back_distance):
	
	heading_deg = point[2]
	heading_rad = math.radians(heading_deg)
	dx = -back_distance * math.sin(heading_rad)  # East-West offset
	dy = -back_distance * math.cos(heading_rad)  # North-South offset

	# move down
	new_x = point[0] - dx
	new_y = point[1] - dy
	new_h = point[2]
	p_back = [new_x, new_y, new_h]
	return p_back



def anchor_forward(points, le_vs_ri):
	
	# Convert heading (clockwise from North) to radians
	p_e = points[-1]
	back_distance = 6.0  

	## move back 6, turn right 90, move back 6
	p1_n = move_back(p_e, back_distance)
	p1_n[2] = (p1_n[2]+90)%360
	if le_vs_ri=="right":
		p1_n[2] = (p1_n[2]-180)%360

	p2_n = move_back(p1_n, back_distance)
	
	points_n = [points[0], p2_n, points[1]]
	# print(points_n)
	return points_n


def anchor_forward_2stage(points, le_vs_ri):
	
	# Convert heading (clockwise from North) to radians
	p_e = points[-1]
	back_distance = 6.0  

	## move back 6, turn right 90, move forward 6, turn right 45
	p1_n = move_back(p_e, back_distance)
	p1_n[2] = (p1_n[2]+90)%360
	if le_vs_ri=="right":
		p1_n[2] = (p1_n[2]-180)%360

	p2_n = move_forward(p1_n, back_distance)
	p2_n[2] = (p2_n[2])%360
	
	## end point reverse
	p_e_h = (points[-1][2]+180)%360
	p_f = [points[1][0], points[1][1], p_e_h]
	
	points_n = [points[0], p2_n, p_f]
	# print(points_n)
	return points_n


def anchor_backward(points, le_vs_ri):
	
	# Convert heading (clockwise from North) to radians
	p_e = points[-1]
	back_distance = 6.0  

	## move back 6, turn right 90, move back 6
	p1_n = move_back(p_e, back_distance)
	p1_n[2] = (p1_n[2]+90)%360
	if le_vs_ri=="left":
		p2_n = move_back(p1_n, -back_distance)
	elif le_vs_ri=="right":
		p2_n = move_back(p1_n, back_distance)
		p2_n[2] = (p2_n[2]+180)%360

	p_e_h = (points[-1][2]+180)%360
	
	p_f = [points[1][0], points[1][1], p_e_h]
	 
	points_n = [points[0], p2_n, p_f]
	# print(points_n)
	return points_n


def anchor_backward_2stage(points, le_vs_ri):
	
	# Convert heading (clockwise from North) to radians
	p_e = points[-1]
	back_distance = 6.0  

	## move back 6, turn right 90, move forward 6, turn right 45
	p1_n = move_back(p_e, back_distance)
	p1_n[2] = (p1_n[2]+90)%360
	if le_vs_ri=="left":	
		p2_n = move_forward(p1_n, -back_distance)
	elif le_vs_ri=="right":
		p2_n = move_forward(p1_n, back_distance)
		p2_n[2] = (p2_n[2]+180)%360
	
	## end point reverse
	# points[-1][2] = (points[-1][2]+180)%360
	points_n = [points[0], p2_n, points[1]]
	# print(points_n)
	return points_n



def rel_turning_direction(start_h, end_h):
	direction = "same"
	delta = (end_h -start_h + 360) % 360

	if delta <= 180:
		direction = "clockwise"
	elif delta > 180:
		direction = "counter-clockwise"
	return direction
	

def heading_to_unit_vector(heading_deg):
	rad = np.radians(heading_deg)
	dx = np.sin(rad)
	dy = np.cos(rad)
	return np.array([dx, dy])


def intersect_lines(p1, dir1, p2, dir2):
	# Solve: p1 + t1 * dir1 = p2 + t2 * dir2
	A = np.array([dir1, -dir2]).T
	b = np.array(p2) - np.array(p1)
	if np.linalg.matrix_rank(A) < 2:
		return None  # lines are parallel or colinear
	t = np.linalg.solve(A, b)
	intersection = np.array(p1) + t[0] * dir1
	return intersection


def is_facing_away(p, heading_deg, intersection):
	dir_vec = heading_to_unit_vector(heading_deg)
	to_intersection = intersection - np.array(p)
	return np.dot(to_intersection, dir_vec) < 0


## interpulation

## bezier curve
def normalize_heading(h):
	"""Normalize heading to [0, 360)"""
	return h % 360

def heading_to_direction(heading_deg):
	"""
	Converts heading in degrees (0 = North, clockwise) to unit direction vector.
	"""
	heading_rad = np.radians(normalize_heading(heading_deg))
	dx = np.sin(heading_rad)  # X is sin
	dy = np.cos(heading_rad)  # Y is cos
	return np.array([dx, dy])

def compute_bezier(p0, p1, p2, p3, n=50):
	"""
	Compute cubic Bezier curve from control points.
	"""
	t = np.linspace(0, 1, n)[:, None]

	curve = (1 - t)**3 * p0 + \
		   3 * (1 - t)**2 * t * p1 + \
		   3 * (1 - t) * t**2 * p2 + \
		   t**3 * p3

	#deviation on curve
	dcurve = -3 * (1 - t)**2 * p0 + \
			 3 * (1 - t)**2 * p1 - 6 * (1 - t) * t * p1 + \
			 6 * (1 - t) * t * p2 - 3 * t**2 * p2 + \
			 3 * t**2 * p3

	# Heading from tangent: Y=0 deg, clockwise
	headings = (np.degrees(np.arctan2(dcurve[:, 0], dcurve[:, 1])) + 360) % 360
	
	return curve, headings

def generate_path(points, heading_scale=2.0, resolution=50):
	"""
	Generate smooth path using Bezier curves from points with headings.
	"""
	curves = []
	headings = []
	for i in range(len(points) - 1):
		x0, y0, h0 = points[i]
		x1, y1, h1 = points[i + 1]
		
		p0 = np.array([x0, y0])
		p3 = np.array([x1, y1])
		
		d0 = heading_to_direction(h0)
		d1 = heading_to_direction(h1)
		
		p1 = p0 + d0 * heading_scale
		p2 = p3 - d1 * heading_scale
		
		bezier,heading = compute_bezier(p0, p1, p2, p3, n=resolution)
		curves.append(bezier)
		headings.append(heading)
		# print(headings)
	
	return np.vstack(curves), np.vstack(headings[0])

def plot_path_with_headings(points, path, arrow_scale=0.8):
	"""
	Plot the Bezier path and show heading directions at control points.
	"""
	plt.figure(figsize=(6, 6))
	plt.scatter(path[:, 0], path[:, 1], alpha=0.5, label='interpulation')
	plt.plot(path[:, 0], path[:, 1], alpha=0.5, label='Bezier Path')

	for x, y, h in points:
		plt.plot(x, y, 'ro')
		dir_vec = heading_to_direction(h)
		plt.arrow(x, y, dir_vec[0]*arrow_scale, dir_vec[1]*arrow_scale,
				  head_width=0.3, head_length=0.4, fc='r', ec='r')

	plt.axis('equal')
	plt.grid(True)
	plt.title("Bezier Curve Path with Heading Tangents")
	plt.legend()
	plt.tight_layout()
	plt.show()


def rel_loc_ana_and_path_inter_trailer(fn, stage_resolution, fn1, fn2, gear):

	print("path option:", gear)

	with open("./hummer_path/%s.txt"%fn, "r") as f:
		line = f.readline()
	path_loc = json.loads(line)
	# f.close()
	pointx5 = path_loc

	if gear=="forward":	
		print(pointx5, '----')
		# path0, headings0 = generate_path(pointx5[:2], heading_scale=4.0, resolution=stage_resolution) #
		path1, headings1 = generate_path(pointx5[1:3], heading_scale=4.0, resolution=stage_resolution)
		path2, headings2 = generate_path(pointx5[2:4], heading_scale=4.0, resolution=stage_resolution)
		path3, headings3 = generate_path(pointx5[3:], heading_scale=4.0, resolution=stage_resolution)
		# path_sum = np.concatenate((path0, path1, path2, path3))
		# heading_sum = np.concatenate((headings0, headings1, headings2, headings3))
		
		# headings0 sharp change since point#1 and point#2 are too close
		headings0 = [[pointx5[0][2]]]*stage_resolution
		# path_sum = np.concatenate((path0, path1, path2, path3))
		path_sum = np.concatenate((path1, path2, path3))
		# heading_sum = np.concatenate((headings0, headings1, headings2, headings3))
		heading_sum = np.concatenate((headings1, headings2, headings3))
		
		# print(headings0)
		# print(pointx5[:2])
			
		points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum, heading_sum)]
			
		with open('hummer_path/%s.txt'%fn1, 'w', encoding='utf-8') as f:
			json.dump(points_one_stage_den, f, indent=2)
	

	# elif gear=="reverse":
	# 	path0, headings0 = generate_path(pointx5[:2], heading_scale=4.0, resolution=stage_resolution)
	# 	path1, headings1 = generate_path(pointx5[1:3], heading_scale=4.0, resolution=stage_resolution)
	# 	path2, headings2 = generate_path(pointx5[2:4], heading_scale=4.0, resolution=stage_resolution)
	# 	path3, headings3 = generate_path(pointx5[3:], heading_scale=4.0, resolution=stage_resolution)
	# 	# path_sum = np.concatenate((path0, path1, path2, path3))
	# 	# heading_sum = np.concatenate((headings0, headings1, headings2, headings3))
	# 	path_sum = np.concatenate((path1, path2, path3))
	# 	heading_sum = np.concatenate((headings1, headings2, headings3))
			
	# 	points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum, heading_sum)]
			
	# 	with open('hummer_path/%s.txt'%fn1, 'w', encoding='utf-8') as f:
	# 		json.dump(points_one_stage_den, f, indent=2)

	## 1 stage only, need to add 2-stage






def rel_loc_ana_and_path_inter_simple(fn, stage_resolution, fn1, fn2, gear):

	print("path option:", gear)

	with open("./hummer_path/%s.txt"%fn, "r") as f:
		line = f.readline()
	path_loc = json.loads(line)
	# f.close()
	pointx5 = path_loc

	if gear=="forward":	
		print(pointx5, '----')
		path0, headings0 = generate_path(pointx5[:2], heading_scale=4.0, resolution=stage_resolution)
		path1, headings1 = generate_path(pointx5[1:3], heading_scale=4.0, resolution=stage_resolution)
		path2, headings2 = generate_path(pointx5[2:4], heading_scale=4.0, resolution=stage_resolution)
		path3, headings3 = generate_path(pointx5[3:], heading_scale=4.0, resolution=stage_resolution)
		# path_sum = np.concatenate((path0, path1, path2, path3))
		# heading_sum = np.concatenate((headings0, headings1, headings2, headings3))
		
		# headings0 sharp change since point#1 and point#2 are too close
		headings0 = [[pointx5[0][2]]]*stage_resolution
		path_sum = np.concatenate((path0, path1, path2, path3))
		heading_sum = np.concatenate((headings0, headings1, headings2, headings3))
		
		print(headings0)
		print(pointx5[:2])
			
		points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum, heading_sum)]
			
		with open('hummer_path/%s.txt'%fn1, 'w', encoding='utf-8') as f:
			json.dump(points_one_stage_den, f, indent=2)
	

	# elif gear=="reverse":
	# 	path0, headings0 = generate_path(pointx5[:2], heading_scale=4.0, resolution=stage_resolution)
	# 	path1, headings1 = generate_path(pointx5[1:3], heading_scale=4.0, resolution=stage_resolution)
	# 	path2, headings2 = generate_path(pointx5[2:4], heading_scale=4.0, resolution=stage_resolution)
	# 	path3, headings3 = generate_path(pointx5[3:], heading_scale=4.0, resolution=stage_resolution)
	# 	# path_sum = np.concatenate((path0, path1, path2, path3))
	# 	# heading_sum = np.concatenate((headings0, headings1, headings2, headings3))
	# 	path_sum = np.concatenate((path1, path2, path3))
	# 	heading_sum = np.concatenate((headings1, headings2, headings3))
			
	# 	points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum, heading_sum)]
			
	# 	with open('hummer_path/%s.txt'%fn1, 'w', encoding='utf-8') as f:
	# 		json.dump(points_one_stage_den, f, indent=2)

	## 1 stage only, need to add 2-stage




def rel_loc_ana_and_path_inter(fn, stage_resolution, fn1, fn2):

	## load file
	with open("./hummer_path/%s.txt"%fn, "r") as f:
		line = f.readline()
	path_loc = json.loads(line)
	# f.close()
	pointx5 = path_loc
	start_p = path_loc[0]
	end_p = path_loc[3]

	points = [start_p, end_p]
	if show_fig:
		show_path(points, "original")

	## check f/b which to trigger
	f_vs_r = "tbd"
	le_vs_ri = "tbd"

	## check relative turning direction
	start_h = points[0][2]
	end_h = points[-1][2]
	rel_dir = rel_turning_direction(start_h, end_h)
	if (rel_dir=="clockwise"):
		le_vs_ri = "right"
	elif rel_dir=="counter-clockwise":
		le_vs_ri = "left"


	## check if the vehicle is moving closer or further to the spot with its current heading
	## check facing toward or away
	p1 = np.array([start_p[0], start_p[1]])
	h1 = start_p[2]

	p2 = np.array([end_p[0], end_p[1]])
	h2 = end_p[2]

	dir1 = heading_to_unit_vector(h1)
	dir2 = heading_to_unit_vector(h2)

	intersection = intersect_lines(p1, dir1, p2, dir2)

	if intersection is not None:
		facing_away_p1 = is_facing_away(p1, h1, intersection)
		facing_away_p2 = is_facing_away(p2, h2, intersection)

		# print("Intersection at:", intersection)

		if facing_away_p1:
			f_vs_r = "backward"
		else:
			f_vs_r = "forward"
	else:
		print("error, No intersection (parallel or colinear)")

	print("scenario decision:", f_vs_r, le_vs_ri)
	

	## based on 4 above scenarios, generate 8 paths and complete interpulation 
	points_one_stage_anchor = []
	points_two_stage_anchor = []
	points_one_stage_den = []
	points_two_stage_den = []

	## facing toward trajatory (very complicated logic...)
	## bizard curve scale=4.0 test best
	if (f_vs_r=="forward" and le_vs_ri=="left"):
		option = "tbd"

		option = "1 direct front in"
		print(option)
		points_n1 = anchor_forward(points, le_vs_ri)
		# print(points_n1)
		if show_fig:
			show_path(points_n1, option)
		points_one_stage_anchor = points_n1

		path1,headings1 = generate_path(points_n1, heading_scale=4.0, resolution=stage_resolution)
		path2,headings2 = generate_path(points_n1[-1:], heading_scale=4.0, resolution=stage_resolution)
		path_sum = np.concatenate((path1, path2))
		heading_sum = np.concatenate((headings1, headings2))
		# print(path_sum, len(path_sum))
		for xy,h in zip(path_sum, heading_sum):
			print(xy[0], xy[1], h[0])
		points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum, heading_sum)]
		
		## 2 point only
		path_2p,headings_2p = generate_path(points_n1[::2], heading_scale=4.0, resolution=stage_resolution)
		points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_2p, headings_2p)]
		
		# print(points_one_stage_den)

		if show_fig:
			plot_path_with_headings(points_one_stage_anchor, path_sum)

		points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum, headings_sum)]
		
		with open('hummer_path/%s.txt'%fn1, 'w', encoding='utf-8') as f:
			json.dump(points_one_stage_den, f, indent=2)
		
		
		option = "2 front shift back in"
		print(option)
		points_n2 = anchor_forward_2stage(points, le_vs_ri)
		# print(points_n2)
		if show_fig:
			show_path(points_n2, option)
		points_two_stage_anchor = points_n2

		back_list = points_n2[1:]
		back_list = back_list[::-1]
		path11,headings11 = generate_path(points_n2[:-1], heading_scale=4.0, resolution=stage_resolution)
		path22,headings22 = generate_path(back_list, heading_scale=4.0, resolution=stage_resolution)
		# print("rev", path22)
		path22 = path22[::-1]
		headings22 = headings22[::-1]
		path_sum2 = np.concatenate((path11, path22))
		heading_sum2 = np.concatenate((headings11, headings22))

		# for xy,h in zip(path_sum2, heading_sum2):
		# 	print(xy[0], xy[1], h[0])
		# print(path_sum2)
		points_two_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum2, heading_sum2)]
		for i in range(stage_resolution, stage_resolution*2):
			points_two_stage_den[i][3] = "r"
		# print(points_two_stage_den)
		if show_fig:
			plot_path_with_headings(points_two_stage_anchor, path_sum2)

		with open('hummer_path/%s.txt'%fn2, 'w', encoding='utf-8') as f:
			json.dump(points_two_stage_den, f, indent=2)
		
		# with open('hummer_path/path_local_den_2_stage_test2.txt', 'w', encoding='utf-8') as f:
		# 	json.dump(points_two_stage_den, f, indent=2)


		
	elif (f_vs_r=="forward" and le_vs_ri=="right"):
		option = "tbd"

		option = "3 direct front in"
		print(option)
		points_n1 = anchor_forward(points, le_vs_ri)
		# print(points_n1)
		if show_fig:
			show_path(points_n1, option)
		points_one_stage_anchor = points_n1

		path1,headings1 = generate_path(points_n1[:-1], heading_scale=4.0, resolution=stage_resolution)
		path2,headings2 = generate_path(points_n1[1:], heading_scale=4.0, resolution=stage_resolution)
		path_sum = np.concatenate((path1, path2))
		heading_sum = np.concatenate((headings1, headings2))
		# print(path_sum)
		# for xy,h in zip(path_sum, heading_sum):
		# 	print(xy[0], xy[1], h[0])
		points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum, heading_sum)]
		
		## 2 point only
		path_2p,headings_2p = generate_path(points_n1[::2], heading_scale=4.0, resolution=stage_resolution)
		points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_2p, headings_2p)]
		# print(points_one_stage_den)

		
		if show_fig:
			plot_path_with_headings(points_one_stage_anchor, path_sum)

		with open('hummer_path/%s.txt'%fn1, 'w', encoding='utf-8') as f:
			json.dump(points_one_stage_den, f, indent=2)
		


		option = "4 front shift back in"
		print(option)
		points_n2 = anchor_forward_2stage(points, le_vs_ri)
		# print(points_n2)
		if show_fig:
			show_path(points_n2, option)
		points_two_stage_anchor = points_n2

		back_list = points_n2[1:]
		back_list = back_list[::-1]
		path11,headings11 = generate_path(points_n2[:-1], heading_scale=4.0, resolution=stage_resolution)
		path22,headings22 = generate_path(back_list, heading_scale=4.0, resolution=stage_resolution)
		# print("rev", path22)
		path22 = path22[::-1]
		headings22 = headings22[::-1]
		path_sum2 = np.concatenate((path11, path22))
		heading_sum2 = np.concatenate((headings11, headings22))
		# for xy,h in zip(path_sum2, heading_sum2):
		# 	print(xy[0], xy[1], h[0])
		points_two_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum2, heading_sum2)]
		for i in range(stage_resolution, stage_resolution*2):
			points_two_stage_den[i][3] = "r"
		# print(points_two_stage_den)

		if show_fig:
			plot_path_with_headings(points_two_stage_anchor, path_sum2)

		with open('hummer_path/%s.txt'%fn2, 'w', encoding='utf-8') as f:
			json.dump(points_two_stage_den, f, indent=2)
		



	## facing away trajatory
	elif (f_vs_r=="backward" and le_vs_ri=="left"):
		option = "tbd"

		option = "5 direct back in"
		print(option)
		points_n1 = anchor_backward(points, le_vs_ri)
		# print(points_n1)
		if show_fig:
			show_path(points_n1, option)
		points_one_stage_anchor = points_n1

		back_list1 = points_n1[:-1]
		back_list1 = back_list1[::-1]
		path1,headings1 = generate_path(back_list1, heading_scale=4.0, resolution=stage_resolution)
		path1 = path1[::-1]
		headings1 = headings1[::-1]
		back_list2 = points_n1[1:]
		back_list2 = back_list2[::-1]
		path2,headings2 = generate_path(back_list2, heading_scale=4.0, resolution=stage_resolution)
		path2 = path2[::-1]
		headings2 = headings2[::-1]
		path_sum = np.concatenate((path1, path2))
		heading_sum = np.concatenate((headings1, headings2))
		# for xy,h in zip(path_sum, heading_sum):
		# 	print(xy[0], xy[1], h[0])
		points_one_stage_den = [[xy[0], xy[1], h[0], "r"] for xy,h in zip(path_sum, heading_sum)]
		# print(points_one_stage_den)
		
		if show_fig:
			plot_path_with_headings(points_one_stage_anchor, path_sum)

		with open('hummer_path/%s.txt'%fn1, 'w', encoding='utf-8') as f:
			json.dump(points_one_stage_den, f, indent=2)
		


		option = "6 back shift front in"
		print(option)
		points_n2 = anchor_backward_2stage(points, le_vs_ri)
		# print(points_n2)
		if show_fig:
			show_path(points_n2, option)
		points_two_stage_anchor = points_n2

		back_list = points_n2[:-1]
		back_list = back_list[::-1]
		path1,headings1 = generate_path(back_list, heading_scale=4.0, resolution=stage_resolution)
		path2,headings2 = generate_path(points_n2[1:], heading_scale=4.0, resolution=stage_resolution)
		path1 = path1[::-1]
		headings1 = headings1[::-1]
		path_sum = np.concatenate((path1, path2))
		heading_sum = np.concatenate((headings1, headings2))
		# for xy,h in zip(path_sum, heading_sum):
		# 	print(xy[0], xy[1], h[0])
		points_two_stage_den = [[xy[0], xy[1], h[0], "r"] for xy,h in zip(path_sum, heading_sum)]
		# print(points_two_stage_den)
		for i in range(stage_resolution, stage_resolution*2):
			points_two_stage_den[i][3] = "f"
		
		# print(path_sum2)
		# points_two_stage_den = path_sum2
		if show_fig:
			plot_path_with_headings(points_two_stage_anchor, path_sum)

		with open('hummer_path/%s.txt'%fn2, 'w', encoding='utf-8') as f:
			json.dump(points_two_stage_den, f, indent=2)
		


	# head to spot, right hand
	elif (f_vs_r=="backward" and le_vs_ri=="right"):
		option = "tbd"

		option = "7 direct back in"
		print(option)
		points_n1 = anchor_backward(points, le_vs_ri)
		# print(points_n1)
		if show_fig:
			show_path(points_n1, option)
		points_one_stage_anchor = points_n1

		back_list1 = points_n1[:-1]
		back_list1 = back_list1[::-1]
		path1,headings1 = generate_path(back_list1, heading_scale=4.0, resolution=stage_resolution)
		path1 = path1[::-1]
		headings1 = headings1[::-1]
		back_list2 = points_n1[1:]
		back_list2 = back_list2[::-1]
		path2,headings2 = generate_path(back_list2, heading_scale=4.0, resolution=stage_resolution)
		path2 = path2[::-1]
		headings2 = headings2[::-1]
		path_sum = np.concatenate((path1, path2))
		heading_sum = np.concatenate((headings1, headings2))
		# print(headings1)
		# print("path", path_sum)
		# print("headings", heading_sum)
		# for xy,h in zip(path_sum, heading_sum):
		# 	print(xy[0], xy[1], h[0])
		points_one_stage_den = [[xy[0], xy[1], h[0], "r"] for xy,h in zip(path_sum, heading_sum)]
		# print(points_one_stage_den)
		
		if show_fig:
			plot_path_with_headings(points_one_stage_anchor, path_sum)
		with open('hummer_path/%s.txt'%fn1, 'w', encoding='utf-8') as f:
			json.dump(points_one_stage_den, f, indent=2)
		

		option = "8 back shift front in"
		print(option)
		points_n2 = anchor_backward_2stage(points, le_vs_ri)
		# print(points_n2)
		if show_fig:
			show_path(points_n2, option)
		points_two_stage_anchor = points_n2

		back_list = points_n2[:-1]
		back_list = back_list[::-1]
		path1,headings1 = generate_path(back_list, heading_scale=4.0, resolution=stage_resolution)
		path2,headings2 = generate_path(points_n2[1:], heading_scale=4.0, resolution=stage_resolution)
		path1 = path1[::-1]
		headings1 = headings1[::-1]
		path_sum = np.concatenate((path1, path2))
		heading_sum = np.concatenate((headings1, headings2))
		# for xy,h in zip(path_sum, heading_sum):
		# 	print(xy[0], xy[1], h[0])
		points_two_stage_den = [[xy[0], xy[1], h[0], "r"] for xy,h in zip(path_sum, heading_sum)]
		for i in range(stage_resolution, stage_resolution*2):
			points_two_stage_den[i][3] = "f"
		# print(points_two_stage_den)
		
		if show_fig:
			plot_path_with_headings(points_two_stage_anchor, path_sum)

		with open('hummer_path/%s.txt'%fn2, 'w', encoding='utf-8') as f:
			json.dump(points_two_stage_den, f, indent=2)



def reverse_path(fn): 
	with open("./hummer_path/%s.txt"%fn, 'r') as file:
		data = json.load(file)

	for i in range(len(data)):
		data[i][2] = (data[i][2]+180)%360
		if data[i][3] =='r':
			data[i][3] = 'f'
		elif data[i][3] == 'f':
			data[i][3] = 'r'

	with open("./hummer_path/%s_rev.txt"%fn, 'w') as file:
		json.dump(data, file, indent=2)




## rotation since compass is not trustful
def rotate_heading(points, theta_deg):

	theta_rad = np.radians(theta_deg)

	# Rotation matrix
	rotation_matrix = np.array([
		[np.cos(theta_rad), -np.sin(theta_rad)],
		[np.sin(theta_rad),  np.cos(theta_rad)]
	])

	# Apply rotation
	rotated_points = points @ rotation_matrix.T
	return rotated_points


def can_heading_cali_phone_loc():

	fn = "can_heading_ref"
	# f = open("./hummer_path/%s.txt"%fn, "r")
	with open("./hummer_path/%s.txt"%fn, 'r') as f:
		line = f.readline()
	can_data = json.loads(line)
	# f.close()
	can_h = float(can_data[2])

	fn = "pathx5"
	# f = open("./hummer_path/%s.txt"%fn, "r")
	with open("./hummer_path/%s.txt"%fn, 'r') as f:
		line = f.readline()
	# f.close()
	path_x5 = json.loads(line)
	
	phone_compass_h = path_x5[0][2]
	theta_deg_abs = abs(can_h-phone_compass_h)

	theta_deg_rel = rel_turning_direction(phone_compass_h, can_h)
	
	theta_deg = theta_deg_abs
	if theta_deg_rel=="counter-clockwise":
		theta_deg = -theta_deg_abs
	theta_deg = theta_deg%360
	# print(can_h, phone_compass_h, theta_deg_rel, theta_deg)


	with open("./hummer_path/phone_local.txt", "r") as f:
		line = f.readline()
	phone_loc = json.loads(line)
	# print(phone_loc)
	# f.close()
	
	xy = [[x[0], x[1]] for x in phone_loc]
	xy_rot = rotate_heading(xy, -theta_deg)

	data_rot = [[float(loc[0]), float(loc[1])] for loc in xy_rot]

	fon = "phone_local_can_heading"
	with open("./hummer_path/%s.txt"%fon, "w") as fo:
		fo.write(str(data_rot))
	# fo.close()

	# all history data
	fon = "phone_local_can_heading_all"
	with open("./hummer_path/%s.txt"%fon, "a") as file:
		file.write(str(xy_rot[0][0])+","+str(xy_rot[0][1])+"\n")

	return xy_rot[0]

def can_heading_cali_abs():
	
	theta_deg = 0

	## all 3 paths

	fn = "can_heading_ref"
	# f = open("./hummer_path/%s.txt"%fn, "r")
	with open("./hummer_path/%s.txt"%fn, "r") as f:
		line = f.readline()
	can_data = json.loads(line)
	# f.close()
	can_h = float(can_data[2])

	fn = "pathx5"
	# f = open("./hummer_path/%s.txt"%fn, "r")
	with open("./hummer_path/%s.txt"%fn, "r") as f:
		line = f.readline()
	path_x5 = json.loads(line)
	# f.close()

	phone_compass_h = path_x5[0][2]
	theta_deg_abs = abs(can_h-phone_compass_h)

	theta_deg_rel = rel_turning_direction(phone_compass_h, can_h)
	
	theta_deg = theta_deg_abs
	if theta_deg_rel=="counter-clockwise":
		theta_deg = -theta_deg_abs
	# print(can_h, phone_compass_h, theta_deg_rel, theta_deg)
	
	xy = [[x[0], x[1]] for x in path_x5]
	hs = [x[2] for x in path_x5]
	xy_rot = rotate_heading(xy, -theta_deg)
	h_rot = [(h+theta_deg)%360 for h in hs]
	path_x5_rot = [[float(loc[0]), float(loc[1]), h] for loc, h in zip(xy_rot,h_rot)]

	fon = "pathx5_can_heading"
	with open("./hummer_path/%s.txt"%fon, "w") as fo:
		fo.write(str(path_x5_rot))
	# fo.close()


	fn1 = "path_local_den_1_stage"
	with open("./hummer_path/%s.txt"%fn1, 'r') as f:
		path_xn = json.load(f)
	xy = [[x[0], x[1]] for x in path_xn]
	hs = [x[2] for x in path_xn]
	gs = [x[3] for x in path_xn]
	xy_rot = rotate_heading(xy, -theta_deg)
	h_rot = [(h+theta_deg)%360 for h in hs]
	path_xn_rot = [[float(loc[0]), float(loc[1]), h, g] for loc, h,g in zip(xy_rot,h_rot,gs)]

	fon1 = "path_local_den_1_stage_can_heading"
	with open("./hummer_path/%s.txt"%fon1, "w", encoding='utf-8') as f:
			json.dump(path_xn_rot, f, indent=2)


	fn2 = "path_local_den_2_stage"
	with open("./hummer_path/%s.txt"%fn2, 'r') as f:
		path_xn = json.load(f)
	xy = [[x[0], x[1]] for x in path_xn]
	hs = [x[2] for x in path_xn]
	gs = [x[3] for x in path_xn]
	xy_rot = rotate_heading(xy, -theta_deg)
	h_rot = [(h+theta_deg)%360 for h in hs]
	path_xn_rot = [[float(loc[0]), float(loc[1]), h, g] for loc, h,g in zip(xy_rot,h_rot,gs)]

	fon2 = "path_local_den_2_stage_can_heading"
	with open("./hummer_path/%s.txt"%fon2, "w", encoding='utf-8') as f:
			json.dump(path_xn_rot, f, indent=2)


	## floor & obs

	fn = "floor_vers"
	with open("./hummer_path/%s.txt"%fn, 'r') as f:
		ps = json.load(f)
	xy = [[x[0], -x[1]] for x in ps]
	xy_rot = rotate_heading(xy, -theta_deg)
	xy_rot = [[float(x[0]), -1*float(x[1])] for x in xy_rot]
	
	fon = "floor_vers_can_heading"
	with open("./hummer_path/%s.txt"%fon, "w") as fo:
		fo.write(str(xy_rot))
	# fo.close()

	fn = "obs"
	with open("./hummer_path/%s.txt"%fn, 'r') as f:
		ps = json.load(f)
	obs_rot = []
	if len(ps)>0:
		xy = [[x[0], -x[1]] for x in ps]
		zs = [x[2] for x in ps]
		xy_rot = rotate_heading(xy, -theta_deg)
		xy_rot = [[x[0], -x[1]] for x in xy_rot]
		obs_rot = [[float(loc[0]), float(loc[1]), z] for loc,z in zip(xy_rot,zs)]
		
	fon = "obs_can_heading"
	with open("./hummer_path/%s.txt"%fon, "w") as fo:
		fo.write(str(obs_rot))
	# fo.close()


def rotate_path(path, old_heading, new_heading, pivot=(0,0)):
	# how much to rotate (compass clockwise)
	delta_deg = (new_heading - old_heading) % 360
	delta_rad = math.radians(delta_deg)  # clockwise, since compass system matches
	
	cos_t = math.cos(delta_rad)
	sin_t = math.sin(delta_rad)
	
	px, py = pivot
	rotated = []
	for (x, y, heading) in path:
		# shift relative to pivot
		dx, dy = x - px, y - py
		
		# rotate around pivot (clockwise compass)
		new_x = dx * cos_t + dy * sin_t + px
		new_y = -dx * sin_t + dy * cos_t + py
		
		# adjust heading (compass)
		new_h = (heading + delta_deg) % 360
		
		rotated.append([new_x, new_y, new_h])
	
	return rotated


def rotate_path_pv(path, old_heading, new_heading, pivot=(0, 0)):
    # how much to rotate (compass clockwise)
    delta_deg = (new_heading - old_heading) % 360
    delta_rad = math.radians(delta_deg)  # clockwise, since compass system matches
    
    cos_t = math.cos(delta_rad)
    sin_t = math.sin(delta_rad)
    
    px, py = pivot
    rotated = []
    for (x, y, heading) in path:
        # 1. Shift relative to pivot (x0, y0)
        dx, dy = x - px, y - py
        
        # 2. Rotate around pivot (clockwise compass)
        new_x = dx * cos_t + dy * sin_t + px
        new_y = -dx * sin_t + dy * cos_t + py
        
        # 3. Adjust heading (compass)
        new_h = (heading + delta_deg) % 360
        
        rotated.append([new_x, new_y, new_h])
    
    return rotated




def rotate_path_multi(path, old_headings, new_headings, pivot=(0,0)):
	# Calculate the average difference between the headings
	# We use a special function to handle 360-degree wrapping
	total_diff = 0
	for i in range(len(old_headings)):
		# Calculate the shortest angle difference, handling wrap-around
		diff = new_headings[i] - old_headings[i]
		if diff > 180:
			diff -= 360
		elif diff < -180:
			diff += 360
		total_diff += diff
	
	delta_deg = total_diff / len(old_headings)
	delta_rad = math.radians(delta_deg)
	
	cos_t = math.cos(delta_rad)
	sin_t = math.sin(delta_rad)
	
	px, py = pivot
	rotated = []
	
	for (x, y, heading) in path:
		# Shift relative to pivot
		dx, dy = x - px, y - py
		
		# Rotate around pivot
		new_x = dx * cos_t - dy * sin_t + px
		new_y = dx * sin_t + dy * cos_t + py
		
		# Adjust heading
		new_h = (heading + delta_deg) % 360
		
		rotated.append([new_x, new_y, new_h])
		
	return rotated



def can_heading_cali():

	## all 3 paths
	
	fn = "can_heading_ref"
	with open("./hummer_path/%s.txt"%fn, "r") as f:
		line = f.readline()
	can_data = json.loads(line)
	# f.close()
	can_h = float(can_data[2])

	fn = "pathx5"
	with open("./hummer_path/%s.txt"%fn, "r") as f:
		line = f.readline()
	path_x5 = json.loads(line)
	# f.close()


	phone_compass_h = path_x5[0][2]

	## use vehicle heading
	path_x5_rot = rotate_path(path_x5, old_heading=phone_compass_h, new_heading=can_h)

	# print("deb",str(path_x5_rot))

	# fon = "pathx5_can_heading"
	# with open("./hummer_path/%s.txt"%fon, "w", encoding='utf-8') as f:
		# json.dump(path_x5_rot, f)
	# json.dump(path_xn_rot_gear, f, indent=2)
	# f = open("./hummer_path/%s.txt"%fon, "w")
		# f.write(str(path_x5_rot))
	# f.close()
	# print("transformed path anchors", path_x5_rot)

	fon1 = "pathx5_can_heading"
	with open("./hummer_path/%s.txt"%fon1, "w") as fo1:
		fo1.write(str(path_x5_rot))
	# fo1.close()
	


	#######
	## use both target and start heading for calibration
	phone_compass_h_target = path_x5[3][2]
	fn = "spot_heading_ref"
	with open("./hummer_path/%s.txt"%fn, "r") as f:
		line = f.readline()
	can_data_target = json.loads(line)
	# f.close()
	can_h_target = float(can_data_target[2])

	path_x5_rot2 = rotate_path(path_x5, old_heading=phone_compass_h_target, new_heading=can_h_target)
	fon = "pathx5_can_heading2"
	with open("./hummer_path/%s.txt"%fon, "w") as fo:
		fo.write(str(path_x5_rot2))
	# fo.close()

	'''
	path_x5_rot3 = rotate_path_multi(path_x5, old_headings=[phone_compass_h,phone_compass_h_target], new_headings=[can_h,can_h_target])
	fon = "pathx5_can_heading3"
	fo = open("./hummer_path/%s.txt"%fon, "w")
	fo.write(str(path_x5_rot3))
	fo.close()
	'''
	#######

	
	# print(path_x5[0], path_x5[3])
	# print(path_x5_rot[0], path_x5_rot[3])
	# exit()

	fn1 = "path_local_den_1_stage"
	with open("./hummer_path/%s.txt"%fn1, 'r') as f:
		path_xn = json.load(f)
	xyh = [[x[0], x[1], x[2]] for x in path_xn]
	gs = [x[3] for x in path_xn]
	path_xn_rot = rotate_path(xyh, old_heading=phone_compass_h, new_heading=can_h)
	path_xn_rot_gear = [[float(loc[0]), float(loc[1]), float(loc[2]), g] for loc, g in zip(path_xn_rot,gs)]

	fon1 = "path_local_den_1_stage_can_heading"
	with open("./hummer_path/%s.txt"%fon1, "w", encoding='utf-8') as f:
			json.dump(path_xn_rot_gear, f, indent=2)

	fn2 = "path_local_den_2_stage"
	with open("./hummer_path/%s.txt"%fn2, 'r') as f:
		path_xn = json.load(f)
	xyh = [[x[0], x[1], x[2]] for x in path_xn]
	gs = [x[3] for x in path_xn]
	path_xn_rot = rotate_path(xyh, old_heading=phone_compass_h, new_heading=can_h)
	path_xn_rot_gear = [[float(loc[0]), float(loc[1]), float(loc[2]), g] for loc, g in zip(path_xn_rot,gs)]

	fon2 = "path_local_den_2_stage_can_heading"
	with open("./hummer_path/%s.txt"%fon2, "w", encoding='utf-8') as f:
			json.dump(path_xn_rot_gear, f, indent=2)


	## floor & obs

	fn = "floor_vers"
	with open("./hummer_path/%s.txt"%fn, 'r') as f:
		ps = json.load(f)
	
	# xyh = [[x[0], -x[1], 0] for x in ps]
	xyh = [[x[0], x[1], 0] for x in ps]
	xy_rot_h = rotate_path(xyh, old_heading=phone_compass_h, new_heading=can_h)
	xy_rot = [[float(x[0]), float(x[1])] for x in xy_rot_h]
	
	
	fon = "floor_vers_can_heading"
	with open("./hummer_path/%s.txt"%fon, "w") as fo:
		fo.write(str(xy_rot))
	# fo.close()

	fn = "obs"
	with open("./hummer_path/%s.txt"%fn, 'r') as f:
		ps = json.load(f)
	obs_rot = []
	# print("ddbug",ps)
	if len(ps)>0:
		# xyh = [[x[0], -x[1], 0] for x in ps]
		xyh = [[x[0], x[1], 0] for x in ps]
		zs = [x[2] for x in ps]
		xy_rot_h = rotate_path(xyh, old_heading=phone_compass_h, new_heading=can_h)
		xy_rot = [[float(x[0]), float(x[1])] for x in xy_rot_h]
		obs_rot = [[float(loc[0]), float(loc[1]), z] for loc,z in zip(xy_rot,zs)]

	# print("ddbug",obs_rot)
	fon = "obs_can_heading"
	with open("./hummer_path/%s.txt"%fon, "w") as fo:
		fo.write(str(obs_rot))
	# fo.close()

	## all mesh point
	# fn = "allP"
	# with open("./hummer_path/%s.txt"%fn, 'r') as f:
	# 	ps = json.load(f)
	# p_rot = []
	# if len(ps)>0:
	# 	# xyh = [[x[0], -x[1], 0] for x in ps]
	# 	xyh = [[x[0], x[1], 0] for x in ps]
	# 	zs = [x[2] for x in ps]
	# 	xy_rot_h = rotate_path(xyh, old_heading=phone_compass_h, new_heading=can_h)
	# 	xy_rot = [[float(x[0]), float(x[1])] for x in xy_rot_h]
	# 	p_rot = [[float(loc[0]), float(loc[1]), z] for loc,z in zip(xy_rot,zs)]

	# fon = "allP_can_heading"
	# with open("./hummer_path/%s.txt"%fon, "w") as fo:
	# 	fo.write(str(p_rot))
	



def can_heading_cali_J(h0):
	
	fn = "can_heading_ref"
	with open("./hummer_path/%s.txt"%fn, "r") as f:
		line = f.readline()
	can_data = json.loads(line)
	# f.close()
	can_h = float(can_data[2])
	can_h += 1 # 0826 # test and calib on rotation (can_h diff with rtk_h readings)

	fn = "pathx5"
	with open("./hummer_path/%s.txt"%fn, "r") as f:
		line = f.readline()
	path_x5 = json.loads(line)
	# f.close()


	phone_compass_h = path_x5[0][2]

	## use vehicle heading
	####### all paths
	path_x5_rot = rotate_path(path_x5, old_heading=0.0, new_heading=can_h)

	fon1 = "pathx5_can_heading"
	with open("./hummer_path/%s.txt"%fon1, "w") as fo1:
		fo1.write(str(path_x5_rot))
	# fo1.close()
	
	## use both target and start heading for calibration
	phone_compass_h_target = path_x5[3][2]
	fn = "spot_heading_ref"
	with open("./hummer_path/%s.txt"%fn, "r") as f:
		line = f.readline()
	can_data_target = json.loads(line)
	# f.close()
	can_h_target = float(can_data_target[2])

	path_x5_rot2 = rotate_path(path_x5, old_heading=0.0, new_heading=can_h_target)
	fon = "pathx5_can_heading2"
	with open("./hummer_path/%s.txt"%fon, "w") as fo:
		fo.write(str(path_x5_rot2))

	
	fn1 = "path_local_den_1_stage"
	with open("./hummer_path/%s.txt"%fn1, 'r') as f:
		path_xn = json.load(f)
	xyh = [[x[0], x[1], x[2]] for x in path_xn]
	gs = [x[3] for x in path_xn]
	path_xn_rot = rotate_path(xyh, old_heading=0.0, new_heading=can_h)
	path_xn_rot_gear = [[float(loc[0]), float(loc[1]), float(loc[2]), g] for loc, g in zip(path_xn_rot,gs)]


	fon1 = "path_local_den_1_stage_can_heading"
	with open("./hummer_path/%s.txt"%fon1, "w", encoding='utf-8') as f:
			json.dump(path_xn_rot_gear, f, indent=2)

	fn2 = "path_local_den_2_stage"
	with open("./hummer_path/%s.txt"%fn2, 'r') as f:
		path_xn = json.load(f)
	xyh = [[x[0], x[1], x[2]] for x in path_xn]
	gs = [x[3] for x in path_xn]
	path_xn_rot = rotate_path(xyh, old_heading=0.0, new_heading=can_h)
	path_xn_rot_gear = [[float(loc[0]), float(loc[1]), float(loc[2]), g] for loc, g in zip(path_xn_rot,gs)]

	fon2 = "path_local_den_2_stage_can_heading"
	with open("./hummer_path/%s.txt"%fon2, "w", encoding='utf-8') as f:
			json.dump(path_xn_rot_gear, f, indent=2)


	## floor & obs
	fn = "floor_vers"
	with open("./hummer_path/%s.txt"%fn, 'r') as f:
		ps = json.load(f)
	
	# xyh = [[x[0], -x[1], 0] for x in ps]
	xyh = [[x[0], x[1], 0] for x in ps]
	xy_rot_h = rotate_path(xyh, old_heading=h0, new_heading=can_h)
	xy_rot = [[float(x[0]), float(x[1])] for x in xy_rot_h]
	
	fon = "floor_vers_can_heading"
	with open("./hummer_path/%s.txt"%fon, "w") as fo:
		fo.write(str(xy_rot))

	fn = "obs"
	with open("./hummer_path/%s.txt"%fn, 'r') as f:
		ps = json.load(f)
	obs_rot = []
	# print("ddbug",ps)
	if len(ps)>0:
		# xyh = [[x[0], -x[1], 0] for x in ps]
		xyh = [[x[0], x[1], 0] for x in ps]
		zs = [x[2] for x in ps]
		xy_rot_h = rotate_path(xyh, old_heading=h0, new_heading=can_h)
		xy_rot = [[float(x[0]), float(x[1])] for x in xy_rot_h]
		obs_rot = [[float(loc[0]), float(loc[1]), z] for loc,z in zip(xy_rot,zs)]

	# print("ddbug",obs_rot)
	fon = "obs_can_heading"
	with open("./hummer_path/%s.txt"%fon, "w") as fo:
		fo.write(str(obs_rot))






def can_heading_cali_trailer(delta_deg):

	## all 3 paths
	
	fn = "can_heading_ref"
	with open("./hummer_path/%s.txt"%fn, "r") as f:
		line = f.readline()
	can_data = json.loads(line)
	# f.close()
	can_h = float(can_data[2])

	fn = "pathx5"
	with open("./hummer_path/%s.txt"%fn, "r") as f:
		line = f.readline()
	path_x5 = json.loads(line)
	# f.close()


	phone_compass_h = path_x5[0][2] #+delta_deg

	## use trailer heading
	pivot = (path_x5[0][0], path_x5[0][1])
	# pivot = (0,0)
	path_x5_rot_mid = rotate_path(path_x5, old_heading=phone_compass_h, new_heading=can_h)

	path_x5_rot = rotate_path_pv(path_x5_rot_mid, old_heading=delta_deg, new_heading=0, pivot=pivot)
	# path_x5_rot = path_x5_rot_mid
	print("deb",str(path_x5_rot))

	# fon = "pathx5_can_heading"
	# with open("./hummer_path/%s.txt"%fon, "w", encoding='utf-8') as f:
		# json.dump(path_x5_rot, f)
	# json.dump(path_xn_rot_gear, f, indent=2)
	# f = open("./hummer_path/%s.txt"%fon, "w")
		# f.write(str(path_x5_rot))
	# f.close()
	# print("transformed path anchors", path_x5_rot)

	fon1 = "pathx5_can_heading"
	with open("./hummer_path/%s.txt"%fon1, "w") as fo1:
		fo1.write(str(path_x5_rot))
	# fo1.close()
	


	#######
	## use both target and start heading for calibration
	phone_compass_h_target = path_x5[3][2]
	fn = "spot_heading_ref"
	with open("./hummer_path/%s.txt"%fn, "r") as f:
		line = f.readline()
	can_data_target = json.loads(line)
	# f.close()
	can_h_target = float(can_data_target[2])

	path_x5_rot2 = rotate_path(path_x5, old_heading=phone_compass_h_target, new_heading=can_h_target)
	fon = "pathx5_can_heading2"
	with open("./hummer_path/%s.txt"%fon, "w") as fo:
		fo.write(str(path_x5_rot2))
	# fo.close()

	# print(path_x5[0], path_x5[3])
	# print(path_x5_rot[0], path_x5_rot[3])
	# exit()

	fn1 = "path_local_den_1_stage"
	with open("./hummer_path/%s.txt"%fn1, 'r') as f:
		path_xn = json.load(f)
	xyh = [[x[0], x[1], x[2]] for x in path_xn]
	gs = [x[3] for x in path_xn]
	path_xn_rot_mid = rotate_path(xyh, old_heading=phone_compass_h, new_heading=can_h)
	path_xn_rot = rotate_path_pv(path_xn_rot_mid, old_heading=delta_deg, new_heading=0, pivot=pivot)

	path_xn_rot_gear = [[float(loc[0]), float(loc[1]), float(loc[2]), g] for loc, g in zip(path_xn_rot,gs)]

	fon1 = "path_local_den_1_stage_can_heading"
	with open("./hummer_path/%s.txt"%fon1, "w", encoding='utf-8') as f:
			json.dump(path_xn_rot_gear, f, indent=2)

	fn2 = "path_local_den_2_stage"
	with open("./hummer_path/%s.txt"%fn2, 'r') as f:
		path_xn = json.load(f)
	xyh = [[x[0], x[1], x[2]] for x in path_xn]
	gs = [x[3] for x in path_xn]
	path_xn_rot = rotate_path(xyh, old_heading=phone_compass_h, new_heading=can_h)
	path_xn_rot_gear = [[float(loc[0]), float(loc[1]), float(loc[2]), g] for loc, g in zip(path_xn_rot,gs)]

	fon2 = "path_local_den_2_stage_can_heading"
	with open("./hummer_path/%s.txt"%fon2, "w", encoding='utf-8') as f:
			json.dump(path_xn_rot_gear, f, indent=2)


	## floor & obs

	fn = "floor_vers"
	with open("./hummer_path/%s.txt"%fn, 'r') as f:
		ps = json.load(f)
	
	# xyh = [[x[0], -x[1], 0] for x in ps]
	xyh = [[x[0], x[1], 0] for x in ps]
	xy_rot_h = rotate_path(xyh, old_heading=phone_compass_h, new_heading=can_h)
	xy_rot = [[float(x[0]), float(x[1])] for x in xy_rot_h]
	
	
	fon = "floor_vers_can_heading"
	with open("./hummer_path/%s.txt"%fon, "w") as fo:
		fo.write(str(xy_rot))
	# fo.close()

	fn = "obs"
	with open("./hummer_path/%s.txt"%fn, 'r') as f:
		ps = json.load(f)
	obs_rot = []
	# print("ddbug",ps)
	if len(ps)>0:
		# xyh = [[x[0], -x[1], 0] for x in ps]
		xyh = [[x[0], x[1], 0] for x in ps]
		zs = [x[2] for x in ps]
		xy_rot_h = rotate_path(xyh, old_heading=phone_compass_h, new_heading=can_h)
		xy_rot = [[float(x[0]), float(x[1])] for x in xy_rot_h]
		obs_rot = [[float(loc[0]), float(loc[1]), z] for loc,z in zip(xy_rot,zs)]

	# print("ddbug",obs_rot)
	fon = "obs_can_heading"
	with open("./hummer_path/%s.txt"%fon, "w") as fo:
		fo.write(str(obs_rot))
	# fo.close()

	## all mesh point
	# fn = "allP"
	# with open("./hummer_path/%s.txt"%fn, 'r') as f:
	# 	ps = json.load(f)
	# p_rot = []
	# if len(ps)>0:
	# 	# xyh = [[x[0], -x[1], 0] for x in ps]
	# 	xyh = [[x[0], x[1], 0] for x in ps]
	# 	zs = [x[2] for x in ps]
	# 	xy_rot_h = rotate_path(xyh, old_heading=phone_compass_h, new_heading=can_h)
	# 	xy_rot = [[float(x[0]), float(x[1])] for x in xy_rot_h]
	# 	p_rot = [[float(loc[0]), float(loc[1]), z] for loc,z in zip(xy_rot,zs)]

	# fon = "allP_can_heading"
	# with open("./hummer_path/%s.txt"%fon, "w") as fo:
	# 	fo.write(str(p_rot))
	


################ 

app = Flask(__name__)





@app.route('/')
def home():
#	return render_template('index.html')
	return render_template('index3D.html')

@app.route('/test')
def homet():
#	return render_template('index.html')
	return render_template('test.html')


@app.route('/path.json')
def path_data():

	## load from the processed smooth path
	f = open("hummer_path/pathx5_smooth_gps.csv", "r")
	h = f.readline()
	data = []
	for line in f:
		eles = line.split(",")
		data.append([float(eles[0]), float(eles[1])])
	f.close()
	return jsonify(data)

	# print("smo path", data)
	# exit()

@app.route('/path_ll.json')
def path_ll_data():

	## load from the processed smooth path
	with open("hummer_path/pathx5.txt", "r") as f:
		line = f.readline()
	path_loc = json.loads(line)
	# f.close()

	
	return jsonify(path_loc)

	# print("path ll", data)
	# exit()

@app.route('/obs_ll.json')
def obs_ll_data():

	## load from the processed smooth path
	with open("hummer_path/obs.txt", "r") as f:
		line = f.readline()
	path_loc = json.loads(line)
	# f.close()
	return jsonify(path_loc)

	# print("obs ll", data)
	# exit()


@app.route('/cmd_start',  methods=['POST'])
def cmd_start():
	# data = request.get_json()
	# fn = "execution_tag"
	# with open("./hummer_path/%s.txt"%fn, 'w') as f:
	# 	f.write(str("1"))
	# return jsonify({'status': 'success', 'echo': 1}), 200

	fn = "execution_tag"
	with open("./hummer_path/%s.txt"%fn, 'r') as f:
		cur_state = f.readline()
	
	tag = "success"

	if cur_state=="1":
		tag = "already start"
		return jsonify({'status': 'success', 'echo': 1}), 200


	else:
		fn = "execution_tag"
		with open("./hummer_path/%s.txt"%fn, 'w') as f:
			f.write(str("1"))

		## auto start execution
		try:
			os.chdir("../../ConnAu/ros_ws")
			
			launch_process(["ros2", "run", "fusion_py", "VehicleCommander"])

			print("\n starting VehicleCommander.")
			

		except:
			print("\n starting VehicleCommander fail.")
			tag = 'fail'
			

		finally:
			os.chdir("../../srp/birdview")
			return jsonify({'status': tag, 'echo': "start"}), 200



@app.route('/cmd_stop',  methods=['POST'])
def cmd_stop():
	# data = request.get_json()
	# fn = "execution_tag"
	# with open("./hummer_path/%s.txt"%fn, 'w') as f:
	# 	f.write(str("0"))
	# return jsonify({'status': 'success', 'echo': 0}), 200

	fn = "execution_tag"
	with open("./hummer_path/%s.txt"%fn, 'w') as f:
		f.write(str("0"))

	## auto stop
	cleanup()

	return jsonify({'status': 'success', 'echo': 0}), 200




@app.route('/path_upload',  methods=['POST'])
def path_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	with open("path.txt", "w") as f:
		f.write(message)
	# f.close()
	# print(f"Received message: {message}")

	return jsonify({'status': 'success', 'echo': message}), 200




@app.route('/init_vr_para',  methods=['POST'])
def para_init_vr():

	data = request.get_json()
	print("init vr data", data)

	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	init_para = {
		"Ra": 0.82, # car_UWB_tag_dist_half
		"L_sv": 1.23, # trailer_uwb_to_hinge
		"Lt": 3.0, # veh_cen_to_hinge
		"L_ht": 0.37, # car_uwb_mid_to_hinge
		"yh": -2.9, # car_cen_to_hinge
		"Trailer_L": 5.57/2, # trailer_len # tbd
		"Trailer_W": 2.59, # trailer_width_wheel # tbd
		"d1": 1.8, # uwb_init_d1
		"d2": 1.7, # uwb_init_d2
		"wheelbase": 0.4 # tbd
		## may also need uwb anchor positon (define in which coordinate)
	}

	if "Ra" in message:
		init_para["Ra"] = message["Ra"]

	if "L_sv" in message:
		init_para["L_sv"] = message["L_sv"]

	if "Lt" in message:
		init_para["Lt"] = message["Lt"]

	if "L_ht" in message:
		init_para["L_ht"] = message["L_ht"]

	if "yh" in message:
		init_para["yh"] = message["yh"]

	if "Trailer_L" in message:
		init_para["Trailer_L"] = message["Trailer_L"]

	if "Trailer_W" in message:
		init_para["Trailer_W"] = message["Trailer_W"]

	if "d1" in message:
		init_para["d1"] = message["d1"]

	if "d2" in message:
		init_para["d2"] = message["d2"]


	with open("hummer_path/trailer_init.json", "w") as f:
		json.dump(init_para, f)


	return jsonify({'status': 'success', 'echo': message}), 200




@app.route('/path_upload_x5',  methods=['POST'])
def path_upload_x5():

	
	## need to be done first before calibrate with this can heading
	# set gps reference point when new path generated
	# os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p.txt") #?? why p works
	
	## (option 1) this use the current gps and heading 
	os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p2.txt")
	os.system("cp ./hummer_path/can_heading.txt ./hummer_path/can_heading_ref.txt")

	'''
	## try use the 1st gps and the filtered gps
	## (option 2) use initial gps
	os.system("cp ./hummer_path/gps_ref_p1.txt ./hummer_path/gps_ref_p2.txt")
	
	## (option 3) use avg
	file_name = './hummer_path/gps_history_before_start.txt'
	avg_x, avg_y, total_lines = get_average_coordinates(file_name)
	with open("./hummer_path/gps_ref_p2.txt", 'w') as f:
		f.write("[%f,%f]"%(float(avg_x), float(avg_y)))
	'''


	# add 4 degree to calibrate can heading
	# f = open("./hummer_path/can_heading.txt", "r")
	# fo = open("./hummer_path/can_heading_ref.txt", "w")
	# data = json.loads(f.readline())
	# data[2] = str(float(data[2])+5)
	# fo.write(message)
	# f.close()
	# fo.close()


	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	# print("--", np.array(message))
	### estimate turning point p2 based on current p1,p3,p4
	# p2 = find_sta_turn_p_bri(np.array(message))

	# message[1] = p2
	
	with open("hummer_path/pathx5.txt", "w") as f:
		f.write(message)
	# f.close()
	print(f"---- Received ori path x5: {message}")

	fn = "pathx5"
	## change both pathx5 and pathx5_can_heading if do it offline
	with open("hummer_path/%s.txt"%fn, "r") as f:
		message = json.loads(f.readline())
	# f.close()
	# print("tt",message)

	p2 = find_sta_turn_p_bri(np.array(message))
	message[1] = p2

	# print("tt", p2)
	# print(message, "d")
	with open('hummer_path/%s.txt'%fn, 'w') as f:
		json.dump(message, f)


	## convert to smooth path
	# realtime data
	'''
	f = open("hummer_path/pathx5.txt", "r")
	data = f.readline()
	f.close()
		
	path_gps_with_heading = json.loads(data)
	# print(path_gps_with_heading)

	## adjust to testing parking spot location
	# target_latlon = [42.52033598879717, -83.04338551227549]
	# target_latlon = [42.5203530204544, -83.0433692649389] ## calibrate to where the spot is
	# target_latlon = [path_gps_with_heading[4][0], path_gps_with_heading[4][1]]
	# offset_lat = target_latlon[0]-path_gps_with_heading[4][0]
	# offset_lon = target_latlon[1]-path_gps_with_heading[4][1]
	offset_lat = 0
	offset_lon = 0

	for i in range(5):
		path_gps_with_heading[i][0] += offset_lat
		path_gps_with_heading[i][1] += offset_lon
		path_gps_with_heading[i][2] = path_gps_with_heading[i][2]/180*3.1415926
	pts_in_rad = np.array(path_gps_with_heading)
	# print("path cali gps key", pts_in_rad)

	tot_point = 100
	lat_s, lon_s, h_s, x_s, y_s = path_gen_for_eh(pts_in_rad, tot_point)

	offset = 5
	sel = int(tot_point*3/4 + offset)
	lat_s = lat_s[:sel]
	lon_s = lon_s[:sel]
	h_s = h_s[:sel]
	x_s = x_s[:sel]
	y_s = y_s[:sel]

	## write offline path file
	llh = [[a,b,c] for a,b,c in zip(lat_s, lon_s, h_s)]
	f = open("hummer_path/pathx5_smooth.csv","w")
	f.write("latitude,longitude,heading\n")
	for ele in llh:
		# print(ele[0],", ",ele[1])
		f.write("%f,%f,%f\n"%(ele[0],ele[1],ele[2]))
	f.close()
	'''

	'''
	## 2.4m offset calibration to use antenna mounting location as veh center
	ant_offset = 2.4
	path_antenna_cali(ant_offset)
	'''

	## generate path using 1st (init) and 4th (target) point only with x2 options
	## also include path interpulation
	fn = "pathx5"
	# fn = "pathx5_mod"
	# fn = "pathx5_10_6_test"
	fn1 = "path_local_den_1_stage"
	fn2 = "path_local_den_2_stage"

	stage_resolution = 100

	## path planning (x8 scenario) + path interpulation
	# rel_loc_ana_and_path_inter(fn, stage_resolution, fn1, fn2)
	rel_loc_ana_and_path_inter_simple(fn, stage_resolution, fn1, fn2, "forward")
	
	## adjust pathx3, floor, obs, start,end postion & orientation by can sensed heading
	can_heading_cali()
	# can_heading_cali_abs()




	## add 2nd trajectory for moving car out of parking spot (reverse)
	fin = "pathx5_can_heading"
	with open("./hummer_path/%s.txt"%fin, "r") as file:
		path_x5 = json.load(file)

	print(path_x5, "--=-")

	
	p_s = path_x5[3]
	p_e = path_x5[2]
	reverse_pointx5 = [p_s, p_s, p_s, p_e, p_e]	

	# print("reverse point", reverse_pointx5)
	fon = "pathx5_reverse_test"
	with open("./hummer_path/%s.txt"%fon, "w") as file:
		json.dump(reverse_pointx5, file)


	## clear history phone location data
	fon = "phone_local_can_heading_all"
	with open("./hummer_path/%s.txt"%fon, "w") as file:
		file.write("")



	## estra add by 0306 to calibrate the heading (turn left 3 degree), might due to the slop for demo
	cali_fin_path = rotate_by_0(3)
	with open('hummer_path/path_x5_can_heading.txt', 'w') as f:
		json.dump(cali_fin_path, f)


	return jsonify({'status': 'success', 'echo': message}), 200





@app.route('/path_upload_x5_rev',  methods=['POST'])
def path_upload_x5_rev():
	
	## need to be done first before calibrate with this can heading
	# set gps reference point when new path generated
	# os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p.txt") #?? why p works


	## (option 1) this use the current gps and heading 
	os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p2.txt")
	os.system("cp ./hummer_path/can_heading.txt ./hummer_path/can_heading_ref.txt")


	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	# print("--", np.array(message))
	### estimate turning point p2 based on current p1,p3,p4
	# p2 = find_sta_turn_p_bri(np.array(message))

	# message[1] = p2
	
	# f = open("hummer_path/pathx5.txt", "w")
	# f.write(message)
	# f.close()
	print(f"---- Received ori path x5: {message}")

	with open("hummer_path/pathx5_rev.txt", "w") as f:
		f.write(message)
	# f.close()

	###### Justin 1024 reverse headings of path replace x5 ######
	## change both pathx5 and pathx5_can_heading if do it offline
	with open("hummer_path/pathx5_rev.txt", "r") as f:
		message = json.loads(f.readline())
	
	for i in range(len(message)):
		message[i][-1] = (message[i][-1]+180)%360
	print(f"---- flip path: {message}")

	fn = "pathx5"
	with open('hummer_path/%s.txt'%fn, 'w') as f:
		json.dump(message, f)

	############################################################
	## change both pathx5 and pathx5_can_heading if do it offline
	with open("hummer_path/%s.txt"%fn, "r") as f:
		message = json.loads(f.readline())
	# f.close()
	# print("tt",message)

	p2 = find_sta_turn_p_bri(np.array(message))
	message[1] = p2

	print("update 2nd anchor:", p2)
	# print(message, "d")
	with open('hummer_path/%s.txt'%fn, 'w') as f:
		json.dump(message, f)

	## generate path using 1st (init) and 4th (target) point only with x2 options
	## also include path interpulation
	fn = "pathx5"
	fn1 = "path_local_den_1_stage"
	fn2 = "path_local_den_2_stage"

	stage_resolution = 100

	## path planning (x8 scenario) + path interpulation
	# rel_loc_ana_and_path_inter(fn, stage_resolution, fn1, fn2)
	rel_loc_ana_and_path_inter_simple(fn, stage_resolution, fn1, fn2, "forward")
	
	## adjust pathx3, floor, obs, start,end postion & orientation by can sensed heading
	can_heading_cali()

	## flip to reverse gear
	fn = "pathx5_can_heading"
	with open("./hummer_path/%s.txt"%fn, 'r') as file:
		data = json.load(file)

	for i in range(len(data)):
		data[i][2] = (data[i][2]+180)%360

	with open("./hummer_path/%s_rev.txt"%fn, 'w') as file:
		json.dump(data, file)



	## just reverse the pathx5_can_heading interpolated data (den1/den2)
	fin = "path_local_den_1_stage_can_heading"
	reverse_path(fin)


	fin2 = "path_local_den_2_stage_can_heading"
	reverse_path(fin2)


	## ********* check why flip x/y Justin 1114
	datar = []
	for d in data:
		datar.append([-d[0], -d[1], d[2]])
	fn = "pathx5_can_heading"
	with open("./hummer_path/%s.txt"%fn, 'w') as file:
		json.dump(datar, file)

	fn = "floor_vers_can_heading"
	with open("./hummer_path/%s.txt"%fn, 'r') as file:
		dataf = json.load(file)
	datafr = []
	for d in dataf:
		datafr.append([-d[0], -d[1]])
	
	with open("./hummer_path/%s.txt"%fn, 'w') as file:
		json.dump(datafr, file)

	fn = "obs_can_heading"
	with open("./hummer_path/%s.txt"%fn, 'r') as file:
		datao = json.load(file)
	dataor = []
	for d in datao:
		dataor.append([-d[0], -d[1], d[2]])
	
	with open("./hummer_path/%s.txt"%fn, 'w') as file:
		json.dump(dataor, file)

	fn = "path_local_den_1_stage_can_heading"
	with open("./hummer_path/%s.txt"%fn, 'r') as file:
		data_inter = json.load(file)
	data_inter_r = []
	for d in data_inter:
		data_inter_r.append([-d[0], -d[1], d[2], 'r'])
	
	with open("./hummer_path/%s.txt"%fn, 'w', encoding='utf-8') as file:
		json.dump(data_inter_r, file, indent=2)


	## reverse 180 back for path_x5_can_heading (1118, update for johnson to gen trajectory)
	fn = "pathx5_can_heading"
	with open("./hummer_path/%s.txt"%fn, 'r') as file:
		data = json.load(file)

	for i in range(len(data)):
		data[i][2] = (data[i][2]+180)%360

	with open("./hummer_path/%s.txt"%fn, 'w') as file:
		json.dump(data, file)



	## add 2nd trajectory for moving car out of parking spot (reverse)
	fin = "pathx5_can_heading"
	with open("./hummer_path/%s.txt"%fin, "r") as file:
		path_x5 = json.load(file)

	p_s = path_x5[3]
	p_e = path_x5[2]
	reverse_pointx5 = [p_s, p_s, p_s, p_e, p_e]	

	# print("reverse point", reverse_pointx5)
	fon = "pathx5_reverse_test"
	with open("./hummer_path/%s.txt"%fon, "w") as file:
		json.dump(reverse_pointx5, file)


	p_s_r = [p_s[0], p_s[1], -p_s[2]]
	p_e_r = [p_e[0], p_e[1], -p_e[2]]
	reverse_pointx5_2 = [p_s_r, p_s_r, p_s_r, p_e_r, p_e_r]	

	fon = "pathx5_reverse_test_rev"
	with open("./hummer_path/%s.txt"%fon, "w") as file:
		json.dump(reverse_pointx5_2, file)


	## clear history phone location data
	fon = "phone_local_can_heading_all"
	with open("./hummer_path/%s.txt"%fon, "w") as file:
		file.write("")

	return jsonify({'status': 'success', 'echo': message}), 200




@app.route('/path_upload_x5_trailer_old',  methods=['POST'])
def path_upload_x5_trailer_old():

	# ## (option 1) this use the current gps and heading 
	os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p2.txt")
	os.system("cp ./hummer_path/can_heading.txt ./hummer_path/can_heading_ref.txt")

	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	message = json.loads(message)

	print(f"---- J Received ori path x5 for trailer: {message}")


	## transform to vehicle heading first

	# demo_path = [[0.0, 0.0, 31.8], [-2.8, -3.40, 263], [-7.0, -11.0, 256], [-3.1, -8.70, 294], [12.0, -6.0, 256]]
	transformed, h0 = transform_to_local_frame_by_p0_J_coor(message)

	# with open("./hummer_path/pathx5.txt"%fn, 'w', encoding='utf-8') as file:
	# 	json.dump(message, file)


	message = transformed
	print(f"---- J vehicle heading path x5 for trailer: {message}")


	# unified function estimated theta from vehicleCommanderSRP node
	
	d1 = 0.0
	d2 = 0.0
	theta_rad = 0.0
	tx = 0.0
	ty = 0.0

	pre = "hummer_path/"
	fn1 = 'uwb_recent_d1.txt'
	fn2 = 'uwb_recent_d2.txt'
	with open("%s%s"%(pre,fn1), "r") as f:
		data = json.loads(f.readline())
		d1 = data[1]
	
	with open("%s%s"%(pre,fn2), "r") as f:
		data = json.loads(f.readline())
		d2 = data[1]

	f_t = "./hummer_path/uwb_recent_theta.txt"
	with open(f_t, "r") as f:
		uwb_est = json.loads(f.readline())
		theta_rad = uwb_est[1]
		tx = uwb_est[2]
		ty = uwb_est[3]


	print("--J est. unify uwb decawave (trailer2veh)--", d1, d2, theta_rad, tx, ty)
	
	veh_h = message[0][2]

	def global_to_local_rotate(car_heading_degrees, relative_global_offset):
		# Convert heading to radians
		theta = np.radians(car_heading_degrees)
	
		# Standard Cartesian 2D frame rotation matrix
		rotation_matrix = np.array([
			[np.cos(theta), np.sin(theta)],
			[-np.sin(theta), np.cos(theta)]
		])
		
		local_coord = rotation_matrix.dot(relative_global_offset)
		return np.round(local_coord, 2).tolist()

	print("trailer to veh", tx, ty, theta_rad)
	# print("target to veh", message[-2][:2], (message[-2][2]-veh_h)%360)
	trailer_target_to_veh = global_to_local_rotate(-veh_h, message[-2][:2])
	print("target to veh", trailer_target_to_veh, (message[-2][2]-veh_h)%360)
	

	
	
	## from vr kit to vehicle direction coordinate
	new_trailer_l = global_to_local_rotate(veh_h, [tx, ty])
	# new_trailer_l = [tx,ty]
	new_trailer_h = (theta_rad+veh_h)%360
	# print("trailer", new_trailer_l)
	new_target_l = message[-2][:2]
	new_target_h = message[-2][2]

	message[0] = [new_trailer_l[0], new_trailer_l[1], (360-new_trailer_h)%360]
	# message[0][2] = (message[0][2]-180)%360

	## in phone VR coordinate vehicle as (0,0)
	print('init vr trailer loc', message[0])
	print('final vr trailer loc', message[-2])

	##! for Dan's coordinate (vehicle heading as 0, trailer init as 0,0; left y, front x):
	## shift to trailer center
	start_node_mid = [0, 0, theta_rad]
	# start_node = [0, 0, (360-theta_rad)%360]
	start_node = [0, 0, theta_rad]
	end_node_mid = [trailer_target_to_veh[0]-tx, trailer_target_to_veh[1]-ty, (message[-2][2]-veh_h)%360]
	end_node = [end_node_mid[1], -end_node_mid[0], (360-end_node_mid[2])%360]
	print("dan's coordinate start/target", start_node, end_node)

	## intersection
	p2_loc = intersection_tra(
		p0_3d=np.array(start_node)[:2],
		heading_start_deg=start_node[2], 
		p3_3d=np.array(end_node)[:2], 
		heading_end_deg=end_node[2]
	)

	p2_h = get_heading_p_to_p0(end_node[0:2], start_node[:2])
	inter_point = [p2_loc[0], p2_loc[1], p2_h]
	print("Intersection", p2_loc, p2_h)

	## projection 
	p2_loc = projection_tra(
		p0_3d=np.array(start_node)[:2],
		heading_start_deg=start_node[2], 
		p3_3d=np.array(end_node)[:2], 
		heading_end_deg=end_node[2]
	)
	p2_h = end_node[2]
	inter_point = [p2_loc[0], p2_loc[1], p2_h]
	print("Projection", p2_loc, p2_h)


	control_points = np.array([
		start_node, inter_point, inter_point, end_node
	])

	# Pass points into the interpolation
	p0, p1, p2, p3 = control_points[0], control_points[1], control_points[2], control_points[3]
	curve_points, headings = compute_bezier_tra(p0, p1, p2, p3, n=200) ## start/end point on line, middle for curve

	points_trailer_dan = []
	points_trailer_dan2 = []
	points_trailer_dan_trailer_h = []
	points_trailer_dan_trailer_h2 = []
	for i in range(200):
		points_trailer_dan.append([curve_points[i][0], curve_points[i][1], curve_points[i][2], 'r'])
		p_trailer_h = global_to_local_rotate(theta_rad, [curve_points[i][0], curve_points[i][1]]) # p / n
		points_trailer_dan_trailer_h.append([p_trailer_h[0], p_trailer_h[1], curve_points[i][2]-theta_rad, 'r']) # p / n
		points_trailer_dan2.append([curve_points[i][0], curve_points[i][1], (curve_points[i][2])%360, 'r'])
		points_trailer_dan_trailer_h2.append([p_trailer_h[0], p_trailer_h[1], ((curve_points[i][2]-theta_rad))%360, 'r']) # p or n
		
	print("downsample path (dan's coordinage)", points_trailer_dan[::50], points_trailer_dan[-1]) ## include last point in log
	print("downsample path trailer H (dan's coordinage)", points_trailer_dan_trailer_h[::50], points_trailer_dan_trailer_h[-1]) ## include last point in log

	with open("hummer_path/pathx200_dan.txt", "w", encoding='utf-8') as f:
		json.dump(points_trailer_dan2, f, indent=2)

	## rotate to trailer's current heading
	with open("hummer_path/pathx200_dan_trailer_heading.txt", "w", encoding='utf-8') as f:
		json.dump(points_trailer_dan_trailer_h2, f, indent=2)


	# extension point (not included in the send over file through can)
	def move_backward(p2, distance=5.0, degrees=True):
		x, y, heading = p2

		# Convert heading to radians if provided in degrees
		rad = math.radians(heading) if degrees else heading

		# Move in opposite direction of heading vector
		new_x = x - distance * math.cos(rad)
		new_y = y - distance * math.sin(rad)

		return (new_x, new_y, heading)
	
	extend_node = move_backward(end_node, distance=5)
	print("--- extend point at dan's coordinate", extend_node)

	# Interpolate to 150 points
	
	target_len = 150
	old_len = 200

	# Generates 150 indices from 0 to 199, guaranteed to start at 0 and end at 199
	indices = [round(i * (old_len - 1) / (target_len - 1)) for i in range(target_len)]

	# Select the elements at those indices
	A_150 = [points_trailer_dan2[i] for i in indices]

	points_trailer_dan_with_ext = A_150

	ext_step = 50
	for i in range(ext_step):
		px = ((ext_step-i)/ext_step)*end_node[0]+(i/ext_step)*extend_node[0]
		py = ((ext_step-i)/ext_step)*end_node[1]+(i/ext_step)*extend_node[1]
		ph = end_node[2]
		p = global_to_local_rotate(theta_rad, [px, py])
		points_trailer_dan_with_ext.append([p[0],p[1],ph])

	################

	# Justin's html 3D view coordinate
	# points_trailer_J = []
	
	start_j = message[0]
	end_j = message[-2]

	## intersection for Mid point (if not ahead of p0)
	p2_loc = intersection_tra_J_coor(
		p0_3d=np.array([start_j[0], start_j[1]]),
		heading_start_deg=start_j[2], 
		p3_3d=np.array([end_j[0], end_j[1]]), 
		heading_end_deg=end_j[2]
	)
	p2_h = get_heading_p_to_p0([end_j[0], end_j[1]], [start_j[0], start_j[1]])

	inter_point_j = [p2_loc[0], p2_loc[1], end_j[2]]

	print("Intersection new J", p2_loc, p2_h, inter_point, "between", start_j, end_j)

	## projection for Mid point (could be sharp)
	# p2_loc = projection_tra_J_coor(
	# 	p0_3d=np.array([start_j[0], start_j[1]]),
	# 	heading_start_deg=start_j[2], 
	# 	p3_3d=np.array([end_j[0], end_j[1]]), 
	# 	heading_end_deg=end_j[2]
	# )
	# p2_h = get_heading_p_to_p0([end_j[0], end_j[1]], [start_j[0], start_j[1]])

	# inter_point_j = [p2_loc[0], p2_loc[1], end_j[2]]

	# print("Projection new J", p2_loc, p2_h, inter_point, "between", start_j, end_j)



	control_points = np.array([
		start_j, inter_point_j, inter_point_j, end_j
	])

	p0, p1, p2, p3 = control_points[0], control_points[1], control_points[2], control_points[3]
	curve_points, headings = compute_bezier_tra_j(p0, p1, p2, p3, n=200) ## start/end point on line, middle for curve

	points_trailer_J2 = []

	for i in range(200):
		points_trailer_J2.append([curve_points[i][0], curve_points[i][1], curve_points[i][2], 'r'])
		
	print("downsample path (J's coordinage)", points_trailer_J2[::50], points_trailer_J2[-1]) ## include last point in log
	
	
	# print(len(points_trailer_dan_with_ext))
	# # for i in range(len(points_trailer_dan_with_ext)):
	# for i in range(len(points_trailer_dan2)):
	# 	## back to vrkit coordinate

	# 	## first rotate then shift
	# 	p0 = global_to_local_rotate(-theta_rad, [points_trailer_dan2[i][0], points_trailer_dan2[i][1]])
	# 	p = [-p0[1]+tx, p0[0]+ty, (360-points_trailer_dan_with_ext[i][2])%360, 'r']
		
	# 	## first shift then rotate
	# 	# p0 = [-points_trailer_dan2[i][1]+tx, points_trailer_dan2[i][0]+ty, (360-points_trailer_dan2[i][2])%360, 'r']
	# 	# p = global_to_local_rotate(-theta_rad, [p0[0], p0[1]])
		
	# 	## continue rotate back to vr
	# 	p2 = global_to_local_rotate(veh_h, [p[0], p[1]])
	# 	if i==0:
	# 		print("rot back", p0, p, p2)
	
	# 	points_trailer_J.append([p2[0], p2[1], p[2], 'r'])
	# 	# points_trailer_J.append([p2[0], p2[1], p0[2], 'r'])
	# 	# print(p)
		
	fn = "path_local_den_1_stage"
	with open("./hummer_path/%s.txt"%fn, 'w', encoding='utf-8') as file:
		# json.dump(points_trailer_J, file, indent=2)
		json.dump(points_trailer_J2, file, indent=2)

	fn = "pathx5"
	points_trailer_J_x5 = []
	for i in range(4):
		# p = points_trailer_J[i*50][:3]
		p = points_trailer_J2[i*50][:3]
		# print(p)
		points_trailer_J_x5.append(p)

	# points_trailer_J_x5[-2] = points_trailer_J_x5[-1]
	points_trailer_J_x5.append(points_trailer_J2[-1][:3])
	print("x5 trailer path J's coordinate", points_trailer_J_x5)
		
	with open("./hummer_path/%s.txt"%fn, 'w') as file:
		json.dump(points_trailer_J_x5, file)



	## vrkit to can sensed heading
	# can_heading_cali_trailer(theta_rad)

	## all path back to 0.0 heading, all objs/floor back to h0
	can_heading_cali_J(h0)

	
	##
	return jsonify({'status': 'success', 'echo': message}), 200



@app.route('/path_upload_x5_trailer_large_rad',  methods=['POST'])
def path_upload_x5_trailer_large_rad():

	# ## (option 1) this use the current gps and heading 
	os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p2.txt")
	os.system("cp ./hummer_path/can_heading.txt ./hummer_path/can_heading_ref.txt")

	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	message = json.loads(message)

	print(f"---- J Received ori path x5 for trailer: {message}")


	## transform to vehicle heading first

	# demo_path = [[0.0, 0.0, 31.8], [-2.8, -3.40, 263], [-7.0, -11.0, 256], [-3.1, -8.70, 294], [12.0, -6.0, 256]]
	transformed, h0 = transform_to_local_frame_by_p0_J_coor(message)

	# with open("./hummer_path/pathx5.txt"%fn, 'w', encoding='utf-8') as file:
	# 	json.dump(message, file)


	message = transformed
	print(f"---- J vehicle heading path x5 for trailer: {message}")


	# unified function estimated theta from vehicleCommanderSRP node
	
	d1 = 0.0
	d2 = 0.0
	theta_rad = 0.0
	tx = 0.0
	ty = 0.0

	pre = "hummer_path/"
	fn1 = 'uwb_recent_d1.txt'
	fn2 = 'uwb_recent_d2.txt'
	with open("%s%s"%(pre,fn1), "r") as f:
		data = json.loads(f.readline())
		d1 = data[1]
	
	with open("%s%s"%(pre,fn2), "r") as f:
		data = json.loads(f.readline())
		d2 = data[1]

	f_t = "./hummer_path/uwb_recent_theta.txt"
	with open(f_t, "r") as f:
		uwb_est = json.loads(f.readline())
		theta_rad = uwb_est[1]
		tx = uwb_est[2]
		ty = uwb_est[3]


	print("--J est. unify uwb decawave (trailer2veh)--", d1, d2, theta_rad, tx, ty)
	
	veh_h = message[0][2]

	def global_to_local_rotate(car_heading_degrees, relative_global_offset):
		# Convert heading to radians
		theta = np.radians(car_heading_degrees)
	
		# Standard Cartesian 2D frame rotation matrix
		rotation_matrix = np.array([
			[np.cos(theta), np.sin(theta)],
			[-np.sin(theta), np.cos(theta)]
		])
		
		local_coord = rotation_matrix.dot(relative_global_offset)
		return np.round(local_coord, 2).tolist()

	print("trailer to veh", tx, ty, theta_rad)
	# print("target to veh", message[-2][:2], (message[-2][2]-veh_h)%360)
	trailer_target_to_veh = global_to_local_rotate(-veh_h, message[-2][:2])
	print("target to veh", trailer_target_to_veh, (message[-2][2]-veh_h)%360)
	

	
	
	## from vr kit to vehicle direction coordinate
	new_trailer_l = global_to_local_rotate(veh_h, [tx, ty])
	# new_trailer_l = [tx,ty]
	new_trailer_h = (theta_rad+veh_h)%360
	# print("trailer", new_trailer_l)
	new_target_l = message[-2][:2]
	new_target_h = message[-2][2]

	message[0] = [new_trailer_l[0], new_trailer_l[1], (360-new_trailer_h)%360]
	# message[0][2] = (message[0][2]-180)%360

	## in phone VR coordinate vehicle as (0,0)
	print('init vr trailer loc', message[0])
	print('final vr trailer loc', message[-2])

	##! for Dan's coordinate (vehicle heading as 0, trailer init as 0,0; left y, front x):
	## shift to trailer center
	start_node_mid = [0, 0, theta_rad]
	# start_node = [0, 0, (360-theta_rad)%360]
	start_node = [0, 0, theta_rad]
	end_node_mid = [trailer_target_to_veh[0]-tx, trailer_target_to_veh[1]-ty, (message[-2][2]-veh_h)%360]
	end_node = [end_node_mid[1], -end_node_mid[0], (360-end_node_mid[2])%360]
	print("dan's coordinate start/target", start_node, end_node)

	## intersection
	p2_loc = intersection_tra(
		p0_3d=np.array(start_node)[:2],
		heading_start_deg=start_node[2], 
		p3_3d=np.array(end_node)[:2], 
		heading_end_deg=end_node[2]
	)

	p2_h = get_heading_p_to_p0(end_node[0:2], start_node[:2])
	inter_point = [p2_loc[0], p2_loc[1], p2_h]
	print("Intersection", p2_loc, p2_h)

	## projection 
	p2_loc = projection_tra(
		p0_3d=np.array(start_node)[:2],
		heading_start_deg=start_node[2], 
		p3_3d=np.array(end_node)[:2], 
		heading_end_deg=end_node[2]
	)
	p2_h = end_node[2]
	inter_point = [p2_loc[0], p2_loc[1], p2_h]
	print("Projection", p2_loc, p2_h)


	control_points = np.array([
		start_node, inter_point, inter_point, end_node
	])

	# Pass points into the interpolation
	# p0, p1, p2, p3 = control_points[0], control_points[1], control_points[2], control_points[3]
	# curve_points, headings = compute_bezier_tra(p0, p1, p2, p3, n=200) ## start/end point on line, middle for curve

	## with larger rad
	p0, p1, p2, p3 = generate_safe_bezier_deg(
		p0=[start_node[0], start_node[1]], 
		p3=[end_node[0], end_node[1]], 
		heading0_deg=start_node[2],    
		heading3_deg=end_node[2],    
		min_radius=10.0
	)
	p0 = np.array([p0[0], p0[1], start_node[2]])
	p1 = np.array([p1[0], p1[1], inter_point[2]])
	p2 = np.array([p2[0], p2[1], inter_point[2]])
	p3 = np.array([p3[0], p3[1], end_node[2]])
	# print(p0, p1, "xxx")
	curve_points, headings = compute_bezier_tra(p0, p1, p2, p3, n=200) ## start/end point on line, middle for curve


	points_trailer_dan = []
	points_trailer_dan2 = []
	points_trailer_dan_trailer_h = []
	points_trailer_dan_trailer_h2 = []
	for i in range(200):
		points_trailer_dan.append([curve_points[i][0], curve_points[i][1], curve_points[i][2], 'r'])
		p_trailer_h = global_to_local_rotate(theta_rad, [curve_points[i][0], curve_points[i][1]]) # p / n
		points_trailer_dan_trailer_h.append([p_trailer_h[0], p_trailer_h[1], curve_points[i][2]-theta_rad, 'r']) # p / n
		points_trailer_dan2.append([curve_points[i][0], curve_points[i][1], (curve_points[i][2])%360, 'r'])
		points_trailer_dan_trailer_h2.append([p_trailer_h[0], p_trailer_h[1], ((curve_points[i][2]-theta_rad))%360, 'r']) # p or n
		
	print("downsample path (dan's coordinage)", points_trailer_dan[::50], points_trailer_dan[-1]) ## include last point in log
	print("downsample path trailer H (dan's coordinage)", points_trailer_dan_trailer_h[::50], points_trailer_dan_trailer_h[-1]) ## include last point in log

	with open("hummer_path/pathx200_dan.txt", "w", encoding='utf-8') as f:
		json.dump(points_trailer_dan2, f, indent=2)

	## rotate to trailer's current heading
	with open("hummer_path/pathx200_dan_trailer_heading.txt", "w", encoding='utf-8') as f:
		json.dump(points_trailer_dan_trailer_h2, f, indent=2)


	# extension point (not included in the send over file through can)
	def move_backward(p2, distance=5.0, degrees=True):
		x, y, heading = p2

		# Convert heading to radians if provided in degrees
		rad = math.radians(heading) if degrees else heading

		# Move in opposite direction of heading vector
		new_x = x - distance * math.cos(rad)
		new_y = y - distance * math.sin(rad)

		return (new_x, new_y, heading)
	
	extend_node = move_backward(end_node, distance=5)
	print("--- extend point at dan's coordinate", extend_node)

	# Interpolate to 150 points
	
	target_len = 150
	old_len = 200

	# Generates 150 indices from 0 to 199, guaranteed to start at 0 and end at 199
	indices = [round(i * (old_len - 1) / (target_len - 1)) for i in range(target_len)]

	# Select the elements at those indices
	A_150 = [points_trailer_dan2[i] for i in indices]

	points_trailer_dan_with_ext = A_150

	ext_step = 50
	for i in range(ext_step):
		px = ((ext_step-i)/ext_step)*end_node[0]+(i/ext_step)*extend_node[0]
		py = ((ext_step-i)/ext_step)*end_node[1]+(i/ext_step)*extend_node[1]
		ph = end_node[2]
		p = global_to_local_rotate(theta_rad, [px, py])
		points_trailer_dan_with_ext.append([p[0],p[1],ph])

	################

	# Justin's html 3D view coordinate
	# points_trailer_J = []
	
	start_j = message[0]
	end_j = message[-2]

	## intersection for Mid point (if not ahead of p0)
	p2_loc = intersection_tra_J_coor(
		p0_3d=np.array([start_j[0], start_j[1]]),
		heading_start_deg=start_j[2], 
		p3_3d=np.array([end_j[0], end_j[1]]), 
		heading_end_deg=end_j[2]
	)
	p2_h = get_heading_p_to_p0([end_j[0], end_j[1]], [start_j[0], start_j[1]])

	inter_point_j = [p2_loc[0], p2_loc[1], end_j[2]]

	print("Intersection new J", p2_loc, p2_h, inter_point, "between", start_j, end_j)

	## projection for Mid point (could be sharp)
	# p2_loc = projection_tra_J_coor(
	# 	p0_3d=np.array([start_j[0], start_j[1]]),
	# 	heading_start_deg=start_j[2], 
	# 	p3_3d=np.array([end_j[0], end_j[1]]), 
	# 	heading_end_deg=end_j[2]
	# )
	# p2_h = get_heading_p_to_p0([end_j[0], end_j[1]], [start_j[0], start_j[1]])

	# inter_point_j = [p2_loc[0], p2_loc[1], end_j[2]]

	# print("Projection new J", p2_loc, p2_h, inter_point, "between", start_j, end_j)



	control_points = np.array([
		start_j, inter_point_j, inter_point_j, end_j
	])

	p0, p1, p2, p3 = control_points[0], control_points[1], control_points[2], control_points[3]
	curve_points, headings = compute_bezier_tra_j(p0, p1, p2, p3, n=200) ## start/end point on line, middle for curve
	print(p0, p1)

	p0, p1, p2, p3 = generate_safe_bezier_deg_j(
		p0=[start_j[0], start_j[1]], 
		p3=[end_j[0], end_j[1]], 
		heading0_deg=start_j[2],     # Heading North (+Y)
		heading3_deg=end_j[2],    # Heading East (+X)
		min_radius=10.0
	)
	p0 = np.array([p0[0], p0[1], start_j[2]])
	p1 = np.array([p1[0], p1[1], inter_point_j[2]])
	p2 = np.array([p2[0], p2[1], inter_point_j[2]])
	p3 = np.array([p3[0], p3[1], end_j[2]])
	print(p0, p1, "xxx")
	curve_points, headings = compute_bezier_tra_j(p0, p1, p2, p3, n=200) ## start/end point on line, middle for curve


	points_trailer_J2 = []

	for i in range(200):
		points_trailer_J2.append([curve_points[i][0], curve_points[i][1], curve_points[i][2], 'r'])
		
	print("downsample path (J's coordinage)", points_trailer_J2[::50], points_trailer_J2[-1]) ## include last point in log
	
	
	# print(len(points_trailer_dan_with_ext))
	# # for i in range(len(points_trailer_dan_with_ext)):
	# for i in range(len(points_trailer_dan2)):
	# 	## back to vrkit coordinate

	# 	## first rotate then shift
	# 	p0 = global_to_local_rotate(-theta_rad, [points_trailer_dan2[i][0], points_trailer_dan2[i][1]])
	# 	p = [-p0[1]+tx, p0[0]+ty, (360-points_trailer_dan_with_ext[i][2])%360, 'r']
		
	# 	## first shift then rotate
	# 	# p0 = [-points_trailer_dan2[i][1]+tx, points_trailer_dan2[i][0]+ty, (360-points_trailer_dan2[i][2])%360, 'r']
	# 	# p = global_to_local_rotate(-theta_rad, [p0[0], p0[1]])
		
	# 	## continue rotate back to vr
	# 	p2 = global_to_local_rotate(veh_h, [p[0], p[1]])
	# 	if i==0:
	# 		print("rot back", p0, p, p2)
	
	# 	points_trailer_J.append([p2[0], p2[1], p[2], 'r'])
	# 	# points_trailer_J.append([p2[0], p2[1], p0[2], 'r'])
	# 	# print(p)
		
	fn = "path_local_den_1_stage"
	with open("./hummer_path/%s.txt"%fn, 'w', encoding='utf-8') as file:
		# json.dump(points_trailer_J, file, indent=2)
		json.dump(points_trailer_J2, file, indent=2)

	fn = "pathx5"
	points_trailer_J_x5 = []
	for i in range(4):
		# p = points_trailer_J[i*50][:3]
		p = points_trailer_J2[i*50][:3]
		# print(p)
		points_trailer_J_x5.append(p)

	# points_trailer_J_x5[-2] = points_trailer_J_x5[-1]
	points_trailer_J_x5.append(points_trailer_J2[-1][:3])
	print("x5 trailer path J's coordinate", points_trailer_J_x5)
		
	with open("./hummer_path/%s.txt"%fn, 'w') as file:
		json.dump(points_trailer_J_x5, file)



	## vrkit to can sensed heading
	# can_heading_cali_trailer(theta_rad)

	## all path back to 0.0 heading, all objs/floor back to h0
	can_heading_cali_J(h0)

	
	##
	return jsonify({'status': 'success', 'echo': message}), 200


@app.route('/path_upload_x5_trailer',  methods=['POST'])
def path_upload_x5_trailer_large_rad_fn():
# @app.route('/path_upload_x5_trailer_large_rad_front_end',  methods=['POST'])
# def path_upload_x5_trailer_large_rad_fn():

	# ## (option 1) this use the current gps and heading 
	os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p2.txt")
	os.system("cp ./hummer_path/can_heading.txt ./hummer_path/can_heading_ref.txt")

	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	message = json.loads(message)

	print(f"---- J Received ori path x5 for trailer: {message}")


	## transform to vehicle heading first

	# demo_path = [[0.0, 0.0, 31.8], [-2.8, -3.40, 263], [-7.0, -11.0, 256], [-3.1, -8.70, 294], [12.0, -6.0, 256]]
	transformed, h0 = transform_to_local_frame_by_p0_J_coor(message)

	# with open("./hummer_path/pathx5.txt"%fn, 'w', encoding='utf-8') as file:
	# 	json.dump(message, file)


	message = transformed
	print(f"---- J vehicle heading path x5 for trailer: {message}")


	# unified function estimated theta from vehicleCommanderSRP node
	
	d1 = 0.0
	d2 = 0.0
	theta_rad = 0.0
	tx = 0.0
	ty = 0.0

	pre = "hummer_path/"
	fn1 = 'uwb_recent_d1.txt'
	fn2 = 'uwb_recent_d2.txt'
	with open("%s%s"%(pre,fn1), "r") as f:
		data = json.loads(f.readline())
		d1 = data[1]
	
	with open("%s%s"%(pre,fn2), "r") as f:
		data = json.loads(f.readline())
		d2 = data[1]

	f_t = "./hummer_path/uwb_recent_theta.txt"
	with open(f_t, "r") as f:
		uwb_est = json.loads(f.readline())
		theta_rad = uwb_est[1]
		tx = uwb_est[2]
		ty = uwb_est[3]


	print("--J est. unify uwb decawave (trailer2veh)--", d1, d2, theta_rad, tx, ty)
	
	veh_h = message[0][2]

	def global_to_local_rotate(car_heading_degrees, relative_global_offset):
		# Convert heading to radians
		theta = np.radians(car_heading_degrees)
	
		# Standard Cartesian 2D frame rotation matrix
		rotation_matrix = np.array([
			[np.cos(theta), np.sin(theta)],
			[-np.sin(theta), np.cos(theta)]
		])
		
		local_coord = rotation_matrix.dot(relative_global_offset)
		return np.round(local_coord, 2).tolist()

	print("trailer to veh", tx, ty, theta_rad)
	# print("target to veh", message[-2][:2], (message[-2][2]-veh_h)%360)
	trailer_target_to_veh = global_to_local_rotate(-veh_h, message[-2][:2])
	print("target to veh", trailer_target_to_veh, (message[-2][2]-veh_h)%360)
	

	
	
	## from vr kit to vehicle direction coordinate
	new_trailer_l = global_to_local_rotate(veh_h, [tx, ty])
	# new_trailer_l = [tx,ty]
	new_trailer_h = (theta_rad+veh_h)%360
	# print("trailer", new_trailer_l)
	new_target_l = message[-2][:2]
	new_target_h = message[-2][2]

	message[0] = [new_trailer_l[0], new_trailer_l[1], (360-new_trailer_h)%360]
	# message[0][2] = (message[0][2]-180)%360

	## in phone VR coordinate vehicle as (0,0)
	print('init vr trailer loc', message[0])
	print('final vr trailer loc', message[-2])

	##! for Dan's coordinate (vehicle heading as 0, trailer init as 0,0; left y, front x):
	## shift to trailer center
	start_node_mid = [0, 0, theta_rad]
	# start_node = [0, 0, (360-theta_rad)%360]
	start_node = [0, 0, theta_rad]
	end_node_mid = [trailer_target_to_veh[0]-tx, trailer_target_to_veh[1]-ty, (message[-2][2]-veh_h)%360]
	end_node = [end_node_mid[1], -end_node_mid[0], (360-end_node_mid[2])%360]
	print("dan's coordinate start/target", start_node, end_node)

	## intersection
	p2_loc = intersection_tra(
		p0_3d=np.array(start_node)[:2],
		heading_start_deg=start_node[2], 
		p3_3d=np.array(end_node)[:2], 
		heading_end_deg=end_node[2]
	)

	p2_h = get_heading_p_to_p0(end_node[0:2], start_node[:2])
	inter_point = [p2_loc[0], p2_loc[1], p2_h]
	print("Intersection", p2_loc, p2_h)

	## projection 
	p2_loc = projection_tra(
		p0_3d=np.array(start_node)[:2],
		heading_start_deg=start_node[2], 
		p3_3d=np.array(end_node)[:2], 
		heading_end_deg=end_node[2]
	)
	p2_h = end_node[2]
	inter_point = [p2_loc[0], p2_loc[1], p2_h]
	print("Projection", p2_loc, p2_h)


	control_points = np.array([
		start_node, inter_point, inter_point, end_node
	])

	# Pass points into the interpolation
	# p0, p1, p2, p3 = control_points[0], control_points[1], control_points[2], control_points[3]
	# curve_points, headings = compute_bezier_tra(p0, p1, p2, p3, n=200) ## start/end point on line, middle for curve

	points_trailer_dan = []
	points_trailer_dan2 = []
	points_trailer_dan_trailer_h = []
	points_trailer_dan_trailer_h2 = []
	
	## add front/end straight
	p_pre = np.array([start_node[0], start_node[1]], dtype=float)
	rad0 = np.radians(start_node[2])
	dir0 = np.array([np.cos(rad0), np.sin(rad0)])
	# Move behind by subtracting the forward vector
	distance_pre = 3.0
	step = 25
	p_prime = p_pre - distance_pre * dir0
	
	p_post = np.array([end_node[0], end_node[1]], dtype=float)
	rad1 = np.radians(end_node[2])
	dir1 = np.array([np.cos(rad1), np.sin(rad1)])
	# Move behind by subtracting the forward vector
	distance_post = 3.0
	step = 25
	step_cur = 150
	p_prime2 = p_post + distance_post * dir1


	## with larger rad
	p0, p1, p2, p3 = generate_safe_bezier_deg(
		p0=[p_prime[0], p_prime[1]], 
		p3=[p_prime2[0], p_prime2[1]], 
		heading0_deg=start_node[2],    
		heading3_deg=end_node[2],    
		min_radius=10.0
	)
	p0 = np.array([p0[0], p0[1], start_node[2]])
	p1 = np.array([p1[0], p1[1], inter_point[2]])
	p2 = np.array([p2[0], p2[1], inter_point[2]])
	p3 = np.array([p3[0], p3[1], end_node[2]])
	# print(p0, p1, "xxx")
	curve_points, headings = compute_bezier_tra(p0, p1, p2, p3, n=step_cur) ## start/end point on line, middle for curve

	for i in range(step):
		p = [i/step*p_prime[0]+(step-i)/step*p_pre[0], i/step*p_prime[1]+(step-i)/step*p_pre[1]]
		points_trailer_dan2.append([p[0], p[1], (start_node[2])%360, 'r'])
		p_trailer_h = global_to_local_rotate(theta_rad, [p[0], p[1]]) # p / n
		points_trailer_dan_trailer_h2.append([p_trailer_h[0], p_trailer_h[1], ((start_node[2]-theta_rad))%360, 'r']) # p or n
	

	for i in range(step_cur):
		points_trailer_dan.append([curve_points[i][0], curve_points[i][1], curve_points[i][2], 'r'])
		p_trailer_h = global_to_local_rotate(theta_rad, [curve_points[i][0], curve_points[i][1]]) # p / n
		points_trailer_dan_trailer_h.append([p_trailer_h[0], p_trailer_h[1], curve_points[i][2]-theta_rad, 'r']) # p / n
		points_trailer_dan2.append([curve_points[i][0], curve_points[i][1], (curve_points[i][2])%360, 'r'])
		points_trailer_dan_trailer_h2.append([p_trailer_h[0], p_trailer_h[1], ((curve_points[i][2]-theta_rad))%360, 'r']) # p or n
	
	for i in range(step):
		p = [(step-i)/step*p_prime2[0]+i/step*p_post[0], (step-i)/step*p_prime2[1]+i/step*p_post[1]]
		points_trailer_dan2.append([p[0], p[1], (end_node[2])%360, 'r'])
		p_trailer_h = global_to_local_rotate(theta_rad, [p[0], p[1]]) # p / n
		points_trailer_dan_trailer_h2.append([p_trailer_h[0], p_trailer_h[1], ((end_node[2]-theta_rad))%360, 'r']) # p or n
	


	print("downsample path (dan's coordinage)", points_trailer_dan[::50], points_trailer_dan[-1]) ## include last point in log
	print("downsample path trailer H (dan's coordinage)", points_trailer_dan_trailer_h[::50], points_trailer_dan_trailer_h[-1]) ## include last point in log

	with open("hummer_path/pathx200_dan.txt", "w", encoding='utf-8') as f:
		json.dump(points_trailer_dan2, f, indent=2)

	## rotate to trailer's current heading
	with open("hummer_path/pathx200_dan_trailer_heading.txt", "w", encoding='utf-8') as f:
		json.dump(points_trailer_dan_trailer_h2, f, indent=2)


	# extension point (not included in the send over file through can)
	def move_backward(p2, distance=5.0, degrees=True):
		x, y, heading = p2

		# Convert heading to radians if provided in degrees
		rad = math.radians(heading) if degrees else heading

		# Move in opposite direction of heading vector
		new_x = x - distance * math.cos(rad)
		new_y = y - distance * math.sin(rad)

		return (new_x, new_y, heading)
	
	extend_node = move_backward(end_node, distance=5)
	print("--- extend point at dan's coordinate", extend_node)

	# Interpolate to 150 points
	
	target_len = 150
	old_len = 200

	# Generates 150 indices from 0 to 199, guaranteed to start at 0 and end at 199
	indices = [round(i * (old_len - 1) / (target_len - 1)) for i in range(target_len)]

	# Select the elements at those indices
	A_150 = [points_trailer_dan2[i] for i in indices]

	points_trailer_dan_with_ext = A_150

	ext_step = 50
	for i in range(ext_step):
		px = ((ext_step-i)/ext_step)*end_node[0]+(i/ext_step)*extend_node[0]
		py = ((ext_step-i)/ext_step)*end_node[1]+(i/ext_step)*extend_node[1]
		ph = end_node[2]
		p = global_to_local_rotate(theta_rad, [px, py])
		points_trailer_dan_with_ext.append([p[0],p[1],ph])

	################

	# Justin's html 3D view coordinate
	# points_trailer_J = []
	
	start_j = message[0]
	end_j = message[-2]

	## intersection for Mid point (if not ahead of p0)
	p2_loc = intersection_tra_J_coor(
		p0_3d=np.array([start_j[0], start_j[1]]),
		heading_start_deg=start_j[2], 
		p3_3d=np.array([end_j[0], end_j[1]]), 
		heading_end_deg=end_j[2]
	)
	p2_h = get_heading_p_to_p0([end_j[0], end_j[1]], [start_j[0], start_j[1]])

	inter_point_j = [p2_loc[0], p2_loc[1], end_j[2]]

	print("Intersection new J", p2_loc, p2_h, inter_point, "between", start_j, end_j)

	## projection for Mid point (could be sharp)
	# p2_loc = projection_tra_J_coor(
	# 	p0_3d=np.array([start_j[0], start_j[1]]),
	# 	heading_start_deg=start_j[2], 
	# 	p3_3d=np.array([end_j[0], end_j[1]]), 
	# 	heading_end_deg=end_j[2]
	# )
	# p2_h = get_heading_p_to_p0([end_j[0], end_j[1]], [start_j[0], start_j[1]])

	# inter_point_j = [p2_loc[0], p2_loc[1], end_j[2]]

	# print("Projection new J", p2_loc, p2_h, inter_point, "between", start_j, end_j)



	control_points = np.array([
		start_j, inter_point_j, inter_point_j, end_j
	])


	## add front/end straight
	p_pre = np.array([start_j[0], start_j[1]], dtype=float)
	rad0 = np.radians(start_j[2])
	dir0 = np.array([np.sin(rad0), np.cos(rad0)]) # J's clock-wise
	# Move behind by subtracting the forward vector
	distance_pre = 3.0
	step = 25
	p_prime = p_pre - distance_pre * dir0
	
	p_post = np.array([end_j[0], end_j[1]], dtype=float)
	rad1 = np.radians(end_j[2])
	dir1 = np.array([np.sin(rad1), np.cos(rad1)])
	# Move behind by subtracting the forward vector
	distance_post = 3.0
	step = 25
	step_cur = 150
	p_prime2 = p_post + distance_post * dir1


	## with larger rad
	p0, p1, p2, p3 = generate_safe_bezier_deg_j(
		p0=[p_prime[0], p_prime[1]], 
		p3=[p_prime2[0], p_prime2[1]], 
		heading0_deg=start_j[2],    
		heading3_deg=end_j[2],    
		min_radius=10.0
	)
	p0 = np.array([p0[0], p0[1], start_j[2]])
	p1 = np.array([p1[0], p1[1], inter_point_j[2]])
	p2 = np.array([p2[0], p2[1], inter_point_j[2]])
	p3 = np.array([p3[0], p3[1], end_j[2]])
	
	curve_points, headings = compute_bezier_tra_j(p0, p1, p2, p3, n=step_cur) ## start/end point on line, middle for curve

	points_trailer_J2 = []
	
	for i in range(step):
		p = [i/step*p_prime[0]+(step-i)/step*p_pre[0], i/step*p_prime[1]+(step-i)/step*p_pre[1]]
		points_trailer_J2.append([p[0], p[1], (start_j[2])%360, 'r'])

	for i in range(step_cur):
		points_trailer_J2.append([curve_points[i][0], curve_points[i][1], curve_points[i][2], 'r'])
		
	for i in range(step):
		p = [(step-i)/step*p_prime2[0]+i/step*p_post[0], (step-i)/step*p_prime2[1]+i/step*p_post[1]]
		points_trailer_J2.append([p[0], p[1], (end_j[2])%360, 'r'])
		
	###
	

	
	
	print("downsample path (J's coordinage)", points_trailer_J2[::50], points_trailer_J2[-1]) ## include last point in log
	
	
	# print(len(points_trailer_dan_with_ext))
	# # for i in range(len(points_trailer_dan_with_ext)):
	# for i in range(len(points_trailer_dan2)):
	# 	## back to vrkit coordinate

	# 	## first rotate then shift
	# 	p0 = global_to_local_rotate(-theta_rad, [points_trailer_dan2[i][0], points_trailer_dan2[i][1]])
	# 	p = [-p0[1]+tx, p0[0]+ty, (360-points_trailer_dan_with_ext[i][2])%360, 'r']
		
	# 	## first shift then rotate
	# 	# p0 = [-points_trailer_dan2[i][1]+tx, points_trailer_dan2[i][0]+ty, (360-points_trailer_dan2[i][2])%360, 'r']
	# 	# p = global_to_local_rotate(-theta_rad, [p0[0], p0[1]])
		
	# 	## continue rotate back to vr
	# 	p2 = global_to_local_rotate(veh_h, [p[0], p[1]])
	# 	if i==0:
	# 		print("rot back", p0, p, p2)
	
	# 	points_trailer_J.append([p2[0], p2[1], p[2], 'r'])
	# 	# points_trailer_J.append([p2[0], p2[1], p0[2], 'r'])
	# 	# print(p)
		
	fn = "path_local_den_1_stage"
	with open("./hummer_path/%s.txt"%fn, 'w', encoding='utf-8') as file:
		# json.dump(points_trailer_J, file, indent=2)
		json.dump(points_trailer_J2, file, indent=2)

	fn = "pathx5"
	points_trailer_J_x5 = []
	for i in range(4):
		# p = points_trailer_J[i*50][:3]
		p = points_trailer_J2[i*50][:3]
		# print(p)
		points_trailer_J_x5.append(p)

	# points_trailer_J_x5[-2] = points_trailer_J_x5[-1]
	points_trailer_J_x5.append(points_trailer_J2[-1][:3])
	print("x5 trailer path J's coordinate", points_trailer_J_x5)
		
	with open("./hummer_path/%s.txt"%fn, 'w') as file:
		json.dump(points_trailer_J_x5, file)



	## vrkit to can sensed heading
	# can_heading_cali_trailer(theta_rad)

	## all path back to 0.0 heading, all objs/floor back to h0
	can_heading_cali_J(h0)

	# update control state
	traj_control = "/home/connau/srp/birdview/hummer_path/execution_tag_trailer.txt"
	with open(traj_control, "w") as f:
		f.write("4")

	
	##
	return jsonify({'status': 'success', 'echo': message}), 200



def transform_to_local_frame_by_p0_J_coor(path):
    if not path:
        return []
    
    # 1. Establish the reference frame from the first element
    x0, y0, h0 = path[0]
    rad0 = math.radians(h0)
    
    cos_h0 = math.cos(rad0)
    sin_h0 = math.sin(rad0)
    
    transformed_path = []
    
    for x, y, heading in path:
        # 2. Translate relative to the first point
        dx = x - x0
        
        dy = y - y0
        
        # 3. Rotate using your custom coordinate rules:
        # Left = +X, Up = +Y
        # 0 deg = Up (+Y), 90 deg = Left (+X)
        local_x = dx * cos_h0 - dy * sin_h0
        local_y = dx * sin_h0 + dy * cos_h0
        
        # 4. Normalize the heading
        local_h = (heading - h0) % 360
        
        transformed_path.append([local_x, local_y, local_h])
        
    return transformed_path, h0


## deplicated
@app.route('/path_upload_x5_trailer2',  methods=['POST'])
def path_upload_x5_trailer2():

	
	## similar to x5_rev, but need to get initial trailer location and 
	## heading based on init uwb sensor to replace (0,0)

	## (option 1) this use the current gps and heading 
	os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p2.txt")
	os.system("cp ./hummer_path/can_heading.txt ./hummer_path/can_heading_ref.txt")

	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	
	# print("--", np.array(message))
	### estimate turning point p2 based on current p1,p3,p4
	# p2 = find_sta_turn_p_bri(np.array(message))

	# message[1] = p2
	
	# f = open("hummer_path/pathx5.txt", "w")
	# f.write(message)
	# f.close()
	print(f"---- J Received ori path x5 for trailer: {message}")

	## tbd (uwb for initial point)
	Rv = 0.82	# Half-width of trailer tags
	L_sv = 1.23  # Sensor is 1m ahead of hinge
	L_ht = 0.37  # Hinge to 2 Tag Line center distance 
	yh = -2.9   # Hinge position relative to vehicle center
	Lt = 3.0	# Hinge to Trailer center

	d1 = 0.0
	d2 = 0.0
	
	# fn1 = 'uwb_d1.txt'
	# fn2 = 'uwb_d2.txt'
	pre = "hummer_path/"
	# pre = "/home/ConnAu/ros_ws/"

	fn1 = 'uwb_recent_d1.txt'
	fn2 = 'uwb_recent_d2.txt'
	with open("%s%s"%(pre,fn1), "r") as f:
		data = json.loads(f.readline())
		d1 = data[1]
	
	with open("%s%s"%(pre,fn2), "r") as f:
		data = json.loads(f.readline())
		d2 = data[1]


	denominator = 4 * Rv * L_sv

	sin_theta = (d2**2 - d1**2) / denominator
	sin_theta = np.clip(sin_theta, -1.0, 1.0)
	theta_rad = np.arcsin(sin_theta)

	# 3. Calculate Trailer Position 
	tx = Lt * np.sin(theta_rad)
	ty = yh - Lt * np.cos(theta_rad)

	print("--J est. 0804 uwb decawave--", d1, d2, theta_rad)
	
	message = json.loads(message)
	veh_h = message[0][2]


	def global_to_local_rotate(car_heading_degrees, relative_global_offset):
		# Convert heading to radians
		theta = np.radians(car_heading_degrees)
	
		# Standard Cartesian 2D frame rotation matrix
		rotation_matrix = np.array([
			[np.cos(theta), np.sin(theta)],
			[-np.sin(theta), np.cos(theta)]
		])
		
		local_coord = rotation_matrix.dot(relative_global_offset)
		return np.round(local_coord, 2).tolist()

	# Car heading 270 degrees, trailer global offset (0, -6)
	new_trailer_l = global_to_local_rotate(veh_h, [tx, ty])
	new_trailer_h = (theta_rad+veh_h)%360
	print("trailer", new_trailer_l)

	# print("PPP")
	# print(message)
	# print("PPPP")
	# print(tx, ty, theta_rad+veh_h)
	# message[0] = [tx, ty, theta_rad]
	message[0] = [new_trailer_l[0], new_trailer_l[1], new_trailer_h]
	# message[0][2] = (message[0][2]-180)%360

	print('init trailer loc', message[0])
	print('final trailer loc', message[-1])

	# print("--J est. 0804 init path for trailer--", message)


	with open("hummer_path/pathx5_rev.txt", "w") as f:
		json.dump(message, f)
	# f.close()

	###### Justin 1024 reverse headings of path replace x5 ######
	## change both pathx5 and pathx5_can_heading if do it offline
	with open("hummer_path/pathx5_rev.txt", "r") as f:
		message = json.loads(f.readline())
	
	for i in range(len(message)):
		message[i][-1] = (message[i][-1]+180)%360
	print(f"---- flip path: {message}")

	fn = "pathx5"
	with open('hummer_path/%s.txt'%fn, 'w') as f:
		json.dump(message, f)

	############################################################
	## change both pathx5 and pathx5_can_heading if do it offline
	with open("hummer_path/%s.txt"%fn, "r") as f:
		message = json.loads(f.readline())
	# f.close()
	print("tt",message)

	p2 = find_sta_turn_p_bri(np.array(message))
	# p2 = [10,0,60]
	message[1] = p2
	print('turn point', p2)

	print("update 2nd anchor:", p2)
	# print(message, "d")
	with open('hummer_path/%s.txt'%fn, 'w') as f:
		json.dump(message, f)

	## generate path using 1st (init) and 4th (target) point only with x2 options
	## also include path interpulation
	fn = "pathx5"
	fn1 = "path_local_den_1_stage"
	fn2 = "path_local_den_2_stage"

	stage_resolution = 100

	## path planning (x8 scenario) + path interpulation
	# rel_loc_ana_and_path_inter(fn, stage_resolution, fn1, fn2)
	# rel_loc_ana_and_path_inter_simple(fn, stage_resolution, fn1, fn2, "forward")
	rel_loc_ana_and_path_inter_trailer(fn, stage_resolution, fn1, fn2, "forward")
	
	## adjust pathx3, floor, obs, start,end postion & orientation by can sensed heading
	can_heading_cali()

	## flip to reverse gear
	fn = "pathx5_can_heading"
	with open("./hummer_path/%s.txt"%fn, 'r') as file:
		data = json.load(file)

	for i in range(len(data)):
		data[i][2] = (data[i][2]+180)%360

	with open("./hummer_path/%s_rev.txt"%fn, 'w') as file:
		json.dump(data, file)
	print("J est. 0804 trailer path", data)


	## just reverse the pathx5_can_heading interpolated data (den1/den2)
	fin = "path_local_den_1_stage_can_heading"
	reverse_path(fin)


	# fin2 = "path_local_den_2_stage_can_heading"
	# reverse_path(fin2)


	## ********* check why flip x/y Justin 1114
	datar = []
	for d in data:
		datar.append([-d[0], -d[1], d[2]])
	fn = "pathx5_can_heading"
	with open("./hummer_path/%s.txt"%fn, 'w') as file:
		json.dump(datar, file)

	fn = "floor_vers_can_heading"
	with open("./hummer_path/%s.txt"%fn, 'r') as file:
		dataf = json.load(file)
	datafr = []
	for d in dataf:
		datafr.append([-d[0], -d[1]])
	
	with open("./hummer_path/%s.txt"%fn, 'w') as file:
		json.dump(datafr, file)

	fn = "obs_can_heading"
	with open("./hummer_path/%s.txt"%fn, 'r') as file:
		datao = json.load(file)
	dataor = []
	for d in datao:
		dataor.append([-d[0], -d[1], d[2]])
	
	with open("./hummer_path/%s.txt"%fn, 'w') as file:
		json.dump(dataor, file)

	fn = "path_local_den_1_stage_can_heading"
	with open("./hummer_path/%s.txt"%fn, 'r') as file:
		data_inter = json.load(file)
	data_inter_r = []
	for d in data_inter:
		data_inter_r.append([-d[0], -d[1], d[2], 'r'])
	
	with open("./hummer_path/%s.txt"%fn, 'w', encoding='utf-8') as file:
		json.dump(data_inter_r, file, indent=2)


	## reverse 180 back for path_x5_can_heading (1118, update for johnson to gen trajectory)
	fn = "pathx5_can_heading"
	with open("./hummer_path/%s.txt"%fn, 'r') as file:
		data = json.load(file)

	for i in range(len(data)):
		data[i][2] = (data[i][2]+180)%360

	with open("./hummer_path/%s.txt"%fn, 'w') as file:
		data[1] = data[0]
		json.dump(data, file)


	## add 2nd trajectory for moving car out of parking spot (reverse)
	# fin = "pathx5_can_heading"
	# with open("./hummer_path/%s.txt"%fin, "r") as file:
	# 	path_x5 = json.load(file)

	# p_s = path_x5[3]
	# p_e = path_x5[2]
	# reverse_pointx5 = [p_s, p_s, p_s, p_e, p_e]	

	# # print("reverse point", reverse_pointx5)
	# fon = "pathx5_reverse_test"
	# with open("./hummer_path/%s.txt"%fon, "w") as file:
	# 	json.dump(reverse_pointx5, file)


	# p_s_r = [p_s[0], p_s[1], -p_s[2]]
	# p_e_r = [p_e[0], p_e[1], -p_e[2]]
	# reverse_pointx5_2 = [p_s_r, p_s_r, p_s_r, p_e_r, p_e_r]	

	# fon = "pathx5_reverse_test_rev"
	# with open("./hummer_path/%s.txt"%fon, "w") as file:
	# 	json.dump(reverse_pointx5_2, file)


	# ## clear history phone location data
	# fon = "phone_local_can_heading_all"
	# with open("./hummer_path/%s.txt"%fon, "w") as file:
	# 	file.write("")

	return jsonify({'status': 'success', 'echo': message}), 200





@app.route('/floor_upload',  methods=['POST'])
def floor_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	with open("hummer_path/floor_vers.txt", "w") as f:
		f.write(message)
	# f.close()

	return jsonify({'status': 'success', 'echo': message}), 200

	# print(f"--------Received message floor vertices: {message}")


@app.route('/obs_upload',  methods=['POST'])
def obs_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	with open("hummer_path/obs.txt", "w") as f:
		f.write(message)
	# f.close()

	return jsonify({'status': 'success', 'echo': message}), 200

	# print(f"--------Received message obs ll: {message}")


## sampled by phone 10cm
@app.route('/allP_upload',  methods=['POST'])
def allP_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	with open("hummer_path/allP.txt", "w") as f:
		f.write(message)
	# f.close()

	return jsonify({'status': 'success', 'echo': message}), 200



@app.route('/camera_local_upload',  methods=['POST'])
def person_local_upload():
	# data = request.get_json()
	# if not data or 'message' not in data:
	# 	return jsonify({'error': 'Invalid data'}), 400

	# message = data['message']

	# f = open("./hummer_path/phone_local.txt", "w")
	# f.write(message)
	# f.close()
	# # print(f"Received message: {message}")

	# ## rotate phone location to can heading coordinate
	# print("bf cali pho", message)
	# phone_loc = can_heading_cali_phone_loc()
	# print("af cali pho", phone_loc)

	# f = open("./hummer_path/phone_local_can_heading.txt", "w")
	# line = f.readline()
	# phone_loc = json.loads(line)
	# f.close()

	return jsonify({'status': 'success', 'echo': None}), 200





@app.route('/camera_gps_upload',  methods=['POST'])
def person_gps_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	with open("person.txt", "w") as f:
		f.write(message)
	# f.close()
	# print(f"Received message: {message}")

	
	return jsonify({'status': 'success', 'echo': message}), 200


@app.route('/path_local_upload',  methods=['POST'])
def path_local_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	with open("path_local.txt", "w") as f:
		f.write(message)
	# f.close()
	# print(f"Received message: {message}")

	
	return jsonify({'status': 'success', 'echo': message}), 200


@app.route('/path_local_upload_with_head',  methods=['POST'])
def path_local_upload_wh():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	with open("path_localx3_with_head.txt", "w") as f:
		f.write(message)
	# f.close()
	# print(f"Received message: {message}")


	## smooth path by interpulation
	with open("path_localx3_with_head.txt", "r") as f:
		pts = json.loads(f.readline())
	# f.close()
	# print(pts)

	curve_scale = 1.0*5
	granularity = 10

	new_path = []
	## unit [m, m, degree]
	# new_path = smooth_path_with_heading(pts, curve_scale, granularity)
	# print(new_path)
	
	with open('path_localxn_with_head.txt', 'w', encoding='utf-8') as f:
		json.dump(new_path, f, indent=2)
	f.close()
	
	return jsonify({'status': 'success', 'echo': message}), 200



@app.route('/path_local_v_center_upload',  methods=['POST'])
def path_local_v_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	with open("path_local_v.txt", "w") as f:
		f.write(message)
	# f.close()
	# print(f"Received message: {message}")

	
	return jsonify({'status': 'success', 'echo': message}), 200



@app.route('/uwb_alert_upload',  methods=['POST'])
def uwb_alert_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	with open("uwb_distance.txt", "w") as f:
		f.write(message)
	# f.close()
	# print(f"Received uwb message: {message}")

	
	return jsonify({'status': 'success', 'echo': message}), 200


@app.route('/uwb_location_upload',  methods=['POST'])
def uwb_location_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	with open("uwb_location.txt", "w") as f:
		f.write(message)
	# f.close()
	# print(f"Received uwb message: {message}")

	
	return jsonify({'status': 'success', 'echo': message}), 200


@app.route('/uwb_anc1_upload',  methods=['POST'])
def uwb_anc1_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	with open("hummer_path/UWB_anc1.txt", "w") as f:
		f.write(message)
	# f.close()
	# print(f"Received uwb message: {message}")

	
	return jsonify({'status': 'success', 'echo': message}), 200


@app.route('/uwb_anc2_upload',  methods=['POST'])
def uwb_anc2_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	with open("hummer_path/UWB_anc2.txt", "w") as f:
		f.write(message)
	# f.close()
	# print(f"Received uwb message: {message}")

	
	return jsonify({'status': 'success', 'echo': message}), 200


@app.route('/lidar_distance_upload',  methods=['POST'])
def lidar_dis_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	with open("lidar_distance.txt", "w") as f:
		f.write(message)
	# f.close()
	# print(f"Received lidar message: {message}")

	return jsonify({'status': 'success', 'echo': message}), 200





# read

## debug!!
## phone must face forward to get real vehicle orientation
@app.route('/fetch_veh_gps')
def gps_n_heading_read():
	with open("veh_gps.txt", "r") as f:
		data = f.readline()
	# f.close()
	# print("raw veh gps:", data)
	veh_gps = json.loads(data.replace("'", '"').replace("None", "null"))
	# print("load veh gps", veh_gps)

	# return jsonify(path)
	return jsonify(veh_gps)




if __name__ == '__main__':
	# app.run(host='0.0.0.0', port=5001, debug=True, ssl_context=('cert.pem', 'key.pem'))
	app.run(host='0.0.0.0', port=5000, debug=True)

