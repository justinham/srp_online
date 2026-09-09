
## req by three
#import matplotlib.pyplot as plt
from flask import Flask, render_template, request, jsonify
import json
# from simpleGPS import *

from pynmeagps import NMEAReader
import math




## req by iphone
import numpy as np
from scipy.interpolate import CubicHermiteSpline

from scipy.interpolate import CubicSpline, make_interp_spline

from pynmeagps import NMEAReader

from scipy.interpolate import UnivariateSpline

# import pandas as pd
import os

# os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p.txt")


# import matplotlib.pyplot as plt
show_fig = False
app = Flask(__name__, static_folder='static')






## req by iphone
############################################################

def smooth_component(arr, t, t_smooth, k=2, smooth_factor=None):
    """
    Smooth a data sequence using UnivariateSpline with specified smoothness.
    """
    s = smooth_factor or (len(arr) * np.var(arr) * 0.001)  # baseline smoothing
    print("sf", s)
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

	# 2️⃣ Unwrap heading to remove ±π jumps
	xyh[:, 2] = np.unwrap(xyh[:, 2])  # ensures continuous heading signal :contentReference[oaicite:1]{index=1}

	# 3️⃣ Smooth x, y, and heading independently with quadratic smoothing splines
	t = np.arange(len(xyh))
	t_smooth = np.linspace(0, len(xyh) - 1, tot_point)


	x_s = smooth_component(xyh[:, 0], t, t_smooth)
	y_s = smooth_component(xyh[:, 1], t, t_smooth)
	h_s = smooth_component(xyh[:, 2], t, t_smooth)

	# 4️⃣ Convert back to lat/lon and normalize heading
	lat_s = lat0 + np.degrees(y_s / R)
	lon_s = lon0 + np.degrees(x_s / (R * cosLat0))
	h_s = (h_s + np.pi) % (2 * np.pi) - np.pi  # wrap into [-π,π)

	return lat_s, lon_s, h_s, x_s, y_s




def path_antenna_cali(ant_offset):

	f = open("hummer_path/pathx5.txt", "r")
	line = f.readline()
	path_loc = json.loads(line)
	f.close()
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
	print(points_n)
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
	print(points_n)
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
	print(points_n)
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
	print(points_n)
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



def rel_loc_ana_and_path_inter(fn, stage_resolution, fn1, fn2):

	## load file
	f = open("./hummer_path/%s.txt"%fn, "r")
	line = f.readline()
	path_loc = json.loads(line)
	f.close()
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

		print("Intersection at:", intersection)

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
	points_two_stage_den = []
	points_two_stage_den = []

	## facing toward trajatory (very complicated logic...)
	## bizard curve scale=4.0 test best
	if (f_vs_r=="forward" and le_vs_ri=="left"):
		option = "tbd"

		option = "1 direct front in"
		print(option)
		points_n1 = anchor_forward(points, le_vs_ri)
		print(points_n1)
		if show_fig:
			show_path(points_n1, option)
		points_one_stage_anchor = points_n1

		path1,headings1 = generate_path(points_n1[:-1], heading_scale=4.0, resolution=stage_resolution)
		path2,headings2 = generate_path(points_n1[1:], heading_scale=4.0, resolution=stage_resolution)
		path_sum = np.concatenate((path1, path2))
		heading_sum = np.concatenate((headings1, headings2))
		# print(path_sum)
		for xy,h in zip(path_sum, heading_sum):
			print(xy[0], xy[1], h[0])
		points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum, heading_sum)]
		print(points_one_stage_den)

		if show_fig:
			plot_path_with_headings(points_one_stage_anchor, path_sum)

		with open('hummer_path/%s.txt'%fn1, 'w', encoding='utf-8') as f:
			json.dump(points_one_stage_den, f, indent=2)
		# with open('hummer_path/path_local_den_1_stage_test1.txt', 'w', encoding='utf-8') as f:
		# 	json.dump(points_one_stage_den, f, indent=2)
		
		
		option = "2 front shift back in"
		print(option)
		points_n2 = anchor_forward_2stage(points, le_vs_ri)
		print(points_n2)
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

		for xy,h in zip(path_sum2, heading_sum2):
			print(xy[0], xy[1], h[0])
		# print(path_sum2)
		points_two_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum2, heading_sum2)]
		for i in range(stage_resolution, stage_resolution*2):
			points_two_stage_den[i][3] = "r"
		print(points_two_stage_den)
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
		print(points_n1)
		if show_fig:
			show_path(points_n1, option)
		points_one_stage_anchor = points_n1

		path1,headings1 = generate_path(points_n1[:-1], heading_scale=4.0, resolution=stage_resolution)
		path2,headings2 = generate_path(points_n1[1:], heading_scale=4.0, resolution=stage_resolution)
		path_sum = np.concatenate((path1, path2))
		heading_sum = np.concatenate((headings1, headings2))
		# print(path_sum)
		for xy,h in zip(path_sum, heading_sum):
			print(xy[0], xy[1], h[0])
		points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum, heading_sum)]
		print(points_one_stage_den)
		
		if show_fig:
			plot_path_with_headings(points_one_stage_anchor, path_sum)

		with open('hummer_path/%s.txt'%fn1, 'w', encoding='utf-8') as f:
			json.dump(points_one_stage_den, f, indent=2)
		


		option = "4 front shift back in"
		print(option)
		points_n2 = anchor_forward_2stage(points, le_vs_ri)
		print(points_n2)
		if show_fig:
			show_path(points_n2, option)
		points_two_stage_anchor = points_n2

		back_list = points_n2[1:]
		back_list = back_list[::-1]
		path11,headings11 = generate_path(points_n2[:-1], heading_scale=4.0, resolution=stage_resolution)
		path22,headings22 = generate_path(back_list, heading_scale=4.0, resolution=stage_resolution)
		print("rev", path22)
		path22 = path22[::-1]
		headings22 = headings22[::-1]
		path_sum2 = np.concatenate((path11, path22))
		heading_sum2 = np.concatenate((headings11, headings22))
		for xy,h in zip(path_sum2, heading_sum2):
			print(xy[0], xy[1], h[0])
		points_two_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum2, heading_sum2)]
		for i in range(stage_resolution, stage_resolution*2):
			points_two_stage_den[i][3] = "r"
		print(points_two_stage_den)

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
		print(points_n1)
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
		for xy,h in zip(path_sum, heading_sum):
			print(xy[0], xy[1], h[0])
		points_one_stage_den = [[xy[0], xy[1], h[0], "r"] for xy,h in zip(path_sum, heading_sum)]
		print(points_one_stage_den)
		
		if show_fig:
			plot_path_with_headings(points_one_stage_anchor, path_sum)

		with open('hummer_path/%s.txt'%fn1, 'w', encoding='utf-8') as f:
			json.dump(points_one_stage_den, f, indent=2)
		


		option = "6 back shift front in"
		print(option)
		points_n2 = anchor_backward_2stage(points, le_vs_ri)
		print(points_n2)
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
		for xy,h in zip(path_sum, heading_sum):
			print(xy[0], xy[1], h[0])
		points_two_stage_den = [[xy[0], xy[1], h[0], "r"] for xy,h in zip(path_sum, heading_sum)]
		print(points_two_stage_den)
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
		print(points_n1)
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
		print(headings1)
		print("path", path_sum)
		print("headings", heading_sum)
		for xy,h in zip(path_sum, heading_sum):
			print(xy[0], xy[1], h[0])
		points_one_stage_den = [[xy[0], xy[1], h[0], "r"] for xy,h in zip(path_sum, heading_sum)]
		print(points_one_stage_den)
		
		if show_fig:
			plot_path_with_headings(points_one_stage_anchor, path_sum)
		with open('hummer_path/%s.txt'%fn1, 'w', encoding='utf-8') as f:
			json.dump(points_one_stage_den, f, indent=2)
		

		option = "8 back shift front in"
		print(option)
		points_n2 = anchor_backward_2stage(points, le_vs_ri)
		print(points_n2)
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
		for xy,h in zip(path_sum, heading_sum):
			print(xy[0], xy[1], h[0])
		points_two_stage_den = [[xy[0], xy[1], h[0], "r"] for xy,h in zip(path_sum, heading_sum)]
		for i in range(stage_resolution, stage_resolution*2):
			points_two_stage_den[i][3] = "f"
		print(points_two_stage_den)
		
		if show_fig:
			plot_path_with_headings(points_two_stage_anchor, path_sum)

		with open('hummer_path/%s.txt'%fn2, 'w', encoding='utf-8') as f:
			json.dump(points_two_stage_den, f, indent=2)





@app.route('/path_upload',  methods=['POST'])
def path_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	f = open("path.txt", "w")
	f.write(message)
	f.close()
	# print(f"Received message: {message}")

	return jsonify({'status': 'success', 'echo': message}), 200


@app.route('/path_upload_x5',  methods=['POST'])
def path_upload_x5():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	f = open("hummer_path/pathx5.txt", "w")
	f.write(message)
	f.close()
	print(f"--------Received message x5: {message}")



	## convert to smooth path
	# realtime data
	f = open("hummer_path/pathx5.txt", "r")
	data = f.readline()
	f.close()
		
	path_gps_with_heading = json.loads(data)
	print(path_gps_with_heading)

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
	print("path cali gps key", pts_in_rad)


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

	## 2.4m offset calibration to use antenna mounting location as veh center
	ant_offset = 2.4
	path_antenna_cali(ant_offset)


	## generate path using 1st (init) and 4th (target) point only with x2 options
	## also include path interpulation
	fn = "pathx5"
	# fn = "pathx5_mod"
	# fn = "pathx5_10_6_test"
	fn1 = "path_local_den_1_stage"
	fn2 = "path_local_den_2_stage"

	stage_resolution = 150

	rel_loc_ana_and_path_inter(fn, stage_resolution, fn1, fn2)
	

	# set gps reference point when new path generated
	os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p.txt")


	return jsonify({'status': 'success', 'echo': message}), 200

@app.route('/floor_upload',  methods=['POST'])
def floor_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	f = open("hummer_path/floor_vers.txt", "w")
	f.write(message)
	f.close()
	print(f"--------Received message floor vertices: {message}")


@app.route('/obs_upload',  methods=['POST'])
def obs_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	f = open("hummer_path/obs.txt", "w")
	f.write(message)
	f.close()
	print(f"--------Received message obs ll: {message}")



@app.route('/camera_local_upload',  methods=['POST'])
def person_local_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	f = open("./hummer_path/person_local.txt", "w")
	f.write(message)
	f.close()
	# print(f"Received message: {message}")

	
	return jsonify({'status': 'success', 'echo': message}), 200





@app.route('/camera_gps_upload',  methods=['POST'])
def person_gps_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	f = open("person.txt", "w")
	f.write(message)
	f.close()
	# print(f"Received message: {message}")

	
	return jsonify({'status': 'success', 'echo': message}), 200


@app.route('/path_local_upload',  methods=['POST'])
def path_local_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	f = open("path_local.txt", "w")
	f.write(message)
	f.close()
	# print(f"Received message: {message}")

	
	return jsonify({'status': 'success', 'echo': message}), 200


@app.route('/path_local_upload_with_head',  methods=['POST'])
def path_local_upload_wh():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	f = open("path_localx3_with_head.txt", "w")
	f.write(message)
	f.close()
	# print(f"Received message: {message}")


	## smooth path by interpulation
	f = open("path_localx3_with_head.txt", "r")
	pts = json.loads(f.readline())
	f.close()
	print(pts)

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
	f = open("path_local_v.txt", "w")
	f.write(message)
	f.close()
	# print(f"Received message: {message}")

	
	return jsonify({'status': 'success', 'echo': message}), 200



@app.route('/uwb_alert_upload',  methods=['POST'])
def uwb_alert_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	f = open("uwb_distance.txt", "w")
	f.write(message)
	f.close()
	print(f"Received uwb message: {message}")

	
	return jsonify({'status': 'success', 'echo': message}), 200


@app.route('/uwb_location_upload',  methods=['POST'])
def uwb_location_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	f = open("uwb_location.txt", "w")
	f.write(message)
	f.close()
	print(f"Received uwb message: {message}")

	
	return jsonify({'status': 'success', 'echo': message}), 200


@app.route('/lidar_distance_upload',  methods=['POST'])
def lidar_dis_upload():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = data['message']
	f = open("lidar_distance.txt", "w")
	f.write(message)
	f.close()
	print(f"Received lidar message: {message}")

	
	return jsonify({'status': 'success', 'echo': message}), 200





## debug!!
## phone must face forward to get real vehicle orientation
@app.route('/fetch_veh_gps')
def gps_n_heading_read():
	f = open("veh_gps.txt", "r")
	data = f.readline()
	f.close()
	# print("raw veh gps:", data)
	veh_gps = json.loads(data.replace("'", '"').replace("None", "null"))
	# print("load veh gps", veh_gps)

	# return jsonify(path)
	return jsonify(veh_gps)




############################################################






















############################################################

## req by birdview

def gps_to_loc_meters(lat0, lon0, lat1, lon1):
	
	R = 6378137
	lat0_rad = math.radians(lat0)
	lat1_rad = math.radians(lat1)
	dlat = math.radians(lat1-lat0)
	dlon = math.radians(lon1-lon0)

	x = R*dlon*math.cos(lat0_rad)
	y = R*dlat

	heading_rad = math.atan2(x,y)
	# print(x,y)
	return x,y



@app.route('/')
def home():
#    return render_template('index.html')
    return render_template('index3D.html')


@app.route('/path.json')
def path_data():

	## load from the processed smooth path
	import csv
	f = open("hummer_path/pathx5_smooth_gps.csv", "r")
	h = f.readline()
	data = []
	for line in f:
		eles = line.split(",")
		data.append([float(eles[0]), float(eles[1])])
	f.close()
	return jsonify(data)

	print("smo path", data)
	# exit()

@app.route('/path_ll.json')
def path_ll_data():

	## load from the processed smooth path
	import csv
	f = open("hummer_path/pathx5.txt", "r")
	line = f.readline()
	path_loc = json.loads(line)
	f.close()
	return jsonify(path_loc)

	print("path ll", data)
	# exit()

@app.route('/path_den1.json')
def path_den1_data():

	## load from the processed smooth path
	with open("hummer_path/path_local_den_1_stage.txt", 'r') as f:
		path_loc_den = json.load(f)
	
	print(path_loc_den)
	return jsonify(path_loc_den)

@app.route('/path_den2.json')
def path_den2_data():

	## load from the processed smooth path
	with open("hummer_path/path_local_den_2_stage.txt", 'r') as f:
		path_loc_den = json.load(f)
	
	print(path_loc_den)
	return jsonify(path_loc_den)
	
@app.route('/obs_ll.json')
def obs_ll_data():

	## load from the processed smooth path
	import csv
	f = open("hummer_path/obs.txt", "r")
	line = f.readline()
	path_loc = json.loads(line)
	f.close()
	return jsonify(path_loc)

	# print("obs ll", data)
	# exit()


@app.route('/floor_ll.json')
def floor_data():

	## load from the processed smooth path
	f = open("hummer_path/floor_vers.txt", "r")
	line = f.readline()
	floor = json.loads(line)
	f.close()
	return jsonify(floor)

	# print("obs ll", data)
	# exit()




@app.route('/person.json')
def person_data():
	f = open("person.txt", "r")
	data = f.readline()
	f.close()
	
	person_gps = json.loads(data)[0]
	print("load", person_gps)

	# return jsonify(path)
	return jsonify(person_gps)


@app.route('/ublox.json')
def ublox_data():
	# f = open("hummer_path/realtime_ublox_gps.txt", "r")
	# data = f.readline()
	# f.close()
	
	# gps = json.loads(data)[0]
	# print("load ublox", gps)

	# return jsonify([gps])

	
	# gps to meter
	f = open("hummer_path/gps_ref_p.txt", "r")
	line = f.readline()
	gps0 = json.loads(line)
	f.close()

	f = open("hummer_path/gps.txt", "r")
	line = f.readline()
	gps1 = json.loads(line)
	f.close()

	print(gps0, gps1[1])
	x,y = gps_to_loc_meters(lat0=gps0[0], lon0=gps0[1], lat1=gps1[0], lon1=gps1[1])

	return jsonify([x,y])



@app.route('/init_gps.json')
def init_gps_data():
	f = open("hummer_path/init_veh_gps.txt", "r")
	data = f.readline()
	f.close()
	
	gps = json.loads(data)[0]
	print("load init veh", gps)

	return jsonify(gps)


@app.route('/person_local.json')
def person_local_data():
	f = open("./hummer_path/person_local.txt", "r")
	data = f.readline()
	f.close()
	
	person_gps = json.loads(data)[0]
	print("load", person_gps)

	# return jsonify(path)
	return jsonify(person_gps)








@app.route('/path_local.json')
def path_local_data():
	# planed_path = [[0.5349896, -1.0209819, -0.034887522], [1.1479379, -0.9539301, 0.022688236], [1.6387491, -0.9002394, 0.06879134], [2.0209935, -0.8584248, 0.104696505], [2.3082423, -0.82700217, 0.13167852], [2.5140662, -0.80448663, 0.15101206], [2.6520362, -0.7893937, 0.16397193], [2.735723, -0.7802391, 0.17183281], [2.778697, -0.7755382, 0.1758695], [2.7945297, -0.77380615, 0.17735669], [2.7967916, -0.77355874, 0.17756915], [2.7967916, -0.77355874, 0.17756915], [2.9501946, -0.7735586, -0.34228048], [3.052261, -0.7735588, -0.764819], [3.0863569, -0.7735586, -1.1076071], [3.0358512, -0.77355874, -1.3882055], [2.8841121, -0.77355874, -1.6241748], [2.6145074, -0.7735587, -1.8330758], [2.210405, -0.77355874, -2.0324688], [1.6551733, -0.77355874, -2.239915], [0.93218005, -0.77355874, -2.472974], [0.02479285, -0.77355874, -2.749208]]
	# veh_loc = [0.53, -0.03]
	# path = get_path(veh_loc, planed_path)

	# test_path_gps = [[42.60855579648876, -82.99335282511062], [42.608554184618896, -82.99334639877743], [42.60855289393257, -82.99334125296848], [42.60855188874457, -82.99333724540344], [42.60855113336653, -82.99333423379981], [42.608550592111705, -82.99333207587951], [42.608550229292256, -82.99333062936292], [42.6085500092209, -82.99332975196612], [42.60854989621195, -82.9933293014095], [42.60854985457704, -82.99332913541787], [42.6085498486289, -82.99332911170437], [42.6085498486289, -82.99332911170437], [42.6085449800624, -82.99332902898209], [42.608541077917856, -82.99332922751904], [42.608538030848315, -82.99332996081316], [42.608535727487535, -82.99333148234201], [42.60853405649284, -82.9933340456035], [42.608532906500145, -82.99333790408542], [42.60853216616034, -82.99334331127547], [42.60853172411148, -82.99335052066725], [42.60853146900874, -82.99335978574338], [42.60853128948803, -82.99337136000547]]
	
	f = open("path_local.txt", "r")
	data = f.readline()
	f.close()
	
	path_gps = json.loads(data)
	print("load", path_gps)

	# return jsonify(path)
	return jsonify(path_gps)




@app.route('/uwb_read.json')
def uwb_read():
	f = open("uwb_distance.txt", "r")
	data = f.readline()
	f.close()
	
	uwb_dis = json.loads(data)["distance"]
	print("load uwb distance", uwb_dis)

	# return jsonify(path)
	return jsonify(uwb_dis)


	# data = request.get_json()
	# if not data or 'message' not in data:
	# 	return jsonify({'error': 'Invalid data'}), 400

	# message = data['message']
	# f = open("uwb_distance.txt", "w")
	# f.write(message)
	# f.close()
	# print(f"Received uwb message: {message}")

	
	# return jsonify({'status': 'success', 'echo': message}), 200


# write
@app.route('/gps_update',  methods=['POST'])
def gps_update():
	data = request.get_json()
	if not data or 'message' not in data:
		return jsonify({'error': 'Invalid data'}), 400

	message = str(data['message'])
	f = open("veh_gps.txt", "w")
	f.write(message)
	f.close()
	print(f"Received veh gps message: {message}")

	
	return jsonify({'status': 'success', 'echo': message}), 200
















############################################################






if __name__ == '__main__':
	# app.run(host='0.0.0.0', port=5000, debug=True)
	# app.run(host='0.0.0.0', port=5001, debug=True, ssl_context=('cert.pem', 'key.pem'))
	# app.run(host='0.0.0.0', port=80, debug=True, ssl_context='adhoc')

	app.run(host='0.0.0.0', port=5002, debug=True)
	

    
	
