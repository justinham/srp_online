
# import matplotlib.pyplot as plt
from flask import Flask, render_template, request, jsonify
import json
import math

import numpy as np
from scipy.interpolate import CubicHermiteSpline

from scipy.interpolate import CubicSpline, make_interp_spline

from pynmeagps import NMEAReader
import math


show_fig = False


#########


## find p2 based on intersection
def find_sta_turn_p_bri(points):

	p2 = [0,0,0]

	# Extract p1, p3, and p4
	p1 = points[0, :2]  # (x, y)
	p3 = points[2, :2]
	p4 = points[3, :2]
	p5 = points[4, :2]
	# print(p1)
	xs = [p1[0], p3[0], p4[0], p5[0]]
	ys = [p1[1], p3[1], p4[1], p5[1]]

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

		to_p1_vector = p1[:2] - intersection
		distance = np.linalg.norm(to_p1_vector)

		# Check to avoid division by zero
		if distance == 0:
			moved_point = intersection  # Already at p1
		else:
			direction = to_p1_vector / distance  # Normalize
			moved_point = intersection + direction * 8.0  # Move 8 meters toward p1

		# Output the adjusted point
		print("Moved intersection point (8m toward p1):", moved_point)

		p2 = [moved_point[0],moved_point[1],points[0][2]]


	except np.linalg.LinAlgError:
		print("No unique intersection found — lines may be parallel or ill-defined.")

	return p2

fn = "pathx5_can_heading"
fn = "pathx5"
## change both pathx5 and pathx5_can_heading if do it offline
f = open("hummer_path/%s.txt"%fn, "r")
message = json.loads(f.readline())
f.close()
print(message)

### estimate turning point p2 based on current p1,p3,p4
p2 = find_sta_turn_p_bri(np.array(message))
message[1] = p2

print(message)

with open('hummer_path/%s.txt'%fn, 'w') as f:
	json.dump(message, f)
exit()


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



def anchor_forward(points, le_vs_ri, intersection):
	
	# Convert heading (clockwise from North) to radians
	p_e = points[-1]
	back_distance = 6.0  
	inter_to_spot_dis = ((points[1][0]-intersection[0])**2+(points[1][1]-intersection[1])**2)**0.5


	## move back 6, turn right 90, move back 6
	p1_n = move_back(p_e, back_distance)
	# p1_n = move_back(p_e, inter_to_spot_dis)
	p1_n[2] = (p1_n[2]+90)%360
	if le_vs_ri=="right":
		p1_n[2] = (p1_n[2]-180)%360

	p2_n = move_back(p1_n, back_distance)
	
	points_n = [points[0], p2_n, points[1], points[2]]
	# print(points_n)
	return points_n


def anchor_forward_2stage(points, le_vs_ri, intersection):
	
	# Convert heading (clockwise from North) to radians
	p_e = points[-1]
	back_distance = 6.0  
	inter_to_spot_dis = ((points[1][0]-intersection[0])**2+(points[1][1]-intersection[1])**2)**0.5


	## move back 6, turn right 90, move forward 6, turn right 45
	p1_n = move_back(p_e, back_distance)
	# p1_n = move_back(p_e, inter_to_spot_dis)
	p1_n[2] = (p1_n[2]+90)%360
	if le_vs_ri=="right":
		p1_n[2] = (p1_n[2]-180)%360

	p2_n = move_forward(p1_n, back_distance)
	p2_n[2] = (p2_n[2])%360
	
	## end point reverse
	p_e_h = (points[-1][2]+180)%360
	p_f = [points[1][0], points[1][1], p_e_h]
	p_f2 = [points[2][0], points[2][1], p_e_h]
	
	points_n = [points[0], p2_n, p_f, p_f2]
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
	p_f2 = [points[2][0], points[2][1], p_e_h]
	
	 
	points_n = [points[0], p2_n, p_f, p_f2]
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
	points_n = [points[0], p2_n, points[1], points[2]]
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

	# print(np.vstack(curves).shape)
	# print(np.vstack(headings[0]).shape)
	
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
	mid_p = path_loc[2]
	end_p = path_loc[3]

	points = [start_p, mid_p, end_p]
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

	p_mid = np.array([mid_p[0], mid_p[1]])
	h_mid = mid_p[2]

	dir1 = heading_to_unit_vector(h1)
	dir2 = heading_to_unit_vector(h2)
	dir_mid = heading_to_unit_vector(h_mid)

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
	points_one_stage_den = []
	points_two_stage_den = []

	## facing toward trajatory (very complicated logic...)
	## bizard curve scale=4.0 test best
	if (f_vs_r=="forward" and le_vs_ri=="left"):
		option = "tbd"

		option = "1 direct front in"
		print(option)
		points_n1 = anchor_forward(points, le_vs_ri, intersection)
		# print(points_n1)
		if show_fig:
			show_path(points_n1, option)
		points_one_stage_anchor = points_n1

		## 4 points
		# path1,headings1 = generate_path(points_n1[:2], heading_scale=4.0, resolution=stage_resolution)
		# path_mid,headings_mid = generate_path(points_n1[1:-1], heading_scale=4.0, resolution=stage_resolution)
		# path2,headings2 = generate_path(points_n1[-2:], heading_scale=4.0, resolution=stage_resolution)
		# path_sum = np.concatenate((path1, path_mid, path2))
		# heading_sum = np.concatenate((headings1, headings_mid, headings2))
		
		# points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum, heading_sum)]
		# print(points_one_stage_den)

		## 3 point only
		points_n2 = [points_n1[0], points_n1[2], points_n1[3]]
		path1,headings1 = generate_path(points_n2[:2], heading_scale=4.0, resolution=stage_resolution)
		path2,headings2 = generate_path(points_n2[-2:], heading_scale=2.0, resolution=stage_resolution)
		path_sum = np.concatenate((path1, path2))
		heading_sum = np.concatenate((headings1, headings2))
		
		points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum, heading_sum)]

		# for ele in points_one_stage_den:
		# 	print(ele)
		# exit()

		if show_fig:
			plot_path_with_headings(points_one_stage_anchor, path_sum)

		with open('hummer_path/%s.txt'%fn1, 'w', encoding='utf-8') as f:
			json.dump(points_one_stage_den, f, indent=2)
		# with open('hummer_path/path_local_den_1_stage_test1.txt', 'w', encoding='utf-8') as f:
		# 	json.dump(points_one_stage_den, f, indent=2)
		
		
		option = "2 front shift back in"
		print(option)
		points_n2 = anchor_forward_2stage(points, le_vs_ri, intersection)
		# print(points_n2)
		if show_fig:
			show_path(points_n2, option)
		points_two_stage_anchor = points_n2

		back_list = [points_n2[-2],points_n2[-3]]
		back_list2 = [points_n2[-1], points_n2[-2]]
		
		path11,headings11 = generate_path(points_n2[:2], heading_scale=2.0, resolution=stage_resolution)
		path_mid,headings_mid = generate_path(back_list, heading_scale=4.0, resolution=stage_resolution)
		path22,headings22 = generate_path(back_list2, heading_scale=2.0, resolution=stage_resolution)
		# print("rev", path22)
		path22 = path22[::-1]
		headings22 = headings22[::-1]
		path_mid22 = path_mid[::-1]
		headings_mid22 = headings_mid[::-1]
		path_sum2 = np.concatenate((path11,path_mid22, path22))
		heading_sum2 = np.concatenate((headings11,headings_mid22, headings22))

		points_two_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum2, heading_sum2)]
		for i in range(stage_resolution, stage_resolution*3):
			points_two_stage_den[i][3] = "r"
		# print(points_two_stage_den)

		# for ele in points_two_stage_den:
		# 	print(ele)
		# exit()

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
		points_n1 = anchor_forward(points, le_vs_ri, intersection)
		# print(points_n1)
		if show_fig:
			show_path(points_n1, option)
		points_one_stage_anchor = points_n1

		## 4 points
		# path1,headings1 = generate_path(points_n1[:2], heading_scale=4.0, resolution=stage_resolution)
		# path_mid,headings_mid = generate_path(points_n1[1:-1], heading_scale=4.0, resolution=stage_resolution)
		# path2,headings2 = generate_path(points_n1[-2:], heading_scale=4.0, resolution=stage_resolution)
		# path_sum = np.concatenate((path1, path_mid, path2))
		# heading_sum = np.concatenate((headings1, headings_mid, headings2))
		# points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum, heading_sum)]
		# print(points_one_stage_den)
		
		## 3 point only
		points_n2 = [points_n1[0], points_n1[2], points_n1[3]]
		path1,headings1 = generate_path(points_n2[:2], heading_scale=4.0, resolution=stage_resolution)
		path2,headings2 = generate_path(points_n2[-2:], heading_scale=2.0, resolution=stage_resolution)
		path_sum = np.concatenate((path1, path2))
		heading_sum = np.concatenate((headings1, headings2))
		points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum, heading_sum)]
		
		# for ele in points_one_stage_den:
		# 	print(ele)
		# exit()

		if show_fig:
			plot_path_with_headings(points_one_stage_anchor, path_sum)

		with open('hummer_path/%s.txt'%fn1, 'w', encoding='utf-8') as f:
			json.dump(points_one_stage_den, f, indent=2)
		


		option = "4 front shift back in"
		print(option)
		points_n2 = anchor_forward_2stage(points, le_vs_ri, intersection)
		# print(points_n2)
		if show_fig:
			show_path(points_n2, option)
		points_two_stage_anchor = points_n2

		back_list = [points_n2[-2],points_n2[-3]]
		back_list2 = [points_n2[-1], points_n2[-2]]

		path11,headings11 = generate_path(points_n2[:2], heading_scale=2.0, resolution=stage_resolution)
		path_mid,headings_mid = generate_path(back_list, heading_scale=4.0, resolution=stage_resolution)
		path22,headings22 = generate_path(back_list2, heading_scale=2.0, resolution=stage_resolution)
		# print("rev", path22)
		path22 = path22[::-1]
		headings22 = headings22[::-1]
		path_mid22 = path_mid[::-1]
		headings_mid22 = headings_mid[::-1]
		path_sum2 = np.concatenate((path11,path_mid22, path22))
		heading_sum2 = np.concatenate((headings11,headings_mid22, headings22))

		points_two_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum2, heading_sum2)]
		
		# for ele in points_two_stage_den:
		# 	print(ele)
		# exit()

		for i in range(stage_resolution, stage_resolution*3):
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

		## 4x points
		# back_list1 = points_n1[:-2]
		# back_list1 = back_list1[::-1]
		# path1,headings1 = generate_path(back_list1, heading_scale=4.0, resolution=stage_resolution)
		# path1 = path1[::-1]
		# headings1 = headings1[::-1]

		# back_list2 = points_n1[1:-1]
		# back_list2 = back_list2[::-1]
		# path2,headings2 = generate_path(back_list2, heading_scale=4.0, resolution=stage_resolution)
		# path2 = path2[::-1]
		# headings2 = headings2[::-1]

		# back_list3 = points_n1[2:]
		# back_list3 = back_list3[::-1]
		# path3,headings3 = generate_path(back_list3, heading_scale=4.0, resolution=stage_resolution)
		# path3 = path3[::-1]
		# headings2 = headings2[::-1]
		
		# path_sum = np.concatenate((path1, path2, path3))
		# heading_sum = np.concatenate((headings1, headings2, headings3))
		# # for xy,h in zip(path_sum, heading_sum):
		# # 	print(xy[0], xy[1], h[0])
		# points_one_stage_den = [[xy[0], xy[1], h[0], "r"] for xy,h in zip(path_sum, heading_sum)]
		# # print(points_one_stage_den)


		## 3x points
		back_list1 = [points_n1[2],points_n1[0]]
		path1,headings1 = generate_path(back_list1, heading_scale=4.0, resolution=stage_resolution)
		path1 = path1[::-1]
		headings1 = headings1[::-1]

		back_list2 = [points_n1[3], points_n1[2]]
		path2,headings2 = generate_path(back_list2, heading_scale=2.0, resolution=stage_resolution)
		path2 = path2[::-1]
		headings2 = headings2[::-1]

		path_sum = np.concatenate((path1, path2))
		heading_sum = np.concatenate((headings1, headings2))
		# for xy,h in zip(path_sum, heading_sum):
		# 	print(xy[0], xy[1], h[0])
		points_one_stage_den = [[xy[0], xy[1], h[0], "r"] for xy,h in zip(path_sum, heading_sum)]
		# print(points_one_stage_den)

		# for ele in points_one_stage_den:
		# 	print(ele)
		# exit()
		
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

		back_list = [points_n2[1],points_n2[0]]
		
		# print(points_n2)
		path11,headings11 = generate_path(back_list, heading_scale=2.0, resolution=stage_resolution)
		path_mid,headings_mid = generate_path([points_n2[1],points_n2[2]], heading_scale=4.0, resolution=stage_resolution)
		path22,headings22 = generate_path([points_n2[2],points_n2[3]], heading_scale=2.0, resolution=stage_resolution)
		
		# print("rev", path22)
		path11 = path11[::-1]
		headings11 = headings11[::-1]
		path_sum2 = np.concatenate((path11, path_mid, path22))
		heading_sum2 = np.concatenate((headings11, headings_mid, headings22))

		points_two_stage_den = [[xy[0], xy[1], h[0], "r"] for xy,h in zip(path_sum2, heading_sum2)]
		# print(points_two_stage_den)
		for i in range(stage_resolution, stage_resolution*3):
			points_two_stage_den[i][3] = "f"
		
		# for ele in points_two_stage_den:
		# 	print(ele)
		# exit()

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

		## 4x points
		# back_list1 = points_n1[:-2]
		# back_list1 = back_list1[::-1]
		# path1,headings1 = generate_path(back_list1, heading_scale=4.0, resolution=stage_resolution)
		# path1 = path1[::-1]
		# headings1 = headings1[::-1]

		# back_list2 = points_n1[1:-1]
		# back_list2 = back_list2[::-1]
		# path2,headings2 = generate_path(back_list2, heading_scale=4.0, resolution=stage_resolution)
		# path2 = path2[::-1]
		# headings2 = headings2[::-1]

		# back_list3 = points_n1[2:]
		# back_list3 = back_list3[::-1]
		# path3,headings3 = generate_path(back_list3, heading_scale=4.0, resolution=stage_resolution)
		# path3 = path3[::-1]
		# headings2 = headings2[::-1]
		
		# path_sum = np.concatenate((path1, path2, path3))
		# heading_sum = np.concatenate((headings1, headings2, headings3))
	
		# points_one_stage_den = [[xy[0], xy[1], h[0], "r"] for xy,h in zip(path_sum, heading_sum)]
		# print(points_one_stage_den)

		## x3 points
		back_list1 = [points_n1[2],points_n1[0]]
		path1,headings1 = generate_path(back_list1, heading_scale=4.0, resolution=stage_resolution)
		path1 = path1[::-1]
		headings1 = headings1[::-1]

		back_list2 = [points_n1[3], points_n1[2]]
		# back_list2 = back_list2[::-1]
		path2,headings2 = generate_path(back_list2, heading_scale=2.0, resolution=stage_resolution)
		path2 = path2[::-1]
		headings2 = headings2[::-1]

		path_sum = np.concatenate((path1, path2))
		heading_sum = np.concatenate((headings1, headings2))
		
		points_one_stage_den = [[xy[0], xy[1], h[0], "r"] for xy,h in zip(path_sum, heading_sum)]
	
		# for ele in points_one_stage_den:
		# 	print(ele)
		# exit()
		
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


		back_list = [points_n2[1],points_n2[0]]
		
		# print(points_n2)
		path11,headings11 = generate_path(back_list, heading_scale=2.0, resolution=stage_resolution)
		path_mid,headings_mid = generate_path([points_n2[1],points_n2[2]], heading_scale=4.0, resolution=stage_resolution)
		path22,headings22 = generate_path([points_n2[2],points_n2[3]], heading_scale=2.0, resolution=stage_resolution)
		# print("rev", path22)
		path11 = path11[::-1]
		headings11 = headings11[::-1]
		path_sum2 = np.concatenate((path11, path_mid, path22))
		heading_sum2 = np.concatenate((headings11, headings_mid, headings22))

		points_two_stage_den = [[xy[0], xy[1], h[0], "r"] for xy,h in zip(path_sum2, heading_sum2)]

		for i in range(stage_resolution, stage_resolution*3):
			points_two_stage_den[i][3] = "f"
		
		# for ele in points_two_stage_den:
		# 	print(ele)
		# exit()
		
		if show_fig:
			plot_path_with_headings(points_two_stage_anchor, path_sum)

		with open('hummer_path/%s.txt'%fn2, 'w', encoding='utf-8') as f:
			json.dump(points_two_stage_den, f, indent=2)
		

fn = "pathx5"
# fn = "pathx5_mod"
# fn = "pathx5_10_6_test"
fn1 = "path_local_den_1_stage"
fn2 = "path_local_den_2_stage"

stage_resolution = 100

## J check resolution dens issue may overlap ##

rel_loc_ana_and_path_inter(fn, stage_resolution, fn1, fn2)








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


def rotate_path_pre(path, old_heading, new_heading):
    # rotation difference in radians
    delta_deg = new_heading - old_heading
    delta_rad = math.radians(delta_deg)
    
    cos_t = math.cos(delta_rad)
    sin_t = math.sin(delta_rad)

    rotated = []
    for (x, y, heading) in path:

        # Rotate coordinates around (0,0)
        new_x = x * cos_t - y * sin_t
        new_y = x * sin_t + y * cos_t

        # Rotate heading
        new_h = (heading + delta_deg) % 360

        rotated.append([new_x, new_y, new_h])
    
    return rotated


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

def can_heading_cali_abs():
	
	theta_deg = 0

	## all 3 paths

	fn = "can_heading_ref"
	f = open("./hummer_path/%s.txt"%fn, "r")
	line = f.readline()
	can_data = json.loads(line)
	f.close()
	can_h = float(can_data[2])

	fn = "pathx5"
	f = open("./hummer_path/%s.txt"%fn, "r")
	line = f.readline()
	path_x5 = json.loads(line)
	f.close()

	phone_compass_h = path_x5[0][2]

	## use vehicle heading
	# theta_deg_abs = abs(can_h-phone_compass_h)
	theta_deg_abs = abs(can_h-phone_compass_h)
	## use the phone compass heading
	# theta_deg_abs = 0
	## use the average
	# theta_deg_abs = abs(can_h-phone_compass_h)/2

	theta_deg_rel = rel_turning_direction(phone_compass_h, can_h)

	print("turning deg", theta_deg_rel)


	
	theta_deg = theta_deg_abs
	if theta_deg_rel=="counter-clockwise":
		theta_deg = -theta_deg_abs
	# print(can_h, phone_compass_h, theta_deg_rel, theta_deg)
	
	xy = [[x[0], x[1]] for x in path_x5]
	hs = [x[2] for x in path_x5]
	xy_rot = rotate_heading(xy, -theta_deg)
	# xy_rot = rotate_heading(xy, theta_deg)
	h_rot = [(h+theta_deg)%360 for h in hs]
	path_x5_rot = [[float(loc[0]), float(loc[1]), h] for loc, h in zip(xy_rot,h_rot)]

	fon = "pathx5_can_heading"
	fo = open("./hummer_path/%s.txt"%fon, "w")
	fo.write(str(path_x5_rot))
	fo.close()


	fn1 = "path_local_den_1_stage"
	with open("./hummer_path/%s.txt"%fn1, 'r') as f:
		path_xn = json.load(f)
	xy = [[x[0], x[1]] for x in path_xn]
	hs = [x[2] for x in path_xn]
	gs = [x[3] for x in path_xn]
	xy_rot = rotate_heading(xy, -theta_deg)
	# xy_rot = rotate_heading(xy, theta_deg)
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
	# xy_rot = rotate_heading(xy, theta_deg)
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
	# xy_rot = rotate_heading(xy, theta_deg)
	xy_rot = [[float(x[0]), -float(x[1])] for x in xy_rot]
	
	fon = "floor_vers_can_heading"
	fo = open("./hummer_path/%s.txt"%fon, "w")
	fo.write(str(xy_rot))
	fo.close()

	fn = "obs"
	with open("./hummer_path/%s.txt"%fn, 'r') as f:
		ps = json.load(f)
	obs_rot = []
	if len(ps)>0:
		xy = [[x[0], -x[1]] for x in ps]
		zs = [x[2] for x in ps]
		xy_rot = rotate_heading(xy, -theta_deg)
		# xy_rot = rotate_heading(xy, theta_deg)
		xy_rot = [[x[0], -x[1]] for x in xy_rot]
		obs_rot = [[float(loc[0]), float(loc[1]), z] for loc,z in zip(xy_rot,zs)]
		
	fon = "obs_can_heading"
	fo = open("./hummer_path/%s.txt"%fon, "w")
	fo.write(str(obs_rot))
	fo.close()

'''
def can_heading_cali():
	
	theta_deg = 0
	## all 3 paths

	fn = "can_heading_ref"
	f = open("./hummer_path/%s.txt"%fn, "r")
	line = f.readline()
	can_data = json.loads(line)
	f.close()
	can_h = float(can_data[2])

	fn = "pathx5"
	f = open("./hummer_path/%s.txt"%fn, "r")
	line = f.readline()
	path_x5 = json.loads(line)
	f.close()

	# path_x5[0][2] -= 3

	phone_compass_h = path_x5[0][2]

	## use vehicle heading
	path_x5_rot = rotate_path(path_x5, old_heading=phone_compass_h, new_heading=can_h)

	fon = "pathx5_can_heading"
	fo = open("./hummer_path/%s.txt"%fon, "w")
	fo.write(str(path_x5_rot))
	fo.close()
	
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
	fo = open("./hummer_path/%s.txt"%fon, "w")
	fo.write(str(xy_rot))
	fo.close()

	fn = "obs"
	with open("./hummer_path/%s.txt"%fn, 'r') as f:
		ps = json.load(f)
	obs_rot = []
	if len(ps)>0:

		# xyh = [[x[0], -x[1], 0] for x in ps]
		xyh = [[x[0], x[1], 0] for x in ps]
		zs = [x[2] for x in ps]
		xy_rot_h = rotate_path(xyh, old_heading=phone_compass_h, new_heading=can_h)
		xy_rot = [[float(x[0]), float(x[1])] for x in xy_rot_h]
		obs_rot = [[float(loc[0]), float(loc[1]), z] for loc,z in zip(xy_rot,zs)]

	fon = "obs_can_heading"
	fo = open("./hummer_path/%s.txt"%fon, "w")
	fo.write(str(obs_rot))
	fo.close()
'''


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
	f = open("./hummer_path/%s.txt"%fn, "r")
	line = f.readline()
	can_data = json.loads(line)
	f.close()
	can_h = float(can_data[2])

	fn = "pathx5"
	f = open("./hummer_path/%s.txt"%fn, "r")
	line = f.readline()
	path_x5 = json.loads(line)
	f.close()

	phone_compass_h = path_x5[0][2]

	## use vehicle heading
	path_x5_rot = rotate_path(path_x5, old_heading=phone_compass_h, new_heading=can_h)

	fon = "pathx5_can_heading"
	fo = open("./hummer_path/%s.txt"%fon, "w")
	fo.write(str(path_x5_rot))
	fo.close()


	#######
	## use both target and start heading for calibration
	phone_compass_h_target = path_x5[3][2]
	fn = "spot_heading_ref"
	f = open("./hummer_path/%s.txt"%fn, "r")
	line = f.readline()
	can_data_target = json.loads(line)
	f.close()
	can_h_target = float(can_data_target[2])

	path_x5_rot2 = rotate_path(path_x5, old_heading=phone_compass_h_target, new_heading=can_h_target)
	fon = "pathx5_can_heading2"
	fo = open("./hummer_path/%s.txt"%fon, "w")
	fo.write(str(path_x5_rot2))
	fo.close()

	path_x5_rot3 = rotate_path_multi(path_x5, old_headings=[phone_compass_h,phone_compass_h_target], new_headings=[can_h,can_h_target])
	fon = "pathx5_can_heading3"
	fo = open("./hummer_path/%s.txt"%fon, "w")
	fo.write(str(path_x5_rot3))
	fo.close()
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
	fo = open("./hummer_path/%s.txt"%fon, "w")
	fo.write(str(xy_rot))
	fo.close()

	fn = "obs"
	with open("./hummer_path/%s.txt"%fn, 'r') as f:
		ps = json.load(f)
	obs_rot = []
	if len(ps)>0:

		# xyh = [[x[0], -x[1], 0] for x in ps]
		xyh = [[x[0], x[1], 0] for x in ps]
		zs = [x[2] for x in ps]
		xy_rot_h = rotate_path(xyh, old_heading=phone_compass_h, new_heading=can_h)
		xy_rot = [[float(x[0]), float(x[1])] for x in xy_rot_h]
		obs_rot = [[float(loc[0]), float(loc[1]), z] for loc,z in zip(xy_rot,zs)]

	fon = "obs_can_heading"
	fo = open("./hummer_path/%s.txt"%fon, "w")
	fo.write(str(obs_rot))
	fo.close()



# can_heading_cali_abs()
can_heading_cali()



exit()





## move center to antenna
def path_antenna_cali(ant_offset):

	f = open("hummer_path/pathx5.txt", "r")
	line = f.readline()
	path_loc = json.loads(line)
	f.close()
	# print(path_loc)
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


	print(center_offset)

	plt.scatter([xs[0], xs[2], xs[3]], [ys[0], ys[2], ys[3]])
	plt.scatter([xs_new[0], xs_new[2], xs_new[3]], [ys_new[0], ys_new[2], ys_new[3]], marker='x')
	plt.xlim(-5, 20)
	plt.ylim(-5, 20)
	plt.show()

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

	plt.scatter([xs[0], xs[2], xs[3]], [ys[0], ys[2], ys[3]])
	plt.scatter([xs_new[0], xs_new[2], xs_new[3]], [ys_new[0], ys_new[2], ys_new[3]], marker='x')
	plt.xlim(-5, 20)
	plt.ylim(-5, 20)
	plt.show()

	with open("hummer_path/pathx5_antenna_cali.txt", "w") as f:
		json.dump(new_pointx5_ac, f)

ant_offset = 2.4
path_antenna_cali(ant_offset)


exit()


# def front_park():
# 	## reverse heading for path2
# 	f = open("path2.csv", "r")
# 	fo = open("path2_front_in.csv", "w")
# 	head = f.readline()
# 	fo.write(head)
# 	for line in f:
# 		eles = line.split(",")
# 		# print(eles)
# 		heading = float(eles[2])
# 		reverse_ang = (heading+3.1416)%3.1416
# 		fo.write("%s,%s,%f\n"%(eles[0], eles[1], reverse_ang))

# 	fo.close()
# 	f.close()


# front_park()
# exit()


def uwb_localization(dis_dict):
	mount_dict = {
		"anc1": [0.0, 0.5],
		"anc2": [0.0, -0.5],
		"anc3": [1.0, 0.5],
		"anc4": [0.0, -0.5]
	}
	return [0,0]

def verify_path():

	with open('path_localxn_with_head.txt', 'r', encoding='utf-8') as f:
		path = json.load(f) 
	print(path)

	# 1.get current ori, 
	# 2.check uwb to get phone position, 
	# 3.from phone(person_loc) map the veh location in vr coordinate. (x,y)
	# 4.path verification, compare to planned (x', y')
	# 5.step on to current target, update to next key stop

	cur_step = 0
	cur_pos = [path[cur_step][0], path[cur_step][1]]
	cur_ori = path[cur_step][2]
	next_target_pos = [path[cur_step+1][0], path[cur_step+1][1]]
	next_target_ori = path[cur_step+1][2]

	with open('uwb_multi_distance.txt', 'r', encoding='utf-8') as f:
		dis_dict = json.load(f)
	print(dis_dict)
	
	loc = uwb_localization(dis_dict)
	print(loc)

	with open('person_local.txt', 'r', encoding='utf-8') as f:
		phone_loc = json.load(f)[0]
	print(phone_loc)

	veh_loc

# verify_path()
# exit()




# def get_path(veh_loc, planed_path):

# 	xs = [x[0]-veh_loc[0] for x in planed_path]
# 	ys = [x[2]-veh_loc[1] for x in planed_path]

# 	# plt.plot(xs, ys)
# 	# plt.show()
# 	path = []
# 	for x,y in zip(xs,ys):
# 		path.append({"x":round(x*100), "y":round(y*100)}) # m to cm
# 	return path

def haversine(lat1, lon1, lat2, lon2):
	"""
	Calculate the great-circle distance between two points on Earth (in meters).
	Input values are in decimal degrees.
	"""
	R = 6_371_000  # Earth's radius in meters

	# Convert degrees to radians
	φ1, φ2 = math.radians(lat1), math.radians(lat2)
	Δφ = math.radians(lat2 - lat1)
	Δλ = math.radians(lon2 - lon1)

	a = math.sin(Δφ / 2)**2 + math.cos(φ1) * math.cos(φ2) * math.sin(Δλ / 2)**2
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

	return R * c


## u-blox
# msg_arr = []
# f = open("ublox_log2.txt", "r")
# for line in f:
# 	if "$GNGLL" in line:
# 		sentence = line.split(' ')[-1]
# 		print(sentence)
# 		msg = NMEAReader.parse(sentence)
# 		print(msg)  # Should output a GNGLL message with lat/lon values

# 		latitude = msg.lat	 # decimal degrees, >0 = N
# 		longitude = msg.lon	# decimal degrees, <0 = W
# 		msg_arr.append(msg)

# 		print(f"Latitude: {latitude}, Longitude: {longitude}")

# f.close()

# delta_arr = []
# for i in range(len(msg_arr)-1):
# 	gps1 = msg_arr[i]
# 	gps2 = msg_arr[i+1]
# 	try:
# 		dis = haversine(gps1.lat, gps1.lon, gps2.lat, gps2.lon)
# 		delta_arr.append(dis)
# 	except:
# 		pass
# print(delta_arr)

# plt.plot(delta_arr)
# plt.ylabel("delta distance in meter between two consequent readings")
# plt.show()


# exit()







## iphone


# def get_path(veh_loc, planed_path):

# 	xs = [x[0]-veh_loc[0] for x in planed_path]
# 	ys = [x[2]-veh_loc[1] for x in planed_path]

# 	# plt.plot(xs, ys)
# 	# plt.show()
# 	path = []
# 	for x,y in zip(xs,ys):
# 		path.append({"x":round(x*100), "y":round(y*100)}) # m to cm
# 	return path

def compute_headings(points):
	"""
	Given a list of [x, y] coordinates, returns a list of heading angles (radians)
	between each adjacent pair: heading from points[i] to points[i+1].
	"""
	headings = []
	for (x1, y1), (x2, y2) in zip(points[1:], points):
		dx = x2 - x1
		dy = y2 - y1
		angle = math.atan2(dy, dx)  # returns angle in radians, -π to +π
		headings.append(angle)
	return headings





## this is not needed for the birdview web UI, but required by the vehicle ROS node
# f = open("path_local.txt", "r")
# pts = json.loads(f.readline())
# f.close()

# angles_rad = compute_headings(pts)
# angles_deg = [math.degrees(angle) for angle in angles_rad]
# angles_deg_norm = [angle % 360 for angle in angles_deg]
# angles_rad_norm = [angle/180*math.pi for angle in angles_deg_norm]

# for i, (rad, deg, dn) in enumerate(zip(angles_rad, angles_deg, angles_deg_norm)):
#	 print(f"Segment {i} → {i+1}: {rad:.3f} rad, {deg:.1f}°, normal deg", dn)

# plt.scatter([x[0] for x in pts], [x[1] for x in pts])
# plt.show()

# loc_x = [x[0] for x in pts]
# loc_y = [x[1] for x in pts]
# loc_h = angles_rad_norm


# print(loc_x, loc_y, loc_h)

# exit()



## test generating path with key stops and orientation

def smooth_path_with_heading(pts, tangent_scale, granularity):

	## show original key stops
	loc_x = [x[0] for x in pts]
	loc_y = [x[1] for x in pts]
	loc_h = [x[2] for x in pts]

	angles_rad = np.deg2rad(loc_h)
	dx = np.cos(angles_rad)
	dy = np.sin(angles_rad)

	# Plot scatter and overlay arrows using quiver
	plt.plot(loc_x, loc_y, color='blue')
	plt.scatter(loc_x, loc_y, color='blue')
	plt.quiver(loc_x, loc_y, dx, dy, angles='xy', scale_units='xy', scale=1, color='red')
	plt.grid(True)
	plt.show()


	##### add mid point p2.5 ######
	# Extract coordinates and heading
	x1, y1, heading1 = pts[1]
	x2, y2, heading2 = pts[2]

	# Convert p2's heading to radians
	heading2_rad = np.deg2rad(heading2)

	# Define the direction vector for p2's heading
	dx2 = np.cos(heading2_rad)
	dy2 = np.sin(heading2_rad)
	direction_vector_p2 = np.array([dx2, dy2])

	# Vector from p2 to p1
	vector_p2_to_p1 = np.array([x1 - x2, y1 - y2])

	# Calculate the projection of vector_p2_to_p1 onto direction_vector_p2
	t = np.dot(vector_p2_to_p1, direction_vector_p2)

	# Calculate the intersection point (the projection of p1 onto the line defined by p2 and its heading)
	intersection_point = np.array([x2, y2]) + t * direction_vector_p2
	# print(intersection_point)

	mid_p = [(2*intersection_point[0]+2*pts[2][0])/4, (2*intersection_point[1]+2*pts[2][1])/4]
	p2p5 = [mid_p[0], mid_p[1], pts[2][2]]
	pts = pts[:2]+[p2p5]+[pts[-1]]
	print(pts)
	# exit()


	## generate smooth path ##
	x_coords = np.array([p[0] for p in pts])
	y_coords = np.array([p[1] for p in pts])
	final_orientations_deg = np.array([p[2] for p in pts])

	# --- Calculate the *path direction* at each point for reverse parking ---
	path_orientations_deg = final_orientations_deg + 180

	# Normalize angles to be within [0, 360) for consistency
	path_orientations_deg = np.fmod(path_orientations_deg, 360)
	path_orientations_deg[path_orientations_deg < 0] += 360

	# Convert path orientations from degrees to radians for trigonometric functions
	path_orientations_rad = np.deg2rad(path_orientations_deg)

	 # You can adjust this value to control curve tightness

	tangent_x_components = tangent_scale * np.cos(path_orientations_rad)
	tangent_y_components = tangent_scale * np.sin(path_orientations_rad)

	# --- Parameterize points for the spline ---
	# A simple parameterization: t values correspond to the index of the point.
	t_params = np.arange(len(pts))

	# --- Create CubicHermiteSpline objects ---
	spline_x = CubicHermiteSpline(t_params, x_coords, tangent_x_components)
	spline_y = CubicHermiteSpline(t_params, y_coords, tangent_y_components)

	# --- Generate dense points along the path ---
	# Create a dense set of 't' values across the full range of the spline.
	num_new_points = granularity # Number of points to generate for the smooth path
	t_new = np.linspace(t_params.min(), t_params.max(), num_new_points)

	# Evaluate the spline functions at these new 't' values to get the smooth path coordinates.
	x_new = spline_x(t_new)
	y_new = spline_y(t_new)

	dx_dt_new = spline_x(t_new, 1) # First derivative of x(t)
	dy_dt_new = spline_y(t_new, 1) # First derivative of y(t)

	h_new_rad = np.arctan2(dy_dt_new, dx_dt_new)

	# 3. Convert radians to degrees
	h_new_deg = np.rad2deg(h_new_rad)

	# --- Plotting the results for visualization ---
	# plt.figure(figsize=(10, 8))
	plt.clf()
	# Plot the original input points
	plt.scatter(x_coords, y_coords, color='blue', zorder=5, s=100, label='key stop points')
	for i in range(len(pts)):
		plt.text(x_coords[i] + 0.15, y_coords[i] + 0.15, f'P{i+1}', color='blue', fontsize=12)
	   
	# Plot the generated smooth, curved path
	plt.plot(x_new, y_new, color='green', linestyle='-', linewidth=2, label='interpolated curved path')


	# Normalize the angles to be within the [0, 360) degree range
	# np.fmod handles the modulo operation for potentially wrapping around (e.g., 370 becomes 10)
	reverse_loc_h = h_new_deg+180
	reverse_loc_h = np.fmod(reverse_loc_h, 360)
	loc_x = x_new
	loc_y = y_new
	loc_h = reverse_loc_h

	angles_rad = np.deg2rad(loc_h)
	dx = np.cos(angles_rad)
	dy = np.sin(angles_rad)


	# Plot scatter and overlay arrows using quiver
	plt.quiver(loc_x, loc_y, dx, dy, angles='xy', scale_units='xy', scale=1, color='red')

	plt.title('Reverse Parking Path (curve scale %d)'%tangent_scale)
	plt.xlabel('X Coordinate')
	plt.ylabel('Y Coordinate')
	plt.grid(True)
	# plt.axis('equal') # Ensures equal scaling for x and y axes, important for spatial accuracy
	plt.legend()
	plt.show()

	new_path =[[round(x,3),round(y,3),round(h,3)] for x,y,h in zip(loc_x, loc_y, loc_h)]
	print("smooth path", new_path)

	return new_path


import math

def local_to_gps(local_x_m, local_y_m, origin_lat_deg, origin_lon_deg):

	# Earth's mean radius in meters
	EARTH_RADIUS_METERS = 6371000.0

	# Convert origin latitude to radians for calculations
	origin_lat_rad = math.radians(origin_lat_deg)

	# Calculate meters per degree of latitude (approximately constant)
	# 1 degree of latitude = circumference / 360 degrees = 2 * pi * R / 360
	# Or more simply: 1 degree latitude ~ 111,139 meters (at the poles) to 110,574 meters (at the equator)
	# Using the mean Earth radius for a general approximation
	meters_per_degree_lat = EARTH_RADIUS_METERS * (math.pi / 180.0)

	# Calculate meters per degree of longitude at the given origin latitude
	# This varies with latitude: meters_per_degree_lon = (2 * pi * R * cos(lat_rad)) / 360
	meters_per_degree_lon = EARTH_RADIUS_METERS * math.cos(origin_lat_rad) * (math.pi / 180.0)

	# Calculate the change in latitude and longitude in degrees
	delta_lat_deg = local_y_m / meters_per_degree_lat
	delta_lon_deg = local_x_m / meters_per_degree_lon

	# Add the deltas to the origin GPS coordinates
	new_lat_deg = origin_lat_deg + delta_lat_deg
	new_lon_deg = origin_lon_deg + delta_lon_deg

	return (new_lat_deg, new_lon_deg)

# --- Example Usage ---

# Define the GPS coordinates for the local (0,0)




def smooth_path_with_heading_no_p3(pts, tangent_scale, granularity):

	## show original key stops
	loc_x = [x[0] for x in pts]
	loc_y = [x[1] for x in pts]
	loc_h = [x[2] for x in pts]

	angles_rad = np.deg2rad(loc_h)
	dx = np.cos(angles_rad)
	dy = np.sin(angles_rad)

	# Plot scatter and overlay arrows using quiver
	plt.plot(loc_x, loc_y, color='blue')
	plt.scatter(loc_x, loc_y, color='blue')
	plt.quiver(loc_x, loc_y, dx, dy, angles='xy', scale_units='xy', scale=1, color='red')
	plt.grid(True)
	# plt.xlim((-10, 10))
	# plt.ylim((-10, 10))
	plt.show()


	##### add mid point p2.5 ######
	# Extract coordinates and heading
	x1, y1, heading1 = pts[1]
	x2, y2, heading2 = pts[2]

	# Convert p2's heading to radians
	heading2_rad = np.deg2rad(heading2)

	# Define the direction vector for p2's heading
	dx2 = np.cos(heading2_rad)
	dy2 = np.sin(heading2_rad)
	direction_vector_p2 = np.array([dx2, dy2])

	# Vector from p2 to p1
	vector_p2_to_p1 = np.array([x1 - x2, y1 - y2])

	# Calculate the projection of vector_p2_to_p1 onto direction_vector_p2
	t = np.dot(vector_p2_to_p1, direction_vector_p2)

	# Calculate the intersection point (the projection of p1 onto the line defined by p2 and its heading)
	intersection_point = np.array([x2, y2]) + t * direction_vector_p2
	# print(intersection_point)

	mid_p = [(2*intersection_point[0]+2*pts[2][0])/4, (2*intersection_point[1]+2*pts[2][1])/4]
	p2p5 = [mid_p[0], mid_p[1], pts[2][2]]
	# pts = pts[:2]+[p2p5]+[pts[-1]]
	# print(pts)
	# exit()

	## generate smooth path ##
	x_coords = np.array([p[0] for p in pts])
	y_coords = np.array([p[1] for p in pts])
	final_orientations_deg = np.array([p[2] for p in pts])

	# --- Calculate the *path direction* at each point for reverse parking ---
	path_orientations_deg = final_orientations_deg + 180

	# Normalize angles to be within [0, 360) for consistency
	path_orientations_deg = np.fmod(path_orientations_deg, 360)
	path_orientations_deg[path_orientations_deg < 0] += 360

	# Convert path orientations from degrees to radians for trigonometric functions
	path_orientations_rad = np.deg2rad(path_orientations_deg)

	 # You can adjust this value to control curve tightness

	tangent_x_components = tangent_scale * np.cos(path_orientations_rad)
	tangent_y_components = tangent_scale * np.sin(path_orientations_rad)

	# --- Parameterize points for the spline ---
	# A simple parameterization: t values correspond to the index of the point.
	t_params = np.arange(len(pts))

	# --- Create CubicHermiteSpline objects ---
	spline_x = CubicHermiteSpline(t_params, x_coords, tangent_x_components)
	spline_y = CubicHermiteSpline(t_params, y_coords, tangent_y_components)

	# --- Generate dense points along the path ---
	# Create a dense set of 't' values across the full range of the spline.
	num_new_points = granularity # Number of points to generate for the smooth path
	t_new = np.linspace(t_params.min(), t_params.max(), num_new_points)

	# Evaluate the spline functions at these new 't' values to get the smooth path coordinates.
	x_new = spline_x(t_new)
	y_new = spline_y(t_new)

	dx_dt_new = spline_x(t_new, 1) # First derivative of x(t)
	dy_dt_new = spline_y(t_new, 1) # First derivative of y(t)

	h_new_rad = np.arctan2(dy_dt_new, dx_dt_new)

	# 3. Convert radians to degrees
	h_new_deg = np.rad2deg(h_new_rad)

	# --- Plotting the results for visualization ---
	# plt.figure(figsize=(10, 8))
	plt.clf()
	# Plot the original input points
	plt.scatter(x_coords, y_coords, color='blue', zorder=5, s=100, label='Key stop points (P)')
	for i in range(len(pts)):
		plt.text(x_coords[i] + 0.15, y_coords[i] + 0.15, f'P{i+1}', color='blue', fontsize=12)
	   
	# Plot the generated smooth, curved path
	plt.plot(x_new, y_new, color='green', linestyle='-', linewidth=2, label='Generated curved Path')


	# Normalize the angles to be within the [0, 360) degree range
	# np.fmod handles the modulo operation for potentially wrapping around (e.g., 370 becomes 10)
	reverse_loc_h = h_new_deg+180
	reverse_loc_h = np.fmod(reverse_loc_h, 360)
	loc_x = x_new
	loc_y = y_new
	loc_h = reverse_loc_h

	angles_rad = np.deg2rad(loc_h)
	dx = np.cos(angles_rad)
	dy = np.sin(angles_rad)


	# Plot scatter and overlay arrows using quiver
	plt.quiver(loc_x, loc_y, dx, dy, angles='xy', scale_units='xy', scale=1, color='red')

	plt.title('Reverse Parking Path')
	plt.xlabel('X Coordinate (meter)')
	plt.ylabel('Y Coordinate (meter)')
	plt.grid(True)
	# plt.xlim((-10, 10))
	# plt.ylim((-10, 10))
	# plt.axis('equal') # Ensures equal scaling for x and y axes, important for spatial accuracy
	plt.legend()
	plt.show()

	new_path =[[round(x,3),round(y,3),round(h,3)] for x,y,h in zip(loc_x, loc_y, loc_h)]
	print("smooth path", new_path)

	return new_path


f = open("path_localx3_with_head.txt", "r")
pts = json.loads(f.readline())
f.close()
print(pts)

curve_scale = 1.0*5
granularity = 10
# unit [m, m, degree]
# new_path = smooth_path_with_heading(pts, curve_scale, granularity)

# with open('path_localxn_with_head.txt', 'w', encoding='utf-8') as f:
#	 json.dump(new_path, f, indent=2)
# f.close()
# exit()


## smooth gps

# Your original path data
'''
f = open("path_copy.txt", "r")
pts = json.loads(f.readline())
f.close()
original_path = np.array(pts)

# Separate x and y coordinates
x_original = original_path[:, 0]
y_original = original_path[:, 1]

plt.plot(x_original, y_original)
plt.show()
# exit()


x_original_key = [original_path[0][0], original_path[10][0], original_path[26][0]] 
y_original_key = [original_path[0][1], original_path[10][1], original_path[26][1]]
print("key gps x, y array", x_original_key, y_original_key)


f = open("path_localx3_with_head_copy.txt", "r")
pts_deg = json.loads(f.readline())
h_original_key = [(v[2]+90)%360 for v in pts_deg]
f.close()

## larger, smoothier
scale = 100000

key_pts = [[x*scale,y*scale,z] for x,y,z in zip(x_original_key, y_original_key, h_original_key)]
print('kpp',key_pts)

## also larger smoothier
curve_scale = 1.0*10
granularity = 100
smooth_keys = smooth_path_with_heading_no_p3(key_pts, curve_scale, granularity)

smooth_keys_lat_lon = [[x[0]/scale, x[1]/scale, x[2]] for x in smooth_keys]

origin_latitude = 42.309130 # degrees
origin_longitude = 83.024240 # degrees
gps_offset = [round(origin_latitude-smooth_keys_lat_lon[0][0],6), round(origin_longitude-smooth_keys_lat_lon[0][1], 6)]

smooth_keys_lat_lon = [[x[0]+gps_offset[0], x[1]+gps_offset[1], x[2]] for x in smooth_keys_lat_lon]
smooth_keys_lat_lon_in_rad = [[x[0], x[1], x[2]/180*3.1415926] for x in smooth_keys_lat_lon]

print(smooth_keys_lat_lon)
print(smooth_keys_lat_lon_in_rad)

# with open('path_gps_x100_with_head_in_degree.txt', 'w', encoding='utf-8') as f:
#	 json.dump(smooth_keys_lat_lon, f, indent=2)
# f.close()
'''


'''
f = open("path_localx3_with_head_copy.txt", "r")
pts_deg = json.loads(f.readline())
f.close()


curve_scale = 7
granularity = 100
print(pts_deg)
# smooth_keys = smooth_path_with_heading_no_p3(pts_deg, curve_scale, granularity)
smooth_keys = smooth_path_with_heading(pts_deg, curve_scale, granularity)

print("local_smooth_path", smooth_keys)


origin_latitude = 42.309130 # degrees
origin_longitude = 83.024240 # degrees

smooth_keys_gps_deg = []
smooth_keys_gps_rad = []
smooth_keys_loc_deg = []
smooth_keys_loc_rad = []

for key in smooth_keys:
	local_x = key[0]
	local_y = key[1]
	h_deg = key[2]
	h_rad = h_deg*3.1415926/180
	gps_coord = local_to_gps(local_x, local_y, origin_latitude, origin_longitude)
	smooth_keys_gps_deg.append([gps_coord[0], gps_coord[1], h_deg])
	smooth_keys_gps_rad.append([gps_coord[0], gps_coord[1], h_rad])
	smooth_keys_loc_deg.append([local_x, local_y, h_deg])
	smooth_keys_loc_rad.append([local_x, local_y, h_rad])

print("gps_smooth_path", smooth_keys_loc_deg)
sm_path = {
	"path_loc_deg":smooth_keys_loc_deg, 
	"path_loc_rad":smooth_keys_loc_rad, 
	"path_gps_deg":smooth_keys_gps_deg, 
	"path_gps_rad":smooth_keys_gps_rad
}

# fn = "path_p100_curve_1.txt"
fn = "path_p%d_curve_%d.txt"%(granularity,curve_scale)
with open(fn, 'w', encoding='utf-8') as f:
	json.dump(sm_path, f, indent=2)
f.close()
'''


### real parking gps

def smooth_path_with_heading_no_p3_gps(pts, tangent_scale, granularity):

	## show original key stops
	loc_x = [x[0] for x in pts]
	loc_y = [x[1] for x in pts]
	loc_h = [x[2] for x in pts]

	# angles_rad = np.deg2rad(loc_h)
	angles_rad = np.deg2rad(loc_h)
	dx = np.cos(angles_rad)
	dy = np.sin(angles_rad)


	##### add mid point p2.5 ######
	# Extract coordinates and heading
	x1, y1, heading1 = pts[1]
	x2, y2, heading2 = pts[2]

	# Convert p2's heading to radians
	heading2_rad = np.deg2rad(heading2)

	# Define the direction vector for p2's heading
	dx2 = np.cos(heading2_rad)
	dy2 = np.sin(heading2_rad)
	direction_vector_p2 = np.array([dx2, dy2])

	# Vector from p2 to p1
	vector_p2_to_p1 = np.array([x1 - x2, y1 - y2])

	# Calculate the projection of vector_p2_to_p1 onto direction_vector_p2
	t = np.dot(vector_p2_to_p1, direction_vector_p2)
	
	## generate smooth path ##
	x_coords = np.array([p[0] for p in pts])
	y_coords = np.array([p[1] for p in pts])
	final_orientations_deg = np.array([p[2] for p in pts])

	# --- Calculate the *path direction* at each point for reverse parking ---
	path_orientations_deg = final_orientations_deg 

	# Normalize angles to be within [0, 360) for consistency
	path_orientations_deg = np.fmod(path_orientations_deg, 360)
	path_orientations_deg[path_orientations_deg < 0] += 360

	# Convert path orientations from degrees to radians for trigonometric functions
	path_orientations_rad = np.deg2rad(path_orientations_deg)

	 # You can adjust this value to control curve tightness

	tangent_x_components = tangent_scale * np.cos(path_orientations_rad)
	tangent_y_components = tangent_scale * np.sin(path_orientations_rad)

	# --- Parameterize points for the spline ---
	# A simple parameterization: t values correspond to the index of the point.
	t_params = np.arange(len(pts))

	# --- Create CubicHermiteSpline objects ---
	spline_x = CubicHermiteSpline(t_params, x_coords, tangent_x_components)
	spline_y = CubicHermiteSpline(t_params, y_coords, tangent_y_components)

	# --- Generate dense points along the path ---
	# Create a dense set of 't' values across the full range of the spline.
	num_new_points = granularity # Number of points to generate for the smooth path
	t_new = np.linspace(t_params.min(), t_params.max(), num_new_points)

	# Evaluate the spline functions at these new 't' values to get the smooth path coordinates.
	x_new = spline_x(t_new)
	y_new = spline_y(t_new)

	dx_dt_new = spline_x(t_new, 1) # First derivative of x(t)
	dy_dt_new = spline_y(t_new, 1) # First derivative of y(t)

	h_new_rad = np.arctan2(dy_dt_new, dx_dt_new)

	# 3. Convert radians to degrees
	h_new_deg = np.rad2deg(h_new_rad)

	# --- Plotting the results for visualization ---
	# plt.figure(figsize=(10, 8))
	plt.clf()
	# Plot the original input points
	plt.scatter(x_coords, y_coords, color='blue', zorder=5, s=100, label='Key stop points (P)')
	for i in range(len(pts)):
		plt.text(x_coords[i] + 0.15, y_coords[i] + 0.15, f'P{i+1}', color='blue', fontsize=12)
	   
	# Plot the generated smooth, curved path
	plt.plot(x_new, y_new, color='green', linestyle='-', linewidth=2, label='Generated curved Path')


	# Normalize the angles to be within the [0, 360) degree range
	# np.fmod handles the modulo operation for potentially wrapping around (e.g., 370 becomes 10)
	reverse_loc_h = h_new_deg+180
	reverse_loc_h = np.fmod(reverse_loc_h, 360)
	loc_x = x_new
	loc_y = y_new
	loc_h = reverse_loc_h

	angles_rad = np.deg2rad(loc_h)
	dx = np.cos(angles_rad)
	dy = np.sin(angles_rad)


	# Plot scatter and overlay arrows using quiver
	plt.quiver(loc_x, loc_y, dx, dy, angles='xy', scale_units='xy', scale=1, color='red')

	plt.title('Reverse Parking Path')
	plt.xlabel('X Coordinate (meter)')
	plt.ylabel('Y Coordinate (meter)')
	plt.grid(True)
	# plt.xlim((-10, 10))
	# plt.ylim((-10, 10))
	# plt.axis('equal') # Ensures equal scaling for x and y axes, important for spatial accuracy
	plt.legend()
	plt.show()

	new_path =[[round(x,3),round(y,3),round(h,3)] for x,y,h in zip(loc_x, loc_y, loc_h)]
	print("smooth path", new_path)

	return new_path



# import pandas as pd

from scipy.interpolate import UnivariateSpline



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

'''
## input lat/lon/heading
start0 = [42.5201616416335, -83.04408180066, -1.5]
start = [42.520182,-83.043624, -1.6]
keystop1 = [42.52019020702132, -83.04346168969953, -1.6]
keystop2 = [42.52026112792775, -83.04338016649136, -3.1416]
# target = [42.52033598879717, -83.04338551227549, -3.1416]
target = [42.5203530204544, -83.0433692649389, -3.1416]
pts_deg = np.array([start0, start, keystop1, keystop2, target])
# pts_deg = np.array([start, keystop1, keystop2])
# pts_deg = np.array([keystop1, keystop2])

tot_point = 100
lat_s, lon_s, h_s, x_s, y_s = path_gen_for_eh(pts_deg, tot_point)


offset = 5
sel = int(tot_point*3/4 + offset)
lat_s = lat_s[:sel]
lon_s = lon_s[:sel]
h_s = h_s[:sel]
x_s = x_s[:sel]
y_s = y_s[:sel]

## write offline path file
llh = [[a,b,c] for a,b,c in zip(lat_s, lon_s, h_s)]
f = open("path2.csv","w")
f.write("latitude,longitude,heading\n")
for ele in llh:
	# print(ele[0],", ",ele[1])
	f.write("%f,%f,%f\n"%(ele[0],ele[1],ele[2]))
f.close()

## plot in meter
plt.figure(figsize=(6, 6))
plt.plot(x_s, y_s, '-', label='Smoothed path')
# plt.scatter(xyh[:, 0], xyh[:, 1], c='red', marker='o', label='Original pts')
for i in range(0, len(x_s), 5):
	dx = np.cos(h_s[i]+3.1416) * 2
	dy = np.sin(h_s[i]+3.1416) * 2
	plt.arrow(x_s[i], y_s[i], dy, dx, head_width=0.5, color='green')
plt.axis('equal')
plt.legend()
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Smoothed Trajectory with Heading")
plt.show()
'''




# realtime data
f = open("hummer_path/pathx5.txt", "r")
data = f.readline()
f.close()
	
path_gps_with_heading = json.loads(data)
print(path_gps_with_heading)

## adjust to testing parking spot location
# target_latlon = [42.52033598879717, -83.04338551227549]
target_latlon = [42.5203530204544, -83.0433692649389] ## calibrate to where the spot is
offset_lat = target_latlon[0]-path_gps_with_heading[4][0]
offset_lon = target_latlon[1]-path_gps_with_heading[4][1]

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
f = open("hummer_path/path2_smooth.csv","w")
f.write("latitude,longitude,heading\n")
for ele in llh:
	# print(ele[0],", ",ele[1])
	f.write("%f,%f,%f\n"%(ele[0],ele[1],ele[2]))
f.close()

## plot in meter
plt.figure(figsize=(6, 6))
plt.plot(x_s, y_s, '-', label='Smoothed path')
# plt.scatter(xyh[:, 0], xyh[:, 1], c='red', marker='o', label='Original pts')
for i in range(0, len(x_s), 5):
	dx = np.cos(h_s[i]) * 2
	dy = np.sin(h_s[i]) * 2
	plt.arrow(x_s[i], y_s[i], dy, dx, head_width=0.5, color='green')
plt.axis('equal')
plt.legend()
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Smoothed Trajectory with Heading")
plt.show()







