import os
import json
import numpy as np
from app_iphone import find_sta_turn_p_bri, rel_loc_ana_and_path_inter_simple, can_heading_cali
import math

def rotate_by_0(deg):

	fn = "pathx5_can_heading"
	with open("hummer_path/%s.txt"%fn, "r") as f:
		message = json.loads(f.readline())

	pts = message
	theta = np.deg2rad(deg)

	theta_deg = deg
	theta = math.radians(theta_deg)

	x0, y0, _ = pts[0]




   # ## 
# 	out = [pts[0]]
# 	for x, y, h in pts[1:]:
# 	    dx = x0 - x
# 	    dy = y0 - y
# 	    d = math.hypot(dx, dy)

# 	    if d <= 1.0:
# 	        xn, yn = x0, y0
# 	    else:
# 	        xn = x + dx / d
# 	        yn = y + dy / d

# 	    out.append([xn, yn, h])

# 	print(out)



	rotated = []
	for x, y, h in pts:
	    dx = x - x0
	    dy = y - y0

	    xr = x0 + dx * math.cos(theta) - dy * math.sin(theta)
	    yr = y0 + dx * math.sin(theta) + dy * math.cos(theta)

	    hr = (h - theta_deg) % 360   # or just h - 1.0 if you don't want wrapping

	    rotated.append([xr, yr, hr])

	print(rotated)

rotate_by_0(3)




def cali_update_path():

	os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p2.txt")
	os.system("cp ./hummer_path/can_heading.txt ./hummer_path/can_heading_ref.txt")

	
	fn = "pathx5"
	## change both pathx5 and pathx5_can_heading if do it offline
	with open("hummer_path/%s.txt"%fn, "r") as f:
		message = json.loads(f.readline())

	
	## find best turnning to calibrate instead of x/y
	print(message)
	message[0][2] -= 2
	message[-1][2] -= 2

	p2 = find_sta_turn_p_bri(np.array(message))
	message[1] = p2

	## update pathx5
	with open('hummer_path/%s.txt'%fn, 'w') as f:
		json.dump(message, f)

	## interpulation
	fn1 = "path_local_den_1_stage"
	fn2 = "path_local_den_2_stage"
	stage_resolution = 100
	rel_loc_ana_and_path_inter_simple(fn, stage_resolution, fn1, fn2, "forward")
	
	## coordinate transform based on calibration
	can_heading_cali()
	


# cali_update_path()


	
