import os
import json
import numpy as np
from app_iphone import find_sta_turn_p_bri, rel_loc_ana_and_path_inter_simple, can_heading_cali

def cali_update_path():

	os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p2.txt")
	os.system("cp ./hummer_path/can_heading.txt ./hummer_path/can_heading_ref.txt")

	
	fn = "pathx5"
	## change both pathx5 and pathx5_can_heading if do it offline
	with open("hummer_path/%s.txt"%fn, "r") as f:
		message = json.loads(f.readline())

	
	## find best turnning to calibrate instead of x/y
	print(message)
	message[0][2] -= 1
	message[-1][2] -= 1

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
	


cali_update_path()


	
