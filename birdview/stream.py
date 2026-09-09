# ffplay /dev/video0

import cv2
import numpy as np


def edge_det(frame):
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5,5), 0)
	edges = cv2.Canny(blurred, 50, 150)
	lines = cv2.HoughLines(edges, 1, np.pi/180, 200)
	return edges, lines

def find_LR(lines):
	ll = []
	rl = []
	## 45 degree
	det_slop_thre_left = -1
	det_slop_thre_right = 1 
	if lines is not None:
		for line in lines:
			rho, theta = line[0]
			if np.sin(theta)!=0:
				m = -np.cos(theta)/np.sin(theta)
				b = rho/np.sin(theta)
				if m>det_slop_thre_right:
					rl.append([m,b])
				elif m<det_slop_thre_left:
					ll.append([m,b])
				# print("line equ (y=%.2fx+%.2f):"%(m,b))
			else:
				pass
				# print("line equ (vertical at %f)"%rho)
	print("left lanes", ll, "right lanes", rl)

	if len(ll)==0 or len(rl)==0:
		print("Left/Right lane not detected")
	else:
		pass


def find_LR2(lines, width, height):
	close_left = None
	close_right = None
	img_center = width//2
	eval_y = height//2
	max_x_left = -float('inf')
	min_x_right = float('inf')

	angle_thr_deg = 70
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

	# print("left", close_left, "right", close_right)


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


cap = cv2.VideoCapture(0)

if not cap.isOpened():
	print("no camera found")
	exit()

while True:
	ret, frame = cap.read()

	if not ret:
		print("stream reading fail")
		break

	height,width = frame.shape[:2]
	
	# print(height, width)
	# # 480, 640
	# exit()

	edges, lines = edge_det(frame)

	
	# find_LR(lines)
	ll,rl,mid_p,err_x, err_h = find_LR2(lines, width, height)

	for best_line, color in [(ll, (0,255,0)), (rl, (0,0,255))]:
		if best_line:
			r,t = best_line
			a,b = np.cos(t), np.sin(t)
			x0,y0 = a*r, b*r
			pt1 = (int(x0+1000*(-b)), int(y0+1000*(a)))
			pt2 = (int(x0-1000*(-b)), int(y0-1000*(a)))
			cv2.line(frame, pt1, pt2, color, 3)
	
	if mid_p:
		cv2.circle(frame, (int(mid_p), height//2), 10, (255,0,0), -1)
		cv2.circle(frame, (width//2, height//2), 10, (0,0,0), -1)
		cv2.putText(frame, "X offset:%.2f"%err_x, (30,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
		cv2.putText(frame, "Err Heading:%.2f"%err_h, (30,80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
	
	cv2.imshow('stream', frame)
	# cv2.imshow('edge Det', edges)

	if cv2.waitKey(1) & 0xFF==ord('q'):
		break


cap.release()
cv2.destroyAllWindows()