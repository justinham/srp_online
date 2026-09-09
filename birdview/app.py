
#import matplotlib.pyplot as plt
from flask import Flask, render_template, request, jsonify
import json
# from simpleGPS import *

from pynmeagps import NMEAReader
import math
import os

import ast
import numpy as np

import subprocess
import sys
import signal
import time
from datetime import datetime
from geopy.distance import distance
from geopy.point import Point

import logging
from app_iphone import rel_loc_ana_and_path_inter_simple, generate_path, find_sta_turn_p_bri, can_heading_cali, get_average_coordinates
# from trailer_math import estimate_trailer_pos




# Suppress all HTTP request logs (including 200 responses)
log = logging.getLogger('werkzeug')
log.setLevel(logging.WARNING)

heading_log = []
gps_log = []


# Track subprocesses for ros execution
processes = []
processes_bg = []


USE_CAN_HEAD = True
# center_off_distance_ublox = 1.9
center_off_distance_ublox = 0.0
center_off_distance_can_gps_head_back = 0.0
center_off_distance_can_gps_left_right = 0.0


## veh 2.196, 4.999, 1.976, anchor 1.5m front/rear to center
veh_width = 2.196
veh_len = 4.999
senosor_front_offset = 0.5
senosor_back_offset = -0.5
a1 = [-veh_width/2, senosor_front_offset]
a2 = [-veh_width/2, senosor_back_offset]

   
# --- Trailer UWB GEOMETRY CONSTANTS ---
Ra = 1.64/2    # Half-width of trailer tags (2 on trailer)
Rv = 1.64/2  # Half-width of vehicle tags (2 on vehicle)
L_sv = 1.50  # Sensor to Hinge distance (Sensor is 0.5m ahead of hitch)
L_ht = 0.28  # Hinge to 2 Tag Line center distance (Tags are 1.2m behind hitch)

yh = -1.5-0.28   # Hinge position relative to vehicle center
Lt = 1.5+1.5    # Hinge to Trailer center



###########

# 2 anchor 2 tag
def estimate_trailer_pos(d1, d2):
    """
    d1, d2: Sensor distances
    Rv, Ra, alpha_deg: Sensor geometry constants
    yh: Y-coordinate of hinge relative to vehicle center (usually negative)
    Lt: Distance from hinge to trailer center
    """
   
    Ty = Ra # tag to trailer front face center
    Th = L_ht # hinge to trailer front face center (if two tags on trailer)
    alpha = math.atan2(Ty, Th)
    angle_deg = math.degrees(alpha)
    # print("anchor mount angle", angle_deg)
    
    
    # 1. Estimate Articulation Angle (Theta)
    # Using your derived logic: d1 is Left, d2 is Right. 
    # If d1 > d2, trailer is swinging Right (+X).
    denominator = 4 * Rv * Ra * np.sin(alpha)
    sin_theta = (d1**2 - d2**2) / denominator
    sin_theta = np.clip(sin_theta, -1.0, 1.0)
    theta_rad = np.arcsin(sin_theta)
    
    # 2. Calculate Trailer Position (relative to vehicle center 0,0)
    # Theta=0 is straight back (along -Y axis)
    trailer_x = Lt * np.sin(theta_rad)
    trailer_y = yh - Lt * np.cos(theta_rad)
    
    return -np.degrees(theta_rad), (trailer_x, trailer_y)


# 1 anchor 2 tag

# 1 anchor 2 tag (tag on trailer)
def estimate_trailer_pos_single_sensor(d1, d2):

    # (d1:left sen, d2:right sen)

    # 1. Total effective lever arm
    # The rotation gain depends on the total distance from sensor to tags
    denominator = 4 * Ra * L_sv
    
    # 2. Solve for Theta
    sin_theta = (d1**2 - d2**2) / denominator
    sin_theta = np.clip(sin_theta, -1.0, 1.0)
    theta_rad = np.arcsin(sin_theta)
    
    # 3. Calculate Trailer Center in World/Vehicle Local Space
    # (Using the hinge 'yh' as the pivot point)
    tx = Lt * np.sin(theta_rad)
    ty = yh - Lt * np.cos(theta_rad)
    
    return np.degrees(theta_rad), (tx, ty)


# 1 anchor 2 tag (tag on veh)
def estimate_trailer_pos_dual_vehicle_sensors(d1, d2):
    
    # (d1:right sen, d2:left sen)
    
    # 1. Total effective lever arm
    # The gain now depends on the vehicle sensor width (Rv)
    denominator = 4 * Rv * L_sv
    
    # 2. Solve for Theta
    # We swap d1 and d2 here to maintain: Right Turn = Positive Angle
    sin_theta = (d2**2 - d1**2) / denominator
    sin_theta = np.clip(sin_theta, -1.0, 1.0)
    theta_rad = np.arcsin(sin_theta)
    
    # 3. Calculate Trailer Position
    tx = Lt * np.sin(theta_rad)
    ty = yh - Lt * np.cos(theta_rad)
    
    return np.degrees(theta_rad), (tx, ty)



def get_trailer_world_state_nav(v_x, v_y, v_h, t_local_x, t_local_y, t_rel_h):
    """
    v_h: 0 degrees is North (+Y), 90 degrees is East (+X)
    t_local_x: Right/Left offset relative to vehicle
    t_local_y: Forward/Backward offset relative to vehicle
    """
    # 1. World Heading (Degrees)
    t_world_h = (v_h + t_rel_h) % 360
    
    # 2. World Position
    rad = np.radians(v_h)
    sin_h = np.sin(rad)
    cos_h = np.cos(rad)
    
    # Standard Navigation Rotation Matrix for [X=90, Y=0]
    # x_world = v_x + (local_x * cos_h + local_y * sin_h)
    # y_world = v_y + (-local_x * sin_h + local_y * cos_h)
    
    world_dx = t_local_x * cos_h + t_local_y * sin_h
    world_dy = -t_local_x * sin_h + t_local_y * cos_h
    
    t_world_x = v_x + world_dx
    t_world_y = v_y + world_dy
    
    return t_world_x, t_world_y, t_world_h




def launch_process(cmd, shell=False):

    # password = "ConnAu\n"
    # p = subprocess.run(cmd, shell=True, executable="/bin/bash", input=password.encode(), check=True)
    p = subprocess.Popen(cmd, shell=shell, preexec_fn=os.setsid)

    processes.append(p)

def launch_process_ps(cmd, shell=False):

    password = "ConnAu\n"
    p = subprocess.run(cmd, shell=True, executable="/bin/bash", input=password.encode(), check=True)
    # p = subprocess.Popen(cmd, shell=shell, preexec_fn=os.setsid)

    processes.append(p)


def launch_process_bg(cmd, shell=False):

    # password = "ConnAu\n"
    # p = subprocess.run(cmd, shell=True, executable="/bin/bash", input=password.encode(), check=True)
    p = subprocess.Popen(cmd, shell=shell, preexec_fn=os.setsid)

    processes_bg.append(p)


def cleanup():
    print("\nStopping all ROS nodes and scripts...")
    for p in processes:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except Exception as e:
            print(f"Error stopping process: {e}")


def cleanup_emg():
    result = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True)
    nodes = result.stdout.strip().splitlines()

    for node in nodes:
        print(f"Killing node: {node}")
        subprocess.run(["ros2", "node", "kill", node])





def circle_intersection(a1, r1, a2, r2):
    x0, y0 = a1
    x1, y1 = a2
    dx = x1 - x0
    dy = y1 - y0
    d = math.hypot(dx, dy)

    # No solution: the circles do not intersect
    if d > r1 + r2 or d < abs(r1 - r2) or d == 0:
        return None

    # Point 2 is the midpoint between the intersection points
    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = math.sqrt(r1**2 - a**2)
    xm = x0 + a * dx / d
    ym = y0 + a * dy / d

    # Offset of the intersection points from point 2
    rx = -dy * (h / d)
    ry = dx * (h / d)

    # Two intersection points
    p1 = [xm + rx, ym + ry]
    p2 = [xm - rx, ym - ry]

    ## p1 left (x<0), p2 right (x>0)
    return p1, p2


def rel_turning_direction(start_h, end_h):
    direction = "same"
    delta = (end_h -start_h + 360) % 360

    if delta <= 180:
        direction = "clockwise"
    elif delta > 180:
        direction = "counter-clockwise"
    return direction
    

def rotate_to_right(p, r_deg):
    x, y = p
    r_rad = math.radians(r_deg)

    # Swap x and y for "y is 0°" orientation, invert x to handle clockwise
    x_rot =  x * math.sin(r_rad) + y * math.cos(r_rad)
    y_rot = -x * math.cos(r_rad) + y * math.sin(r_rad)
    return [x_rot, y_rot]

def rotate_to_right2(point, pivot, r_deg):
    px, py = pivot   # pivot point around which to rotate
    x, y = point     # the point to rotate
    
    # Translate so pivot is at the origin
    x -= px
    y -= py
    
    # Convert degrees to radians
    r_rad = math.radians(r_deg)
    
    # Apply rotation
    x_rot = x * math.cos(r_rad) + y * math.sin(r_rad)
    y_rot = -x * math.sin(r_rad) + y * math.cos(r_rad)
    
    # Translate back
    x_rot += px
    y_rot += py
    
    return [x_rot, y_rot]


def transform_vector(v, theta_deg):
    """
    Transform a vector from coordinate system c1 to c2,
    where angles are measured clockwise from the +y axis.
    theta_deg = angle from c1 to c2 (positive = clockwise)
    """
    x, y = v
    
    # Convert to standard math convention:
    # +y (0°) clockwise → +x (0°) counter-clockwise
    theta_math = math.radians(-theta_deg)  # negate for clockwise
     
    # Standard 2D rotation (axes rotation, coordinate transformation)
    x2 =  x * math.cos(theta_math) + y * math.sin(theta_math)
    y2 = -x * math.sin(theta_math) + y * math.cos(theta_math)
    
    return [x2, y2]


def get_veh_loc_UWB(a1, a2, d1, d2, cam_loc, can_ref_heading, can_cur_heading):
    
    points = circle_intersection(a1, d1, a2, d2)
    # print("pp", points)
    if not points is None:
        left_loc = points[0]
        if points[1][0]<points[0][0]:
            left_loc = points[1]


        # print("Intersection Points:", left_loc)

        ## to initial 
        # theta_deg = -(can_ref_heading-can_cur_heading)%360
        
        ## to sensed north
        theta_deg = -(can_cur_heading-90)

        # vec1 = (-left_loc[0], -left_loc[1])
        # vec2 = transform_vector(vec1, theta_deg)
        # real_veh_loc = [cam_loc[0]+vec2[0], cam_loc[1]+vec2[1]]

        # print("deb", vec1, vec2, theta_deg)

        # print(can_ref_heading,can_cur_heading, theta_deg)

        left_loc_veh_heading = rotate_to_right(left_loc, theta_deg)
        # left_loc_veh_heading = rotate_to_right2(left_loc, left_loc, theta_deg) # use phone as center to rotate
        # print("Intersection Points rotate:", left_loc_veh_heading)

        rev_vec = [-1*left_loc_veh_heading[0], -1*left_loc_veh_heading[1]]
        # print("reverse from intersection to veh", rev_vec, theta_deg)

        real_veh_loc = [cam_loc[0]+rev_vec[0], cam_loc[1]+rev_vec[1]]
        

        anc1_loc_veh_heading = rotate_to_right(a1, theta_deg)
        anc2_loc_veh_heading = rotate_to_right(a2, theta_deg)
        # anc1_loc_veh_heading = rotate_to_right2(a1, left_loc, theta_deg)
        # anc2_loc_veh_heading = rotate_to_right2(a2, left_loc, theta_deg)
        a1x = real_veh_loc[0]+anc1_loc_veh_heading[0]
        a1y = real_veh_loc[1]+anc1_loc_veh_heading[1]
        a2x = real_veh_loc[0]+anc2_loc_veh_heading[0]
        a2y = real_veh_loc[1]+anc2_loc_veh_heading[1]
        
        # print(real_veh_loc[0], real_veh_loc[1], left_loc[0], left_loc[1], a1x, a1y, a2x, a2y)

        # print("est veh at ", real_veh_loc)
        return [real_veh_loc[0], real_veh_loc[1], left_loc[0], left_loc[1], a1x, a1y, a2x, a2y] 

    else:
        # print("no intersection found")
        return None








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



def loc_meters_to_gps(lat0, lon0, x, y):
    R = 6378137  # Earth's radius in meters

    # Convert starting latitude to radians
    lat0_rad = math.radians(lat0)

    # Calculate latitude
    dlat = y / R
    lat1 = lat0 + math.degrees(dlat)

    # Calculate longitude
    dlon = x / (R * math.cos(lat0_rad))
    lon1 = lon0 + math.degrees(dlon)

    return lat1, lon1


def move_gps_point(lat0, lon0, heading_deg, distance_m):
    """
    Move from (lat0, lon0) in the heading direction by distance_m meters.
    """
    R = 6378137  # Earth's radius in meters

    heading_rad = math.radians(heading_deg)
    lat0_rad = math.radians(lat0)

    # Compute deltas in meters
    dx = distance_m * math.sin(heading_rad)
    dy = distance_m * math.cos(heading_rad)

    # Convert meters to degrees
    dlat = dy / R
    dlon = dx / (R * math.cos(lat0_rad))

    # Apply changes
    lat1 = lat0 + math.degrees(dlat)
    lon1 = lon0 + math.degrees(dlon)

    return lat1, lon1


## keep reading heading till point 2
def log_execution():

    with open("hummer_path/can_heading.txt", "r") as f:
        line = f.readline()
    heading0 = json.loads(line)[2]
    heading_log.append(heading0)

    with open("hummer_path/gps.txt", "r") as f:
        line = f.readline()
    gps = json.loads(line)
    gps_log.append(gps)


    

def dynamic_opt_path(dir):
    with open("hummer_path/pathx5_can_heading.txt", "r") as f:
        line = f.readline()
    points = json.loads(line)

    with open("hummer_path/pathx5_can_heading_cp.txt", "w") as f:
        json.dump(points, f)

    start = points[2]
    end = points[3]
    extend = points[4]

    ## move west 1m
    if dir=="left":
        end[0] -= 1
        extend[0] -= 1
    elif dir=="right":
        end[0] += 1
        extend[0] += 1
        
    points = [points[0], points[1], start, end, extend]

    with open("hummer_path/pathx5_can_heading.txt", "w") as f:
        json.dump(points, f)



    


################ 

app = Flask(__name__, static_folder='static')

@app.route('/')
def home():
#    return render_template('index.html')
    return render_template('index3D.html')

@app.route('/test2')
def test2():
#    return render_template('index.html')
    return render_template('test2.html')

## phone execution monitor UI
@app.route('/phone/')
def home_phone():
#    return render_template('index.html')
    return render_template('index_phone.html')


@app.route('/path_dyn_l')
def path_update1():
    dynamic_opt_path("left")
    return jsonify({'status': 'success', 'echo': "path_update"}), 200


@app.route('/path_dyn_r')
def path_update2():
    dynamic_opt_path("right")
    return jsonify({'status': 'success', 'echo': "path_update"}), 200


# avoid double tag to open two commander processes
@app.route('/start_exc', methods=['POST'])
def start_exc():
    # print("aaa")

    with open("./hummer_path/gps_log.txt", 'w') as f:
        f.write(str("--start--\n"))
    with open("./hummer_path/heading_log.txt", 'w') as f:
        f.write(str("--start--\n"))
    print("start execution")

    file_name = './hummer_path/gps_history_before_start.txt'
    with open(file_name, 'a') as f:
        f.write(str("--start execution--\n"))

    
    fn = "execution_tag"
    fn2 = "execution_tag_trailer"
    with open("./hummer_path/%s.txt"%fn, 'r') as f:
        cur_state = f.readline()
    with open("./hummer_path/%s.txt"%fn2, 'r') as f:
        cur_state_trailer = f.readline()
    
    tag = "success"

    if cur_state=="1":
        tag = "already start"
        return jsonify({'status': tag, 'echo': "start"}), 200


    else:
        fn = "execution_tag"
        with open("./hummer_path/%s.txt"%fn, 'w') as f:
            f.write(str("1"))

       
        ## auto start execution
        try:
            os.chdir("../../ConnAu/ros_ws")

            try:
            
                # start execution
                # launch_process(["ros2", "run", "fusion_py", "CANGPSReader"])
                # launch_process(["ros2", "run", "fusion_py", "SerialGPSReader"])
                launch_process(["ros2", "run", "fusion_py", "VehicleCommander"])

                print("\n starting VehicleCommander.")

                ## optimize heading based on start point
                # profile_heading()
                

            except:
                print("\n starting VehicleCommander fail.")
                tag = 'fail'

            finally:
                os.chdir("../../srp/birdview")
                return jsonify({'status': tag, 'echo': "start"}), 200
    
        
        except:
            print("ros_ws folder not exist")


    return jsonify({'status': 'success', 'echo': 'start'}), 200





@app.route('/stop_exc', methods=['POST'])
def stop_exc():
    fn = "execution_tag"
    with open("./hummer_path/%s.txt"%fn, 'w') as f:
        f.write(str("0"))

    file_name = './hummer_path/gps_history_before_start.txt'
    with open(file_name, 'a') as f:
        f.write(str("--stop execution--\n"))

    ## auto stop
    cleanup()

    # global gps_log
    # global heading_log
    # print("gps log:", gps_log)
    # print("heading log:", heading_log)
    # gps_log = []
    # heading_log = []

    with open("./hummer_path/gps_log.txt", 'a') as f:
        f.write("--stop--\n")
    with open("./hummer_path/heading_log.txt", 'a') as f:
        f.write("--stop--\n")
    

    return jsonify({'status': 'success', 'echo': 'stop'}), 200


@app.route('/reset_exc', methods=['POST'])
def reset_exc():
    print("trigger reset")
    fn = "execution_tag"
    with open("./hummer_path/%s.txt"%fn, 'w') as f:
        f.write(str("0"))


    ## auto stop
    cleanup()

    return jsonify({'status': 'success', 'echo': 'reset'}), 200




# execution_tag_trailer 0: init; 1: move; 2: stop; 4: ready
@app.route('/start_exc_trailer', methods=['POST'])
def start_exc_trailer():

    next_state = "1"
    cur_state = None
    fn = "execution_tag_trailer"
    desc = None

    print("start trailer")

    with open("./hummer_path/%s.txt"%fn, 'r') as f:
        cur_state = f.readline()    
    
    if cur_state=="0" or cur_state=="3":
        desc = "initial moving"
        with open("./hummer_path/%s.txt"%fn, 'w') as f:
            f.write(next_state)

    elif cur_state=="1":
        desc = "already in moving status"
           
    elif cur_state=="2":
        desc = "resume moving"
        with open("./hummer_path/%s.txt"%fn, 'w') as f:
            f.write(next_state)
    
    # elif cur_state=="3": #emg stop not here
        # desc = "invalid state change"
       
    elif cur_state=="4":
        desc = "invalid state change"
    
           
    return jsonify({'status': desc, 'echo': 'start'}), 200


@app.route('/stop_exc_trailer', methods=['POST'])
def stop_exc_trailer():

    next_state = "2"
    cur_state = None
    desc = None
    print("stop trailer")

    fn = "execution_tag_trailer"
    with open("./hummer_path/%s.txt"%fn, 'r') as f:
        cur_state = f.readline()

    if cur_state=="0":
        desc = "invalid state change"
    
    elif cur_state=="1":
        desc = "stop moving"
        with open("./hummer_path/%s.txt"%fn, 'w') as f:
            f.write(next_state)
           
    elif cur_state=="2":
        desc = "already stopped"
          
    # elif cur_state=="3":
        # desc = "invalid state change"
    
    elif cur_state=="4":
        desc = "invalid state change"

    return jsonify({'status': desc, 'echo': 'stop'}), 200


@app.route('/reset_exc_trailer', methods=['POST'])
def reset_exc_trailer():

    next_state = "0"
    cur_state = None
    desc = None
    print("reset trailer")

    fn = "execution_tag_trailer"
    with open("./hummer_path/%s.txt"%fn, 'r') as f:
        cur_state = f.readline()

    if not cur_state=="4":
        desc = "clearing state"
        with open("./hummer_path/%s.txt"%fn, 'w') as f:
            f.write(next_state)
    
    return jsonify({'status': desc, 'echo': 'stop'}), 200



@app.route('/stop_exc_for_replan', methods=['POST'])
def stop_exc_rep():

    print("cam trigger stop")

    fn = "execution_tag"
    with open("./hummer_path/%s.txt"%fn, 'w') as f:
        f.write(str("0"))

    ## auto stop
    cleanup()

    with open("./hummer_path/gps_log.txt", 'a') as f:
        f.write("--stop rescan point--\n")
    with open("./hummer_path/heading_log.txt", 'a') as f:
        f.write("--stop rescan point--\n")
    

    return jsonify({'status': 'success', 'echo': 'stop'}), 200


@app.route('/start_exc_for_replan', methods=['POST'])
def start_exc_rep():

    print("cam trigger resume")

    with open("./hummer_path/gps_log.txt", 'w') as f:
        f.write(str("--resume--\n"))
    with open("./hummer_path/heading_log.txt", 'w') as f:
        f.write(str("--resume--\n"))

    fn = "execution_tag"
    with open("./hummer_path/%s.txt"%fn, 'r') as f:
        cur_state = f.readline()
    
    tag = "success"

    if cur_state=="1":
        tag = "already start"
        return jsonify({'status': tag, 'echo': "start"}), 200


    else:
        fn = "execution_tag"
        with open("./hummer_path/%s.txt"%fn, 'w') as f:
            f.write(str("1"))

        ## auto start execution
        try:
            os.chdir("../../ConnAu/ros_ws")

            try:
            
                # start execution
                # launch_process(["ros2", "run", "fusion_py", "CANGPSReader"])
                # launch_process(["ros2", "run", "fusion_py", "SerialGPSReader"])
                launch_process(["ros2", "run", "fusion_py", "VehicleCommander"])

                print("\n starting VehicleCommander.")

                ## optimize heading based on start point
                # profile_heading()
                

            except:
                print("\n starting VehicleCommander fail.")
                tag = 'fail'

            finally:
                os.chdir("../../srp/birdview")
                return jsonify({'status': tag, 'echo': "start"}), 200
    
        
        except:
            print("ros_ws folder not exist")

        

    return jsonify({'status': 'success', 'echo': 'start'}), 200



@app.route('/emg_stop', methods=['POST'])
def emg_stop():
    fn = "execution_tag"
    with open("./hummer_path/%s.txt"%fn, 'w') as f:
        f.write(str("0"))

    ## auto stop
    cleanup_emg()

    return jsonify({'status': 'success', 'echo': "stop"}), 200


@app.route('/succ_stop')
def succ_stop():

    dis_thr = 0.3 # 20 testing trigger
    tag = "run"

    # check if close to target
    with open("hummer_path/pathx5_can_heading.txt", "r") as f:
        line = f.readline()
    path_loc = json.loads(line)
    target = path_loc[-1]
    

    with open("hummer_path/gps_ref_p2.txt", "r") as f:
        line = f.readline()
    gps0 = json.loads(line)
    # f.close()

    with open("hummer_path/can_heading_ref.txt", "r") as f:
        line = f.readline()
    heading0 = json.loads(line)[2]
    # f.close()

    gps_v0 = move_gps_point(gps0[0], gps0[1], heading0, center_off_distance_ublox)
    gps_a0 = gps0

    with open("hummer_path/gps.txt", "r") as f:
        line = f.readline()
    gps1 = json.loads(line)
    # f.close()

    with open("hummer_path/can_heading.txt", "r") as f:
        line = f.readline()
    heading1 = json.loads(line)[2]
    # f.close()

    gps_v1 = move_gps_point(gps1[0], gps1[1], heading1, center_off_distance_ublox)
    gps_a1 = gps1

    # print(gps_v0, gps_v1, heading0, heading1)
    x_vcen, y_vcen = gps_to_loc_meters(lat0=gps_v0[0], lon0=gps_v0[1], lat1=gps_v1[0], lon1=gps_v1[1])
    x_ant, y_ant = gps_to_loc_meters(lat0=gps_v0[0], lon0=gps_v0[1], lat1=gps_a1[0], lon1=gps_a1[1])
    
    curr_loc = [x_vcen, y_vcen]
    target = path_loc[-1]
    
    dis = ((curr_loc[0]-target[0])**2+(curr_loc[1]-target[1])**2)**0.5
    print(curr_loc, target, dis)
    if dis<dis_thr:
        tag = "stop"

        fn = "execution_tag"
        with open("./hummer_path/%s.txt"%fn, 'w') as f:
            f.write(str("0"))

        ## auto stop
        cleanup()
        print("mission completed")
    else:
        # print(dis, "to go")
        pass

    return jsonify({'status': 'success', 'tag': tag}), 200



@app.route('/path_gen', methods=['POST'])
def path_gen():
    print("-- new path generation --")
    tag = "success"

    ## also consider init GPS here and save previous path

    # init gps
    # os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p2.txt")

    now = datetime.now()

    # Format date and time as a string
    # dir_name = now.strftime("%Y-%m-%d_%H-%M-%S")

    # Create the directory
    # os.makedirs("./hummer_path/pre_cp/%s"%dir_name, exist_ok=True)

    ## cp backup path, gps, can
    # os.system("cp ./hummer_path/pathx5_can_heading.txt ./hummer_path/pre_cp/%s/pathx5_can_heading.txt"%dir_name)
    # os.system("cp ./hummer_path/gps_ref_p2.txt ./hummer_path/pre_cp/%s/gps_ref_p2.txt"%dir_name)
    # os.system("cp ./hummer_path/can_heading_ref.txt ./hummer_path/pre_cp/%s/can_heading_ref.txt"%dir_name)
    # os.system("cp ./hummer_path/obs_can_heading.txt ./hummer_path/pre_cp/%s/obs_can_heading.txt"%dir_name)
    # os.system("cp ./hummer_path/floor_vers_can_heading.txt ./hummer_path/pre_cp/%s/floor_vers_can_heading.txt"%dir_name)
   
    try:
        os.chdir("../../ConnAu/ros_ws")
    
        ## update path file (need to run all the time before compile new path)
        # launch_process(["ros2", "run", "fusion_py", "WpsFromJustinApp"])
    
        # subprocess.run("source ~/.bashrc && colcon build", shell=True, executable="/bin/bash", check=True)
        # subprocess.run("source ~/.bashrc", shell=True, executable="/bin/bash", check=True)
        # subprocess.run("colcon build", shell=True, executable="/bin/bash", check=True)
        # time.sleep(4)
        # subprocess.run("./scripts/fusion_launch.sh", shell=True, executable="/bin/bash", check=True)
        
        launch_process_ps("source ~/.bashrc")
        launch_process_ps("colcon build")
        launch_process_ps("./scripts/fusion_launch.sh")

        launch_process_bg(["ros2", "run", "fusion_py", "CANGPSReader"]) ## in case compile breaks it
        

    except:
        tag = "fail"
        print("\n Path generation fail.")


    finally:
        # os.chdir("../../srp/birdview_1021")
        os.chdir("../../srp/birdview")
        cleanup()
        return jsonify({'status': tag, 'echo': "new"}), 200

    
@app.route('/save_cur_path', methods=['POST'])
def save_cur_path():

    now = datetime.now()
    
    dir_name = now.strftime("%Y-%m-%d_%H-%M-%S")

    # Create the directory
    os.makedirs("./hummer_path/pre_cp/%s"%dir_name, exist_ok=True)

    ## cp backup path, gps, can
    os.system("cp ./hummer_path/pathx5_can_heading.txt ./hummer_path/pre_cp/%s/pathx5_can_heading.txt"%dir_name)
    os.system("cp ./hummer_path/gps_ref_p2.txt ./hummer_path/pre_cp/%s/gps_ref_p2.txt"%dir_name)
    os.system("cp ./hummer_path/can_heading_ref.txt ./hummer_path/pre_cp/%s/can_heading_ref.txt"%dir_name)
    os.system("cp ./hummer_path/obs_can_heading.txt ./hummer_path/pre_cp/%s/obs_can_heading.txt"%dir_name)
    os.system("cp ./hummer_path/floor_vers_can_heading.txt ./hummer_path/pre_cp/%s/floor_vers_can_heading.txt"%dir_name)
    os.system("cp ./hummer_path/path_local_den_1_stage_can_heading.txt ./hummer_path/pre_cp/%s/path_local_den_1_stage_can_heading.txt"%dir_name)
    
    return jsonify({'status': "success", 'echo': "new"}), 200

    
@app.route('/load_all_paths')
def load_all_paths():

    # read all folders name from directory
    path = './hummer_path/pre_cp/'
    files = os.listdir(path)

    paths = []
    for f in files:
        if "2025" in f:
            paths.append(f)

    return jsonify({'status': "success", 'echo': paths}), 200



@app.route('/load_path_spe', methods=['POST'])
def load_path_spe():
    
    data = request.get_json()  # parses JSON body
    name = data.get('date')

    tag = "success"
    
    ## replace current with save1
    os.system("cp ./hummer_path/pre_cp/%s/pathx5_can_heading.txt ./hummer_path/pathx5_can_heading.txt"%name)
    os.system("cp ./hummer_path/pre_cp/%s/gps_ref_p2.txt ./hummer_path/gps_ref_p2.txt"%name)
    os.system("cp ./hummer_path/pre_cp/%s/can_heading_ref.txt ./hummer_path/can_heading_ref.txt"%name)
    os.system("cp ./hummer_path/pre_cp/%s/obs_can_heading.txt ./hummer_path/obs_can_heading.txt"%name)
    os.system("cp ./hummer_path/pre_cp/%s/floor_vers_can_heading.txt ./hummer_path/floor_vers_can_heading.txt"%name)
    os.system("cp ./hummer_path/pre_cp/%s/path_local_den_1_stage_can_heading.txt ./hummer_path/path_local_den_1_stage_can_heading.txt"%name)

    print("replace path by pre ", name)
    
    '''
    try:
        os.chdir("../../ConnAu/ros_ws")

        # subprocess.run("source ~/.bashrc && sudo colcon build", shell=True, executable="/bin/bash", check=True)
        

        ## update path file (need to run all the time before compile new path)
        # launch_process(["ros2", "run", "fusion_py", "WpsFromJustinApp"])
        
        # subprocess.run("source ~/.bashrc && sudo colcon build", shell=True, executable="/bin/bash", check=True)
        # subprocess.run("source ~/.bashrc", shell=True, executable="/bin/bash", check=True)
        # subprocess.run("colcon build", shell=True, executable="/bin/bash", check=True)
        # subprocess.run("./scripts/fusion_launch.sh", shell=True, executable="/bin/bash", check=True)

        launch_process_ps("source ~/.bashrc")
        launch_process_ps("colcon build")
        launch_process_ps("./scripts/fusion_launch.sh")

        
        # time.sleep(4)
        launch_process_bg(["ros2", "run", "fusion_py", "CANGPSReader"]) ## in case compile breaks it
        
        
    except:
        print("\nload pre-scan path fail.")
        tag = "fail"

    finally:
        cleanup()   
        os.chdir("../../srp/birdview")
        return jsonify({'status': tag, 'echo': "new"}), 200
    '''

    return jsonify({'status': tag, 'echo': "new"}), 200



@app.route('/saved1_path_gen', methods=['POST'])
def saved_path_gen():
    
    ## cp backup path, gps, can
    now = datetime.now()
    tag = "success"


    # Format date and time as a string
    dir_name = now.strftime("%Y-%m-%d_%H-%M-%S")

    # Create the directory
    os.makedirs("./hummer_path/pre_cp/%s"%dir_name, exist_ok=True)


    ## replace current with save1
    os.system("cp ./hummer_path/save1/pathx5_can_heading.txt ./hummer_path/pathx5_can_heading.txt")
    os.system("cp ./hummer_path/save1/gps_ref_p2.txt ./hummer_path/gps_ref_p2.txt")
    os.system("cp ./hummer_path/save1/can_heading_ref.txt ./hummer_path/can_heading_ref.txt")
    os.system("cp ./hummer_path/save1/obs_can_heading.txt ./hummer_path/obs_can_heading.txt")
    os.system("cp ./hummer_path/save1/floor_vers_can_heading.txt ./hummer_path/floor_vers_can_heading.txt")
    os.system("cp ./hummer_path/save1/path_local_den_1_stage_can_heading.txt ./hummer_path/path_local_den_1_stage_can_heading.txt")

    print("replace path by save1")
    '''
    try:
        os.chdir("../../ConnAu/ros_ws")

        # subprocess.run("source ~/.bashrc && sudo colcon build", shell=True, executable="/bin/bash", check=True)
        

        ## update path file (need to run all the time before compile new path)
        # launch_process(["ros2", "run", "fusion_py", "WpsFromJustinApp"])
        
        # subprocess.run("source ~/.bashrc && sudo colcon build", shell=True, executable="/bin/bash", check=True)
        # subprocess.run("source ~/.bashrc", shell=True, executable="/bin/bash", check=True)
        # subprocess.run("colcon build", shell=True, executable="/bin/bash", check=True)
        # subprocess.run("./scripts/fusion_launch.sh", shell=True, executable="/bin/bash", check=True)

        launch_process_ps("source ~/.bashrc")
        launch_process_ps("colcon build")
        launch_process_ps("./scripts/fusion_launch.sh")

        
        # time.sleep(4)
        launch_process_bg(["ros2", "run", "fusion_py", "CANGPSReader"]) ## in case compile breaks it
        
        
    except:
        print("\nload pre-scan path fail.")
        tag = "fail"

    finally:
        cleanup()   
        os.chdir("../../srp/birdview")
        return jsonify({'status': tag, 'echo': "new"}), 200
    '''

    
    return jsonify({'status': 'success', 'echo': "new"}), 200



##############


@app.route('/all_loc')
def saved_locs():

    data = []
    base = [42.520209,-83.043500]
    base_dis = 10

    map1 = {
        "name":"demo",
        "latlon": base,
    }
    data.append(map1)

    
    path_cp = './hummer_path/pre_cp/'
    files = os.listdir(path_cp)

    paths = []
    cnt = 1
    for f in files:
        if "2025" in f:
            # Move 2 meters east (bearing = 90 degrees) 
            start = Point(base[0], base[1])
            destination = distance(meters=base_dis*cnt).destination(start, bearing=90)
            # destination = distance(meters=base_dis*cnt).destination(destination, bearing=180)
            new_lat, new_lon = destination.latitude, destination.longitude

            map_spe = {
                "name":f,
                "latlon": [new_lat, new_lon],
            }
            data.append(map_spe) 
            cnt += 1

    return jsonify(data)



@app.route('/map')
def map_data():
    return render_template('map.html')
    
@app.route('/map_phone')
def map_data_phone():
    return render_template('map_phone.html')
    

# @app.route('/path.json')
# def path_data():

#     ## load from the processed smooth path
#     import csv
#     f = open("hummer_path/pathx5_smooth_gps.csv", "r")
#     h = f.readline()
#     data = []
#     for line in f:
#         eles = line.split(",")
#         data.append([float(eles[0]), float(eles[1])])
#     f.close()
#     return jsonify(data)

    # print("smo path", data)
    # exit()

@app.route('/path_ll.json')
def path_ll_data():

    ## load from the processed smooth path
    # f = open("hummer_path/pathx5.txt", "r")
    # if USE_CAN_HEAD:
    with open("hummer_path/pathx5_can_heading.txt", "r") as f:
        line = f.readline()
    path_loc = json.loads(line)
    # f.close()
    return jsonify(path_loc)

    # print("path ll", data)
    # exit()

@app.route('/path_den1.json')
def path_den1_data():

    ## load from the processed smooth path
    # fn = "hummer_path/path_local_den_1_stage.txt"
    if USE_CAN_HEAD:
        fn = "hummer_path/path_local_den_1_stage_can_heading.txt"
        # fn = "hummer_path/path_local_den_1_stage_can_heading_rev.txt"
    with open(fn, 'r') as f:
    # with open("hummer_path/test.txt", 'r') as f:
        path_loc_den = json.load(f)
    
    # Use Johson's path to replace it (without f/r gear setting yet)
    # fn = "interpolated_wps_from_ros.txt"
    # with open(fn, 'r') as f:
        # path_loc_den = json.load(f)
    ####################################

    # print(path_loc_den)
    return jsonify(path_loc_den)


@app.route('/path_den2.json')
def path_den2_data():

    ## load from the processed smooth path
    fn = "hummer_path/path_local_den_2_stage.txt"
    if USE_CAN_HEAD:
        fn = "hummer_path/path_local_den_2_stage_can_heading.txt"
    with open(fn, 'r') as f:
        path_loc_den = json.load(f)
    
    # print(path_loc_den)
    return jsonify(path_loc_den)
    
@app.route('/obs_ll.json')
def obs_ll_data():

    ## load from the processed smooth path
    # f = open("hummer_path/obs.txt", "r")
    # if USE_CAN_HEAD:
    with open("hummer_path/obs_can_heading.txt", "r") as f:
        line = f.readline()
    obs = json.loads(line)
    # f.close()
    valid_obs = []
    veh_to_ob_thr = 3 # 3m away from vehicle center
    for ob in obs:
        if (ob[0]**2+ob[1]**2)**0.5>veh_to_ob_thr:
            valid_obs.append(ob)

    rate = 10
    down_obs = valid_obs[::rate]
    return jsonify(down_obs)

    # print("obs ll", data)
    # exit()

@app.route('/mesh_ll.json')
def mesh_ll_data():

   # fn = "allP"
   # fn = "allP1"
   fn = "allP2" 
   with open("hummer_path/%s.txt"%fn, "r") as f:
       line = f.readline()
   obs = json.loads(line)

   return jsonify(obs)


@app.route('/floor_ll.json')
def floor_data():

    ## load from the processed smooth path
    fn = "hummer_path/floor_vers.txt"
    if USE_CAN_HEAD:
        fn = "hummer_path/floor_vers_can_heading.txt"
    with open(fn, 'r') as f:
        floor = json.load(f)
    return jsonify(floor)

    # print("obs ll", data)
    # exit()




@app.route('/person.json')
def person_data():
    with open("person.txt", "r") as f:
        data = f.readline()
    # f.close()
    
    person_gps = json.loads(data)[0]
    # print("load", person_gps)

    # return jsonify(path)
    return jsonify(person_gps)

'''
@app.route('/ublox.json')
def ublox_data():
    
    # gps to meter
    # f = open("hummer_path/gps_ref_p.txt", "r")
    f = open("hummer_path/gps_ref_p2.txt", "r")
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
'''

@app.route('/can_h.json')
def camh_data():

    with open("hummer_path/can_heading_ref.txt", "r") as f:
        line = f.readline()
    data = json.loads(line)
    heading0 = data[2]
    gps0 = [data[0], data[1]]
    # f.close()
    
    # gps to meter
    line = [0,0,0]
    try:
        with open("hummer_path/can_heading.txt", "r") as f:
            line = f.readline()
        ch = json.loads(line)
    except:
        print('loading can data error')
    # print("-----", ch)
    heading1 = ch[2]
    gps1 = [ch[0], ch[1]]
    # f.close()

    x,y = gps_to_loc_meters(lat0=gps0[0], lon0=gps0[1], lat1=gps1[0], lon1=gps1[1])

    # f = open("hummer_path/pathx5.txt", "r")
    # if USE_CAN_HEAD:
    with open("hummer_path/pathx5_can_heading.txt", "r") as f:
        line = f.readline()
    path = json.loads(line)
    # f.close()
    target = path[3]
    dx = target[0]-x
    dy = target[1]-y
    cur_distance = (dx**2+dy**2)**0.5

    return jsonify([ch[0],ch[1],ch[2],x,y, cur_distance])


## for value show
@app.route('/ublox_gps.json')
def ublox_gps_data():
    
    # gps to meter
    ## use reference point when uploading path
    # f = open("hummer_path/gps_ref_p.txt", "r")
    ## use reference point right after car stop start scanning (avoid shift)
    
    with open("hummer_path/gps_ref_p2.txt", "r") as f:
        line = f.readline()
    # f = open("hummer_path/gps_ref_p2.txt", "r")
    # line = f.readline()
    gps0 = json.loads(line)
    # f.close()

    with open("hummer_path/gps.txt", "r") as f:
        line = f.readline()
    # f = open("hummer_path/gps.txt", "r")
    # line = f.readline()
    gps1 = json.loads(line)
    # f.close()

    # print(gps0, gps1[1])
    x,y = gps_to_loc_meters(lat0=gps0[0], lon0=gps0[1], lat1=gps1[0], lon1=gps1[1])

    
    with open("hummer_path/pathx5_can_heading.txt", "r") as f:
        line = f.readline()
    # f = open("hummer_path/pathx5_can_heading.txt", "r")
    # line = f.readline()
    path = json.loads(line)
    # f.close()

    target = path[3]
    dx = target[0]-x
    dy = target[1]-y
    cur_distance = (dx**2+dy**2)**0.5


    return jsonify([gps1[0], gps1[1], x,y, cur_distance])




'''
@app.route('/ublox2.json')
def ublox2_data():
    
    # gps to meter
    f = open("hummer_path/gps_ref_p.txt", "r")
    line = f.readline()
    gps0 = json.loads(line)
    f.close()

    f = open("hummer_path/can_heading_ref.txt", "r")
    line = f.readline()
    heading0 = json.loads(line)[2]
    f.close()

    f = open("hummer_path/gps.txt", "r")
    line = f.readline()
    gps1 = json.loads(line)
    f.close()

    f = open("hummer_path/can_heading.txt", "r")
    line = f.readline()
    heading1 = json.loads(line)[2]
    f.close()

    gps_v0 = move_gps_point(gps0[0], gps0[1], heading0, center_off_distance_ublox)
    gps_v1 = move_gps_point(gps1[0], gps1[1], heading1, center_off_distance_ublox)

    print(gps_v0, gps_v1, heading0, heading1)
    x_vcen, y_vcen = gps_to_loc_meters(lat0=gps_v0[0], lon0=gps_v0[1], lat1=gps_v1[0], lon1=gps_v1[1])

    return jsonify([x_vcen, y_vcen])
'''

## for vis dot
@app.route('/ublox3.json')
def ublox3_data():
    
    # gps to meter
    ## use reference point when uploading path
    # f = open("hummer_path/gps_ref_p.txt", "r")
    ## use reference point right after car stop start scanning (avoid shift)
    with open("hummer_path/gps_ref_p2.txt", "r") as f:
        line = f.readline()
    gps0 = json.loads(line)
    # f.close()

    with open("hummer_path/can_heading_ref.txt", "r") as f:
        line = f.readline()
    heading0 = json.loads(line)[2]
    # f.close()

    gps_v0 = move_gps_point(gps0[0], gps0[1], heading0, center_off_distance_ublox)
    gps_a0 = gps0

    with open("hummer_path/gps.txt", "r") as f:
        line = f.readline()
    gps1 = json.loads(line)
    # f.close()

    with open("hummer_path/can_heading.txt", "r") as f:
        line = f.readline()
    heading1 = json.loads(line)[2]
    # f.close()

    gps_v1 = move_gps_point(gps1[0], gps1[1], heading1, center_off_distance_ublox)
    gps_a1 = gps1

    # print(gps_v0, gps_v1, heading0, heading1)
    x_vcen, y_vcen = gps_to_loc_meters(lat0=gps_v0[0], lon0=gps_v0[1], lat1=gps_v1[0], lon1=gps_v1[1])
    x_ant, y_ant = gps_to_loc_meters(lat0=gps_v0[0], lon0=gps_v0[1], lat1=gps_a1[0], lon1=gps_a1[1])
    
    # global gps_log
    # global heading_log
    # heading_log.append(heading1)
    # gps_log.append([x_vcen, y_vcen])

    fn = "execution_tag"
    with open("./hummer_path/%s.txt"%fn, 'r') as f:
        tag = f.readline()

    # if tag[0]==1:

    with open("./hummer_path/gps_log.txt", 'a') as f:
        f.write("[%.2f, %.2f]\n"%(x_vcen, y_vcen))
    with open("./hummer_path/heading_log.txt", 'a') as f:
        f.write("%.2f\n"%heading1)


    

    if tag[0]=="0":
    ## note gps history before generating path to find better coordinate trans reference
        with open("./hummer_path/gps_history_before_start.txt", 'a') as f:
            f.write("[%f, %f]\n"%(gps1[0], gps1[1]))
    

    return jsonify([[x_vcen, y_vcen], [x_ant, y_ant]])
    # return jsonify([x_ant, y_ant])
    # return jsonify([x_vcen, y_vcen])




@app.route('/path_reg_x5')
def path_reg_x5():

    opt = request.args.get('opt', default=None, type=str)
    
    print("get value", opt)
    if opt is None:
        return jsonify({"error"}), 400


    os.system("cp ./hummer_path/can_heading.txt ./hummer_path/can_heading_ref.txt")
    os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p2.txt")
    
    if int(opt)==1:
        ## (option 1) this use the current gps and heading 
        os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p2.txt")
    
    elif int(opt)==2:
        os.system("cp ./hummer_path/gps_ref_p1.txt ./hummer_path/gps_ref_p2.txt")
    
    elif int(opt)==3:
        ## (option 3) use avg
        file_name = './hummer_path/gps_history_before_start.txt'
        avg_x, avg_y, total_lines = get_average_coordinates(file_name)
        with open("./hummer_path/gps_ref_p2.txt", 'w') as f:
            f.write("[%f,%f]"%(float(avg_x), float(avg_y)))
    
    '''
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
    '''

    return jsonify('success'), 200




@app.route('/can_gps.json')
def can_gps_data():
    
    ## need to add in the composition (both move left/right and front/back)

    with open("hummer_path/can_heading_ref.txt", "r") as f:
        line = f.readline()
    data = json.loads(line)
    gps0 = [data[0],data[1]]
    heading0 = data[2]
    # f.close()

    gps_v0 = move_gps_point(gps0[0], gps0[1], heading0, center_off_distance_can_gps_head_back)
    # gps_v0 = move_gps_point(gps_v0[0], gps_v0[1], (heading0-90)%360, center_off_distance_can_gps_left_right)
    gps_a0 = gps0

    with open("hummer_path/can_heading.txt", "r") as f:
        line = f.readline()
    data1 = json.loads(line)
    heading1 = data1[2]
    gps1 = [data1[0],data1[1]]
    # f.close()

    gps_v1 = move_gps_point(gps1[0], gps1[1], heading1, center_off_distance_can_gps_head_back)
    # gps_v1 = move_gps_point(gps_v1[0], gps_v1[1], (heading1-90)%360, center_off_distance_can_gps_left_right)
    gps_a1 = gps1

    # print(gps_v0, gps_v1, heading0, heading1)
    x_vcen, y_vcen = gps_to_loc_meters(lat0=gps_v0[0], lon0=gps_v0[1], lat1=gps_v1[0], lon1=gps_v1[1])
    x_ant, y_ant = gps_to_loc_meters(lat0=gps_v0[0], lon0=gps_v0[1], lat1=gps_a1[0], lon1=gps_a1[1])
    
    return jsonify([[x_vcen, y_vcen], [x_ant, y_ant]])
    # return jsonify([x_ant, y_ant])
    # return jsonify([x_vcen, y_vcen])


@app.route('/ublox_gps_before_shift.json')
def init_gps_bf_shift():

    os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p1.txt")
    # os.system("cp ./hummer_path/can_heading.txt ./hummer_path/can_heading_ref.txt")
    
    os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_history_before_start.txt")
    with open("./hummer_path/gps_history_before_start.txt", "a") as f:
        f.write("\n")


    with open("./hummer_path/gps_ref_p1.txt", "r") as f:
        data = f.readline()
    gps = json.loads(data)

    return jsonify(gps)




@app.route('/init_gps.json')
def init_gps_data():
    with open("hummer_path/init_veh_gps.txt", "r") as f:
        data = f.readline()
    # f.close()
    
    gps = json.loads(data)[0]
    # print("load init veh", gps)

    return jsonify(gps)


@app.route('/phone_local.json')
def phone_local_data():
    # f = open("./hummer_path/phone_local.txt", "r")
    with open("./hummer_path/phone_local_can_heading.txt", "r") as f:
        data = f.readline()
    # f.close()

    phone_gps = json.loads(data)[0]
    
    # return jsonify(path)
    return jsonify(phone_gps)



@app.route('/trailer_local.json')
def trailer_local_data():

    d1 = 0.0
    d2 = 0.0
    theta_rad = 0.0
    tx = 0.0
    ty = 0.0


    ## load updated uwb distance (vehicle to trailer)
    with open("./hummer_path/uwb_recent_d1.txt", "r") as f:
        data = f.readline()
        d1 = json.loads(data)[1]
    
    with open("./hummer_path/uwb_recent_d2.txt", "r") as f:
        data = f.readline()
        d2 = json.loads(data)[1]
    
    ## local trailer position
    # theta_est, pos = estimate_trailer_pos(d1, d2)
    # theta_est, pos = estimate_trailer_pos_single_sensor(d1, d2)
    # theta_est *= -1

    '''
    theta_est, pos = estimate_trailer_pos_dual_vehicle_sensors(d1, d2)
    theta_est *= -1
    '''

    f_t = "./hummer_path/uwb_recent_theta.txt"
    with open(f_t, "r") as f:
        uwb_est = json.loads(f.readline())
        theta_rad = uwb_est[1]
        tx = uwb_est[2]
        ty = uwb_est[3]

    theta_est = -theta_rad
    pos = [tx, ty]

    # print("--J est. unify uwb decawave (trailer2veh)--", d1, d2, theta_rad)
    
    # print(f"Angle: {theta_est:.2f}°")
    # print(f"Trailer Center (X, Y): ({pos[0]:.2f}, {pos[1]:.2f})")

    ## get vehicle current location/heading
    veh_loc = [0, 0]
    veh_h = 0

    # vehicle loc in meter (gps to meter)
    with open("hummer_path/gps_ref_p2.txt", "r") as f:
        line = f.readline()
    gps0 = json.loads(line)

    with open("hummer_path/gps.txt", "r") as f:
        line = f.readline()
    gps1 = json.loads(line)

    x_vcen, y_vcen = gps_to_loc_meters(lat0=gps0[0], lon0=gps0[1], lat1=gps1[0], lon1=gps1[1])
    veh_loc = [x_vcen, y_vcen]

    with open("hummer_path/can_heading.txt", "r") as f:
        line = f.readline()
    veh_h = json.loads(line)[2]
    

    ## transform with respect to veh location
    tx, ty, th = get_trailer_world_state_nav(veh_loc[0], veh_loc[1], veh_h, pos[0], pos[1], theta_est)
    # print(f"Trailer World Location: ({tx:.2f}, {ty:.2f})")

    abs_pos = [tx, ty]

    # raw uwb distance x2, trailer to vehicle heading/pos, trailer to world pos/heading
    return jsonify([d1, d2, theta_est, pos, abs_pos, th])


@app.route('/veh_local_uwb.json')
def veh_local_uwb():

    with open("hummer_path/can_heading_ref.txt", "r") as f:
        line = f.readline()
    data = json.loads(line)
    can_ref_heading = data[2]

    with open("hummer_path/can_heading.txt", "r") as f:
        line = f.readline()
    data = json.loads(line)
    can_cur_heading = data[2]

    with open("hummer_path/phone_local_can_heading.txt", "r") as f:
        line = f.readline()
    data = json.loads(line)
    cam_loc = data[0]

    with open("hummer_path/UWB_anc1.txt", "r") as f:
        line = f.readline()
    data = json.loads(line)
    t1 = int(data["ms"])
    d1 = float(data["distance"])

    with open("hummer_path/UWB_anc2.txt", "r") as f:
        line = f.readline()
    data = json.loads(line)
    t2 = int(data["ms"])
    d2 = float(data["distance"])

    # print("load uwb data", can_ref_heading, can_cur_heading, cam_loc, d1, d2)
    # exit()

    ## anchor configuration
    # a1 = [-1.0, 1.0]
    # a2 = [-1.0, -1.0]
    
    ## debugging data
    # d1 = 1
    # d2 = 2
    # cam_loc = [1,1]
    # can_ref_heading = 100
    # can_cur_heading = 55


    ## onlyif delay <500ms between two anchors and both value are valid
    veh_loc_uwb = [0.0, 0.0, 0.0, 0.0, a1[0], a1[1], a2[0], a2[1], d1, d2, can_cur_heading, 0, 0, "N"]
    if abs(t1-t2)<500 and d1 and d2 and not d1==0.0 and not d2==0.0:
        veh_loc_uwb = get_veh_loc_UWB(a1, a2, d1, d2, cam_loc, can_ref_heading, can_cur_heading)
        if veh_loc_uwb is None:
            veh_loc_uwb = [0.0, 0.0, 0.0, 0.0, a1[0], a1[1], a2[0], a2[1], d1, d2, can_cur_heading, 0,0, "N"]
            # print("UWB distance intersection not valid")
        else:
            veh_loc_uwb = veh_loc_uwb  + [d1, d2, can_cur_heading, cam_loc[0], cam_loc[1], "Y"]
            # print("veh UWB loc", veh_loc_uwb)

            
    # else:
    #     print("veh UWB signal not valid")


    fon = "uwb_loc_can_heading"
    with open("./hummer_path/%s.txt"%fon, "w") as fo:
        fo.write(str(veh_loc_uwb))
    # fo.close()

    return jsonify([veh_loc_uwb])




@app.route('/path_local.json')
def path_local_data():
    # planed_path = [[0.5349896, -1.0209819, -0.034887522], [1.1479379, -0.9539301, 0.022688236], [1.6387491, -0.9002394, 0.06879134], [2.0209935, -0.8584248, 0.104696505], [2.3082423, -0.82700217, 0.13167852], [2.5140662, -0.80448663, 0.15101206], [2.6520362, -0.7893937, 0.16397193], [2.735723, -0.7802391, 0.17183281], [2.778697, -0.7755382, 0.1758695], [2.7945297, -0.77380615, 0.17735669], [2.7967916, -0.77355874, 0.17756915], [2.7967916, -0.77355874, 0.17756915], [2.9501946, -0.7735586, -0.34228048], [3.052261, -0.7735588, -0.764819], [3.0863569, -0.7735586, -1.1076071], [3.0358512, -0.77355874, -1.3882055], [2.8841121, -0.77355874, -1.6241748], [2.6145074, -0.7735587, -1.8330758], [2.210405, -0.77355874, -2.0324688], [1.6551733, -0.77355874, -2.239915], [0.93218005, -0.77355874, -2.472974], [0.02479285, -0.77355874, -2.749208]]
    # veh_loc = [0.53, -0.03]
    # path = get_path(veh_loc, planed_path)

    # test_path_gps = [[42.60855579648876, -82.99335282511062], [42.608554184618896, -82.99334639877743], [42.60855289393257, -82.99334125296848], [42.60855188874457, -82.99333724540344], [42.60855113336653, -82.99333423379981], [42.608550592111705, -82.99333207587951], [42.608550229292256, -82.99333062936292], [42.6085500092209, -82.99332975196612], [42.60854989621195, -82.9933293014095], [42.60854985457704, -82.99332913541787], [42.6085498486289, -82.99332911170437], [42.6085498486289, -82.99332911170437], [42.6085449800624, -82.99332902898209], [42.608541077917856, -82.99332922751904], [42.608538030848315, -82.99332996081316], [42.608535727487535, -82.99333148234201], [42.60853405649284, -82.9933340456035], [42.608532906500145, -82.99333790408542], [42.60853216616034, -82.99334331127547], [42.60853172411148, -82.99335052066725], [42.60853146900874, -82.99335978574338], [42.60853128948803, -82.99337136000547]]
    
    with open("path_local.txt", "r") as f:
        data = f.readline()
    # f.close()
    
    path_gps = json.loads(data)
    # print("load", path_gps)

    # return jsonify(path)
    return jsonify(path_gps)


@app.route('/ar_test.json')
def artest():

    # f = open("hummer_path/pathx5.txt", "r")
    # if USE_CAN_HEAD:
    with open("hummer_path/pathx5_can_heading.txt", "r") as f:
        line = f.readline()
    path = json.loads(line)
    # f.close()
    target = path[3]
    distance = (target[0]**2+target[1]**2)**0.5
    return jsonify(distance)

    


@app.route('/uwb_read.json')
def uwb_read():
    with open("uwb_distance.txt", "r") as f:
        data = f.readline()
    # f.close()
    
    uwb_dis = json.loads(data)["distance"]
    # print("load uwb distance", uwb_dis)

    # return jsonify(path)
    return jsonify(uwb_dis)


    # data = request.get_json()
    # if not data or 'message' not in data:
    #     return jsonify({'error': 'Invalid data'}), 400

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
    with open("veh_gps.txt", "w") as f:
        f.write(message)
    # f.close()
    # print(f"Received veh gps message: {message}")

    
    return jsonify({'status': 'success', 'echo': message}), 200



if __name__ == '__main__':
    # app.run(host='0.0.0.0', port=5000, debug=True)
    app.run(host='0.0.0.0', port=5001, debug=True, ssl_context=('cert.pem', 'key.pem'))
    # app.run(host='0.0.0.0', port=80, debug=True, ssl_context='adhoc')

    # app.run(host='0.0.0.0', port=5001, debug=True)
    

    
    


