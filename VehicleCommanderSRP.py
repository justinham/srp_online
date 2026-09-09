import can
import cantools
import sys

import cantools.database
import rclpy
from rclpy.node import Node
import rclpy
import rclpy.duration
import time
import json
import numpy as np

# import matplotlib.pyplot as plt

from fusion.msg import VehicleCommand
from fusion.msg import TripleVectorWps

## Justin ######

import paho.mqtt.client as mqtt
import base64
import struct
import collections
import math

from scipy.signal import butter, lfilter

# asyncio removed — uBlox loop now runs via rclpy.create_timer (see send_ublox)




## configuration
# gateway_ip = '10.135.229.51' # pi address (MQTT server connect to bridge sensor)
gateway_ip = '192.168.5.51' # pi address on hummer (MQTT server connect to bridge sensor)
port = 1883

tag1 = '0c39' # left
tag2 = '879c' # right 
anc = '442a' 


# fn1 = 'uwb_recent_d1.txt'
# fn2 = 'uwb_recent_d2.txt'
fn1 = '/home/connau/srp/birdview/hummer_path/uwb_recent_d1.txt'
fn2 = '/home/connau/srp/birdview/hummer_path/uwb_recent_d2.txt'
fn3 = '/home/connau/srp/birdview/hummer_path/uwb_recent_theta.txt'

topic1 = 'dwm/node/' + tag1 + '/uplink/data'
topic2 = 'dwm/node/' + tag2 + '/uplink/data'

veh_len = 5
trailer_len = 5
# --- Trailer UWB GEOMETRY CONSTANTS ---
Rv, Ra = 0.85, 0.85    # Half-width of two tags

L_arm = 1.20 # lever arm length
L_ht = 0.35 # vehicle rear center to hinge

yh = -0.35 - veh_len/2   # Hinge position relative to vehicle center
Lt = 1.2 + trailer_len/2    # Hinge to Trailer center

L_total = L_arm + L_ht

# ele_offset = 0.37 ## mounting height diff between anchor and tag 
# ele_offset = 0.28 ## mounting height diff between anchor and tag 
ele_offset = 0.0 ## mounting height diff between anchor and tag 




history = collections.defaultdict(lambda: collections.deque(maxlen=10))
history_arr1 = []
history_arr2 = [] 
history_theta = [0,0,0,0,0,0,0,0,0,0]



## Filter
def is_outlier(val, window, threshold=1.0):
    """Simple outlier detection using Standard Deviation."""
    if len(window) < 5:  # Not enough data to judge yet
        return False
    
    arr = np.array(window)
    mean = np.mean(arr)
    std = np.std(arr)
    
    # If the reading is more than 'threshold' std devs away, it's an outlier
    if abs(val - mean) > threshold * std:
        return True
    return False



def low_pass_filter(current_val, previous_filtered_val, alpha=0.1):
    """
    Applies a first-order low-pass filter (Exponential Moving Average).
    
    alpha: Smoothing factor between 0.0 and 1.0. 
           Lower values = smoother but slower to respond.
           Higher values = faster response but lets more noise through.
    """
    if previous_filtered_val is None:
        return current_val
        
    return (alpha * current_val) + ((1.0 - alpha) * previous_filtered_val)




def butter_lowpass_filter(data, cutoff_freq, sampling_freq, order=2):
    # Calculate Nyquist frequency
    nyquist = 0.5 * sampling_freq
    normal_cutoff = cutoff_freq / nyquist
    
    # Get filter coefficients
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    
    # Apply filter to data array
    filtered_data = lfilter(b, a, data)
    return filtered_data




## localization: 1 anchor 2 tag (tag on veh)
def estimate_trailer_pos_dual_vehicle_sensors(d1, d2):
    
    # 1. Total effective lever arm
    # The gain now depends on the vehicle sensor width (Rv)
    # denominator = 4 * Rv * L_sv
    denominator = 4 * Rv * L_arm
    
    # 2. Solve for Theta
    # We swap d1 and d2 here to maintain: Right Turn = Positive Angle
    sin_theta = (d2**2 - d1**2) / denominator
    sin_theta = np.clip(sin_theta, -1.0, 1.0)
    theta_rad = np.arcsin(sin_theta)
    
    # 3. Calculate Trailer Position
    tx = Lt * np.sin(theta_rad)
    ty = yh - Lt * np.cos(theta_rad)

    print(theta_rad, tx, ty, '--')
    
    return -np.degrees(theta_rad), (tx, ty)


# add ublox (need define can msg support)
#SG_ uBloxGPS_inv: Keyword for a Signal, named uBloxGPS_inv. It indicates if the GPS data is invalid.25|1: Starts at bit 25 and is 1 bit long.@0-:@0 means Big-Endian (Motorola byte order).- means it is a signed data type (though irrelevant for a 1-bit flag).(1,0): The raw value multiplier (Factor) is 1, and the Offset is 0. formula: Physical = (Raw * 1) + 0.[0|0]: The minimum and maximum physical limits (0 to 0).
#uBlox_Latitude: The latitude coordinates from the GPS module.7|30@0-: Starts at bit 7, is 30 bits long, Big-Endian, and signed (allowing negative numbers for Southern latitudes).(1,0): Factor is 1, Offset is 0.[-536870912|536870911]: The range of allowed physical values.
#uBlox_Longitude: The longitude coordinates from the u-blox GPS module.39|31@0-: Starts at bit 39, is 31 bits long, Big-Endian, and signed (allowing negative numbers for Western longitudes).(1,0): Factor is 1, Offset is 0.[-1073741824|1073741823]: The range of allowed physical values."mas": The unit of measurement stands for Milliarcseconds (1/3,600,000 of a degree). To convert this to standard decimal degrees, you usually divide the final value by 3,600,000.
def cast_GPS_int(val):
    val_int = int(val * 3600000)
    return val_int

def load_ublox():
    data_ublox = [0,0]
    fn_x = '/home/connau/srp/birdview/hummer_path/gps.txt'
    with open(fn_x, "r") as f:
        line = f.readline()
    
    data_ublox = json.loads(line)
    lat = cast_GPS_int(data_ublox[0])
    lon = cast_GPS_int(data_ublox[1])
    data_ublox = [lat, lon]
    
    data_veh_gps = [0,0,0]
    fn_x2 = '/home/connau/srp/birdview/hummer_path/can_heading.txt'
    with open(fn_x2, "r") as f:
        line = f.readline()
    data_veh_gps = json.loads(line)
    lat2 = cast_GPS_int(data_veh_gps[0])
    lon2 = cast_GPS_int(data_veh_gps[1])
    data_veh_gps = [lat2, lon2, data_veh_gps[2]]
    
    return data_ublox, data_veh_gps

## MQTT
def on_connect(client, userdata, flags, rc, properties=None):
    if rc==0:
        client.subscribe(topic1)
        client.subscribe(topic2)
        # client.subscribe(topic3)
        print('conn sub distance topic')
    else:
        print('conn fail')



def make_on_message(ros_node, uwb_data):

    def on_message(client, userdata, msg):

        try:
        # if True:
            # print('0')
            time_ms = int(time.time()*1000)
            payload = json.loads(msg.payload.decode('utf-8'))
            raw_data = payload.get('data')
            sid = None
            if tag1 in msg.topic:
                sid = tag1
            elif tag2 in msg.topic:
                sid = tag2
            
            if not raw_data:
                return

            # Unpack the 6 bytes
            dec_byte = base64.b64decode(raw_data)
            addr_l, addr_h, d1, d2, d3, d4 = struct.unpack('BBBBBB', dec_byte)

            # dec_byte = base64.b64decode(raw_data)
            # addr_l, addr_h, d1, d2, d3, d4 = dec_byte[:6]
            
            addr = f'{addr_h:02x}{addr_l:02x}'
            
            # Calculate distance in meters
            dis = (d1 + d2*256 + d3*256**2 + d4*256**3) / 1000.0
            
            # --- FILTERING LOGIC ---
            '''
            # 1. Basic None/Zero/Negative filtering
            if dis <= 0 or dis is None:
                return

            # 2. Outlier filtering based on history
            sensor_history = history[sid]
            
            if is_outlier(dis, sensor_history):
                print(f"Skipping Outlier: {dis} for {sid}")
                return
                
            # 3. Add to dictionary if valid
            sensor_history.append(dis)
            d1 = history[tag1]
            d2 = history[tag2]  
            '''

            # # --- FILTERING LOGIC without dqueue---            
            # # 1. Basic None/Zero/Negative filtering
            # if dis <= 0 or dis is None:
            #     return

            # # 2. Outlier filtering based on history
            # sensor_history = history[sid]
            
            # if is_outlier(dis, sensor_history):
            #     print(f"Skipping Outlier: {dis} for {sid}")
            #     return
                
            # # 3. Add to dictionary if valid
            # sensor_history.append(dis)
            # d1 = history[tag1]
            # d2 = history[tag2]  
            

            # -- without filtering --
            # print('1')
            uwb_data[sid] = dis
            d1 = 0
            d2 = 0

            d1 = uwb_data[tag1]
            d2 = uwb_data[tag2]

            d1 = (d1**2-ele_offset**2)**0.5
            d2 = (d2**2-ele_offset**2)**0.5
            # print('2')

            # 4. theta estimation (using most recent data)
            # valid = 0
            # theta_est = 0
            # pos = (0,0)
            # if (d1==0 or d2==0):
            #     pass
            # else:
            theta_est, pos = estimate_trailer_pos_dual_vehicle_sensors(d1=d1, d2=d2)

            # print(theta_est)

            ## -- directly filter on theta
            prev_filtered = 0.0
            history_theta.pop(0)
            history_theta.append(theta_est)

            for raw in history_theta:
                lp_filtered = low_pass_filter(raw, prev_filtered, alpha=0.1) ## too small 0.1
                # print(f"Raw: {raw:>4} -> Filtered: {filtered:.2f}", history_theta)
                prev_filtered = lp_filtered

            ## simple avg filter
            avg_filtered = theta_est
            avg_filtered = sum(history_theta) / len(history_theta)

            # trailer_data = [d1, d2, theta_est]
            # trailer_data = [d1, d2, prev_filtered]
            
            # add control status: # 0: init; 1:start; 2: stop
            ## check if 'start' btn clicked, otherwise should not into moving mode
            # previous: 0: stop, 1: start
            control_status = 0 
            # fn_c = '/home/connau/srp/birdview/hummer_path/execution_tag.txt'
            # with open(fn_c, "r") as f:
            with open(traj_control, "r") as f:
                line = f.readline()
            data_control = int(line.strip())
            # if (data_control==1):
                # control_status = 1
            # elif (data_control==0):
            #     control_status = 2
            control_status = data_control
            
            trailer_data = [d1, d2, theta_est, control_status]
            # trailer_data = [d1, d2, prev_filtered, control_status]
            # print("uwb dx2/theta/control", trailer_data)
            ros_node.call_back_J(trailer_data)
            
            data_ublox, data_veh_gps = load_ublox()

            combined_gps_data = [data_ublox[0], data_ublox[1], data_veh_gps[0], data_veh_gps[1], data_veh_gps[2]]
            # print("ublox+veh gps/heading", combined_gps_data)
            # ros_node.call_back_J_ublox_veh_gps(combined_gps_data)

            # 5. update value to ros
            # data_all = [trailer_data, combined_gps_data]
            # ros_node.call_back_J_all(data_all)
           
           
            
            # --- LOGGING ---
            # print("mqtt msg:", msg.topic, time_ms, addr, dis)
            
            # fn = 'uwb_recent.txt'
            fn = '/home/connau/srp/birdview/hummer_path/uwb_recent_fail.txt'
            if tag1 in msg.topic: fn = fn1
            elif tag2 in msg.topic: fn = fn2
            
            with open(fn, 'w') as f:
                f.write('[%d,%.3f]' % (time_ms, dis))

            filter_rad = math.radians(prev_filtered)
            tx = Lt * np.sin(filter_rad)
            ty = yh - Lt * np.cos(filter_rad)
    
            with open(fn3, 'w') as f:
                f.write('[%d,%.3f,%.3f,%.3f]' % (time_ms, prev_filtered, float(tx), float(ty)))
                
        except Exception as e:
            print(f'mqtt msg err: {e}')

    return on_message

################



# traj_filename = '/home/connau/srp/birdview/hummer_path/pathx5_can_heading.txt'
# traj_filename = '/home/connau/srp/birdview/hummer_path/path_x5_fake2.txt'
# traj_filename = '/home/connau/srp/birdview/hummer_path/path_x5_fake200.txt'
# traj_filename = '/home/connau/srp/birdview/hummer_path/path_x5_fake200.txt'

# traj_filename = '/home/connau/srp/birdview/hummer_path/back200.txt'
# traj_filename = '/home/connau/srp/birdview/hummer_path/back200_curve.txt'
traj_filename = '/home/connau/srp/birdview/hummer_path/pathx200_dan.txt'
# traj_filename = '/home/connau/srp/birdview/hummer_path/pathx200_dan_trailer_heading.txt'
traj_control = "/home/connau/srp/birdview/hummer_path/execution_tag_trailer.txt"

def read_nested_list_from_file_json(filename):
    with open(filename, 'r') as file:
        content = file.read()
        nested_list = json.loads(content)
        # print(nested_list)
        return nested_list

class VehicleCommanderSRP(Node):

    def __init__(self, argv):
        super().__init__('VehicleCommanderSRP')

        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = "can0"
        can.rc['bitrate'] = 500_000

        self.is_traj_track_ready = False
        self.points = None
        self.start_stop = 0

        self.can_Bus = can.Bus()
        self.dbc = cantools.database.load_file('/home/connau/ConnAu/DBC-ARXML/TrailerReverse_PoC.dbc') # defining dbc
        
        self.input_MSG_195 = self.dbc.get_message_by_frame_id(195) # defining frame id 226
        self.input_MSG_209 = self.dbc.get_message_by_frame_id(209) # defining frame id 209
        # self.input_MSG_210 = self.dbc.get_message_by_frame_id(210) # defining frame id 210
        # self.input_MSG_211 = self.dbc.get_message_by_frame_id(211) # defining frame id 211
                

        self.output_MSG_226 = self.dbc.get_message_by_frame_id(226) # defining frame id 226
        self.output_MSG_234 = self.dbc.get_message_by_frame_id(234) # defining frame id 234
        self.output_MSG_233 = self.dbc.get_message_by_frame_id(233) # defining frame id 233
        self.output_MSG_231 = self.dbc.get_message_by_frame_id(231) # defining frame id 231
        self.output_MSG_230 = self.dbc.get_message_by_frame_id(230) # defining frame id 230
        self.output_MSG_229 = self.dbc.get_message_by_frame_id(229) # defining frame id 229
        self.output_MSG_228 = self.dbc.get_message_by_frame_id(228) # defining frame id 228
       
        ## add ublox/vehicle GPS 
        # BO_ 238 VehGPSHdg: 3 Vector__XXX
        #  SG_ vehGPS_Heading : 7|19@0+ (0.001,0) [0|524.287] "deg" Vector__XXX

        # BO_ 237 VehGPSLatLon: 8 Vector__XXX
        #  SG_ vehGPS_Longitude : 39|31@0- (1,0) [-1073741824|1073741823] "mas" Vector__XXX
        #  SG_ vehGPS_Latitude : 7|30@0- (1,0) [-536870912|536870911] "mas" Vector__XXX

        # BO_ 236 ROSAdtlInfo2: 5 Vector__XXX
        #  SG_ uBlox_Heading : 23|19@0+ (0.001,0) [0|524.287] "deg" Vector__XXX
        #  SG_ HitchAngleRaw : 7|13@0- (0.1,0) [-409.6|409.5] "deg" Vector__XXX

        # BO_ 235 ROSAdtlInfo1: 8 Vector__XXX
        #  SG_ uBlox_Longitude : 39|31@0- (1,0) [-1073741824|1073741823] "mas" Vector__XXX
        #  SG_ uBlox_Latitude : 7|30@0- (1,0) [-536870912|536870911] "mas" Vector__XXX

        self.output_MSG_235 = self.dbc.get_message_by_frame_id(235) 
        self.output_MSG_237 = self.dbc.get_message_by_frame_id(237) 
        self.output_MSG_238 = self.dbc.get_message_by_frame_id(238) 
        
        self.data_dict_235 = {"uBloxGPS_inv": -1, "uBlox_Latitude" : 0, "uBlox_Longitude" : 0}
        self.data_dict_237 = {"VehGPS_inv": -1, "vehGPS_Latitude":0, "vehGPS_Longitude" : 0}
        self.data_dict_238 = {"vehGPS_Heading" : 0.0}
        

        # initializing TrailerReverseRequestStatus to 0 (Inactive)
        self.data_dict_226 = {"VehUWBSens2_reading":0.0, "VehUWBSens1_reading":0.0, "HitchInclination":0.0, "HitchAngle":0.0, "TrailerReverseRequestStatus":0}
        
        self.data_dict_231 = {"TrailerUWBsens_z" : 1.0, "TrailerUWBsens_y" : 2.54 , "TrailerUWBsens_x" : -2.56}
        self.data_dict_230 = {"VehUWBsens2_z" : 1.0, "VehUWBsens2_y" : 2.54 , "VehUWBsens2_x" : -2.56, "VehUWBsens1_z": 1.1, "VehUWBsens1_y": 0.8, "VehUWBsens1_x": -1.6}
        self.data_dict_229 = {"TrailerHtch2Axle3" : 1.0, "TrailerHtch2Axle2" : 1.0 , "TrailerHtch2Axle1" : 1.0}
        self.data_dict_228 = {"VehRrAxl2HtchBall" : 1.1, "TrailerWidth" : 2.54 , "TrailerWheelbase" : 0.56, "TrailerPresent":1, "TrailerLength":5.5}
     

        data_228 = self.output_MSG_228.encode(self.data_dict_228)
        msg_228 = can.Message(arbitration_id=self.output_MSG_228.frame_id, data=data_228, is_extended_id = False)
        data_229 = self.output_MSG_229.encode(self.data_dict_229)
        msg_229 = can.Message(arbitration_id=self.output_MSG_229.frame_id, data=data_229, is_extended_id = False)
        data_231 = self.output_MSG_231.encode(self.data_dict_231)
        msg_231 = can.Message(arbitration_id=self.output_MSG_231.frame_id, data=data_231, is_extended_id = False)
        data_230 = self.output_MSG_230.encode(self.data_dict_230)
        msg_230 = can.Message(arbitration_id=self.output_MSG_230.frame_id, data=data_230, is_extended_id = False)
        try:
            self.can_Bus.send(msg_228)
            self.can_Bus.send(msg_229)
            self.can_Bus.send(msg_231)
            self.can_Bus.send(msg_230)
            print(f"Message sent on {self.can_Bus.channel_info}")
        except can.CanError:
            print("Message NOT sent")
            print(f"{can.CanError}")


        
        self.write_ManeuverControl()

        self.timer = self.create_timer(1.0, self.send_new_traj)

        # uBlox refresh at 10 Hz — same executor, no nested event loop
        self.ublox_timer = self.create_timer(0.1, self.send_ublox)


        ## test can reading
        # read_can_bus(self, 195)
        # self.ublox_timer = self.create_timer(0.1, self.read_can_bus(195))

        # self._data_lock = threading.Lock()

    def write_ManeuverControl(self): # frame id 226
        # if self.is_traj_track_ready == False:
        #     self.data_dict_226["TrailerReverseRequestStatus"] = 4
        # else:
        #     self.data_dict_226["TrailerReverseRequestStatus"] = 3
        
        data_226 = self.output_MSG_226.encode(self.data_dict_226)
        msg_226 = can.Message(arbitration_id=self.output_MSG_226.frame_id, data=data_226, is_extended_id = False)
        try:
            self.can_Bus.send(msg_226)
            print(f"UWB Message sent on {self.can_Bus.channel_info}, {self.data_dict_226}")
        except can.CanError:
            print("UWB Message NOT sent")
            print(f"{can.CanError}")

        ## add ublox
        # data_235 = self.output_MSG_235.encode(self.data_dict_235)
        # msg_235 = can.Message(arbitration_id=self.output_MSG_235.frame_id, data=data_235, is_extended_id = False)
        # try:
        #     self.can_Bus.send(msg_235)
        #     print(f"Ublox Message sent on {self.can_Bus.channel_info}, {self.data_dict_235}")
        # except can.CanError:
        #     print("Ublox Message NOT sent")
        #     print(f"{can.CanError}")

        # data_237 = self.output_MSG_237.encode(self.data_dict_237)
        # msg_237 = can.Message(arbitration_id=self.output_MSG_237.frame_id, data=data_237, is_extended_id = False)
        # try:
        #     self.can_Bus.send(msg_237)
        #     print(f"Veh GPS Message sent on {self.can_Bus.channel_info}, {self.data_dict_237}")
        # except can.CanError:
        #     print("Veh GPS Message NOT sent")
        #     print(f"{can.CanError}")

        # data_238 = self.output_MSG_238.encode(self.data_dict_238)
        # msg_238 = can.Message(arbitration_id=self.output_MSG_238.frame_id, data=data_238, is_extended_id = False)
        # try:
        #     self.can_Bus.send(msg_238)
        #     print(f"Veh heading Message sent on {self.can_Bus.channel_info}, {self.data_dict_238}")
        # except can.CanError:
        #     print("Veh heading Message NOT sent")
        #     print(f"{can.CanError}")

      # 2. Check if the received Frame ID matches your target message (228)
      # SG_ TrajectoryReceived : 3|1@0+ (1,0) [0|1] "" Vector__XXX
      # SG_ TrailerReverseStatus


    def send_ublox(self):
        # rclpy timer callback MUST be sync — executor runs it on the
        # spin thread. Replacing the old async def / asyncio.run mess.
        data_ublox, data_veh_gps = load_ublox()
        self.data_dict_235['uBloxGPS_inv'] = 0              # 0 = Valid
        self.data_dict_235['uBlox_Latitude']  = data_ublox[0]
        self.data_dict_235['uBlox_Longitude'] = data_ublox[1]

        data_235 = self.output_MSG_235.encode(self.data_dict_235)
        msg_235 = can.Message(
            arbitration_id=self.output_MSG_235.frame_id,
            data=data_235,
            is_extended_id=False,
        )
        try:
            self.can_Bus.send(msg_235)
            self.get_logger().info(
                f"uBlox sent: lat={data_ublox[0]} lon={data_ublox[1]}"
            )
        except can.CanError as e:
            self.get_logger().error(f"uBlox send failed: {e}")


      

    def read_can_bus(self, msg_id):
        print("Listening for CAN messages...")
        # try:
        while True:
            # 1. Block and wait for the next incoming CAN message
            # timeout=1.0 prevents the script from freezing indefinitely if nothing is sent
            msg = self.can_Bus.recv(timeout=1.0)
            
            if msg is None:
                continue  # No message received within the timeout window
                
            if msg.arbitration_id == msg_id and msg_id==195: # trailerReverseFeedbacks
                try:
                    # 3. Decode the raw payload bytes using your DBC message definition
                    decoded_data = self.input_MSG_195.decode(msg.data)
                    path_rec = decoded_data["TrajectoryReceived"]
                    sta_rec = decoded_data["TrailerReverseStatus"]
                    
                    # 4. Access your signals from the resulting dictionary
                    print("\n--- Received Frame ---", msg_id, path_rec, sta_rec)
                    
                except Exception as decode_error:
                    print(f"Failed to decode payload: {decode_error}")
            

            elif msg.arbitration_id == msg_id and msg_id==209: # vehControlSta
                try:
                    # 3. Decode the raw payload bytes using your DBC message definition
                    decoded_data = self.input_MSG_209.decode(msg.data)
                    speed = decoded_data["Velocity_Stat"] #[0|40.95] "m/s"
                    wheel = decoded_data["StrgWhlAng_Stat"] #[-960|1006.05] "deg"
                    
                    # 4. Access your signals from the resulting dictionary
                    print("\n--- Received Frame ---", msg_id, speed, wheel)
                    
                except Exception as decode_error:
                    print(f"Failed to decode payload: {decode_error}")


            elif msg.arbitration_id == msg_id and msg_id==235: # ublox
                try:
                    # 3. Decode the raw payload bytes using your DBC message definition
                    decoded_data = self.input_MSG_235.decode(msg.data)
                    

                    # 4. Access your signals from the resulting dictionary
                    print("\n--- test self-loop Received Frame ---", decoded_data)
                    
                except Exception as decode_error:
                    print(f"Failed to decode payload: {decode_error}")


            # elif msg.arbitration_id == msg_id and msg_id==210: #imu
            #     try:
            #         # 3. Decode the raw payload bytes using your DBC message definition
            #         decoded_data = self.input_MSG_210.decode(msg.data)
            #         Ay_Stat = decoded_data["Ay_Stat"] 
            #         Ax_Stat = decoded_data["Ax_Stat"] 
                    
            #         # 4. Access your signals from the resulting dictionary
            #         print("\n--- Received Frame ---", msg_id, Ax_Stat, Ay_Stat)
                    
            #     except Exception as decode_error:
            #         print(f"Failed to decode payload: {decode_error}")

            # elif msg.arbitration_id == msg_id and msg_id==211: #wheel speed
            #     try:
            #         decoded_data = self.input_MSG_211.decode(msg.data)
            #         print("\n--- Received Frame ---", msg_id, path_rec, sta_rec)
                    
            #     except Exception as decode_error:
            #         print(f"Failed to decode payload: {decode_error}")

                    
        # except KeyboardInterrupt:
            # print("\nStopped listening to CAN bus.")




    ### Justin
    def call_back_J(self, msg): # PLACEHOLDER .CALLBACK FUNCTION FOR JUSTIN TO BRING SENSOR DATA

        self.data_dict_226['VehUWBSens2_reading'] = msg[0]
        self.data_dict_226['VehUWBSens1_reading'] = msg[1]
        self.data_dict_226['HitchAngle'] = msg[2]
        self.data_dict_226["TrailerReverseRequestStatus"] = msg[3]

        valid = 1
        # if (msg[0]==0.0 or msg[1]==0.0):
        #     self.data_dict_226['VehUWBSens1_reading'] = 1    
        #     self.data_dict_226['VehUWBSens2_reading'] = 1    
        #     self.data_dict_226['HitchAngle'] = 1        
        #     valid = 0
        
        # print("validate ros receive UWB data 0804 (log before flip)", msg, "valid", valid)
        self.write_ManeuverControl()


    def call_back_J_ublox_veh_gps(self, msg): # PLACEHOLDER .CALLBACK FUNCTION FOR JUSTIN TO BRING SENSOR DATA
        # with self._data_lock:
        self.data_dict_235['uBloxGPS_inv'] = 0
        self.data_dict_235['uBlox_Latitude'] = msg[0]
        self.data_dict_235['uBlox_Longitude'] = msg[1]
        self.data_dict_237['vehGPS_Latitude'] = msg[2]
        self.data_dict_237['vehGPS_Longitude'] = msg[3]
        self.data_dict_237['VehGPS_inv'] = -1
        self.data_dict_238['vehGPS_Heading'] = msg[4]

        self.write_ManeuverControl()


    def call_back_J_all(self, msg):
        self.data_dict_226['VehUWBSens2_reading'] = msg[0][0]
        self.data_dict_226['VehUWBSens1_reading'] = msg[0][1]
        self.data_dict_226['HitchAngle'] = msg[0][2]
        self.data_dict_226["TrailerReverseRequestStatus"] = msg[0][3]

        self.data_dict_235['uBloxGPS_inv'] = 0
        self.data_dict_235['uBlox_Latitude'] = msg[1][0]
        self.data_dict_235['uBlox_Longitude'] = msg[1][1]
        self.data_dict_237['vehGPS_Latitude'] = msg[1][2]
        self.data_dict_237['vehGPS_Longitude'] = msg[1][3]
        self.data_dict_237['VehGPS_inv'] = -1
        self.data_dict_238['vehGPS_Heading'] = msg[1][4]


        
        self.write_ManeuverControl()

        # self.read_can_bus(195)
        # self.read_can_bus(209)

        ## check_path_update (load and check if points==nuw_points in below function)
        self.send_new_traj()



    ########


    def send_new_traj(self):
        print("send new trajectory triggered")
        points = self.points
        try:
            new_points = read_nested_list_from_file_json(traj_filename)
            # print("new path loaded")
        except json.JSONDecodeError:
            print(f"Error: The file '{traj_filename}' is empty or contains invalid JSON.")
            return
        except FileNotFoundError:
            print(f"Error: The file '{traj_filename}' was not found.")
            return
        if points == new_points: ## only send update path
            return
        points = new_points
        self.points = points
        # self.get_logger().info(f"Got points from Justinapp: {points}")
        traj_x, traj_y, traj_h = [], [], []
        for i, pt in enumerate(points):
            traj_x.append(pt[0])
            traj_y.append(pt[1])
            # h_math = np.deg2rad(90.0 - pt[2])
            traj_h.append(pt[2]) # must be in deg

        self.is_traj_track_ready = False
        
        # setting TrailerReverseRequestStatus to 4 (SendingTrajectory)
        # if self.is_traj_track_ready == False:
        self.data_dict_226["TrailerReverseRequestStatus"] = 4
        # else:
            # self.data_dict_226["TrailerReverseRequestStatus"] = 3
        
        self.write_ManeuverControl()

        with open(traj_control, "w") as f:
            f.write("3")

        trailer_des = [0.0, 0.0, 0.0]
        msg_len = 200
        
        time.sleep(0.1)

        # send trajector
        traj_maxID = int(len(traj_x)-1)
        if traj_maxID > 255:
            traj_maxID = 255

        ## add padding for 1st and last msg (loop=5)
        for i in range(len(traj_x)):    

            if i==msg_len-1:
                trailer_des = [traj_x[i], traj_y[i], traj_h[i]]

            if i > 255:
                break

            ## front/end padding x5
            loop = 1
            if i==0 or i==msg_len-1:
                loop = 5
            
            for j in range(loop):

                data_dict_234 = {"InitialTrajectory_y":traj_y[i], "InitialTrajectory_x":traj_x[i], "InitialTrajectory_MaxID":traj_maxID, "InitialTrajectory_ID":i, "InitialTrajectory_Heading":traj_h[i]}
                data = self.output_MSG_234.encode(data_dict_234)
                msg = can.Message(arbitration_id=self.output_MSG_234.frame_id, data=data, is_extended_id = False)
                # if i==len(traj_x)-2:
                    # self.data_dict_233 = {"TrailerDestination_y" : traj_y[i], "TrailerDestination_x" : traj_x[i], "TrailerDestination_Heading" : traj_h[i]}
                
                try:
                    self.can_Bus.send(msg)
                    print(msg)
                except can.CanError:
                    print("Message NOT sent")
                    print(f"{can.CanError}")

                time.sleep(0.02)

        time.sleep(0.1)

        # data_dict_233 = {"TrailerDestination_y" : trailer_des[1], "TrailerDestination_x" : trailer_des[0], "TrailerDestination_Heading" : trailer_des[2]}
        # data = self.output_MSG_233.encode(data_dict_233)
        # msg = can.Message(arbitration_id=self.output_MSG_233.frame_id, data=data, is_extended_id = False)
        # try:
        #     self.can_Bus.send(msg)
        #     print(f"Message sent on {self.can_Bus.channel_info}")
        # except can.CanError:
        #     print("Message NOT sent")
        #     print(f"{can.CanError}")

        # time.sleep(0.1)

        # init as 0, when UWB is ready, send 4, wait for feedback (high) then set back to 0 and send trajectory. when trajectory received, send 1 trigger move  

        # is_traj_sent_success = False
        # while not is_traj_sent_success:
        #     with can.Bus() as bus:
        #         for frame_msg in bus:
        #             frame_id = frame_msg.arbitration_id
        #             if frame_id == 195:
        #                 decoded = self.dbc.decode_message(frame_msg.arbitration_id, frame_msg.data)
        #                 if decoded['TrajectoryReceived'] == 1:
        #                     is_traj_sent_success = True
        #                     print("rec traj by autobox")
        #                     break
        # time.sleep(0.5)

        
        self.write_ManeuverControl()

        time.sleep(0.1)

        self.is_traj_track_ready = True
        self.data_dict_226["TrailerReverseRequestStatus"] = 3
        # is_traj_track_ready = False
        # while not is_traj_track_ready:
            # print("waiting autobox return path-rec feedback on 195:TrailerReverseStatus")
        # if True:
        #     with can.Bus() as bus:
        #         for frame_msg in bus:
        #             frame_id = frame_msg.arbitration_id
        #             if frame_id == 195:
        #                 decoded = self.dbc.decode_message(frame_msg.arbitration_id, frame_msg.data)
        #                 if decoded['TrailerReverseStatus'] == 1:
        #                 # if True:
        #                     self.is_traj_track_ready = True
        #                     print("ready-- reset to 0--***")
        #                     with open(traj_control, "w") as f:
        #                         f.write("0")
        #                     break

    def reverse_req_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)

    publisher = VehicleCommanderSRP(sys.argv)

    ## Justin ##
    uwb_data = {
        tag1: 0.0,
        tag2: 0.0  
    } 
    
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = make_on_message(publisher, uwb_data)

    client.connect(gateway_ip, port, 60)
    client.loop_start() # move MQTT to background thread (non-blocking)

    try:
        rclpy.spin(publisher)
        # client.loop_forever() # need to check conflict between two spins

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:

         # Clean up MQTT thread 
        client.loop_stop()
        client.disconnect()

        publisher.destroy_node()
        rclpy.shutdown()

    ##############

if __name__ == '__main__':
    main(sys.argv)






