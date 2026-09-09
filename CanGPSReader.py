import asyncio
import can
import cantools
import sys

import rclpy
from rclpy.node import Node

from fusion.msg import GPS
from fusion.msg import Acc
from fusion.msg import Vel
from fusion.msg import Track
from std_msgs.msg import String

import cantools
import can
import sys
import time
import math

from .can_to_message import *

import rclpy
import rclpy.duration

import numpy as np

def convert_xy_to_lat_lon(ref_lat_rad, ref_lon_rad, ref_heading_rad, x_m, y_m):
    f = 0.003353
    a = 6378137
    f1 = np.sqrt(f * (2 - f))
    sin_lat = np.sin(ref_lat_rad)
    denom = np.sqrt(1 - (f1 ** 2) * (sin_lat ** 2))
    f2 = a * (1 - f1 ** 2) / (denom ** 3)
    f3 = a / denom
    N = x_m * np.cos(ref_heading_rad) - y_m * np.sin(ref_heading_rad)
    E = x_m * np.sin(ref_heading_rad) + y_m * np.cos(ref_heading_rad)
    lat_rad = ref_lat_rad + N / f2
    lon_rad = ref_lon_rad + E / (f3 * np.cos(ref_lat_rad))
    return lat_rad, lon_rad

class CanGPSReader(Node):
    def __init__(self, args):
        super().__init__('CanCapture')

        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = "can2"
        can.rc['bitrate'] = 5000000

        self.publisher_gps = self.create_publisher(GPS, 'gps_can', 100)
        self.publisher_visual = self.create_publisher(Marker, "object_marker", 50)
        self.publisher_turn_sig = self.create_publisher(String, 'ego_turn_signal', 100)

        # Publish an EGO vehicle
        ego = Marker()

        #Set the frame ID and timestamp.  See the TF tutorials for information on these.
        ego.header.frame_id = "/my_frame"
        ego.header.stamp = self.get_clock().now().to_msg()

        #Set the namespace and id for this marker.  This serves to create a unique ID
        #Any marker sent with the same namespace and id will overwrite the old one
        ego.ns = "EGO"
        ego.id = 0

        #Set the marker type.
        ego.type = 1

        #Set the marker action.  Options are ADD, DELETE, and new in rclcpp Indigo: 3 (DELETEALL)
        ego.action = 0

        # Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        ego.pose.position.x = 0.0
        ego.pose.position.y = 0.0
        ego.pose.position.z = 0.0
        ego.pose.orientation.x = 0.0
        ego.pose.orientation.y = 0.0
        ego.pose.orientation.z = 0.0
        ego.pose.orientation.w = 1.0

        #Set the scale of the marker -- 1x1x1 here means 1m on a side
        ego.scale.x = 2.0574
        ego.scale.y = 5.38226
        ego.scale.z = 1.0

        #Set the color -- be sure to set alpha to something non-zero!
        ego.color.r = 1.0
        ego.color.g = 1.0
        ego.color.b = 1.0
        ego.color.a = 1.0
        self.publisher_visual.publish(ego)
        self.i = 0

        self.f_in = None

        # Configure Function
        if (len(args) > 2 and (args[2] is None or args[2] == "Lyriq")) or len(args) < 3:
            self.db_can5 = cantools.database.load_file('/home/connau/ConnAu/DBC-ARXML/MY24CAN5.dbc')
            self.speed_heading = 727#'2D7'
            self.time_day = 726#'2D6'
            self.lat_lon = 725#'2D5'
        elif (len(args) > 2 and (args[2] is None or args[2] == "MY22")):
            self.db_can5 = cantools.database.load_file('/home/connau/ConnAu/DBC-ARXML/MY22CAN5.dbc')
            self.speed_heading = 620#'26C'
            self.time_day = 619#'26B'
            self.lat_lon = 618#'26A'

        file_path = "log"
        self.gps_file = open(file_path + "_can_gps.txt", 'w+')
        self.gps_file.write("timestamp (s),lat (ms arc),lon (msg arc),heading (deg),calculated speed (km/h),year,day of year,time of day in ms\n")

        # GPS Variables
        self.first_gps_time = 0
        self.gps_msg_recv = 0
        self.avg_gps_time = 0
        self.gps_lat = 0
        self.gps_lon = 0
        self.gps_heading = 0
        self.gps_speed = 0
        self.gps_year = 0
        self.gps_day = 0
        self.gps_ms = 0

        self.rwa = 0.0

        self.prev_heading_raw = None
        self.prev_heading_time = None
        self.heading_rate = 0.0
        self.heading_predict_dt = 0.1

        self.turn_signal = 'S'

        self.read_can()

    def publish_gps(self, msg):
        frame_id = msg.arbitration_id
        if hex(frame_id)[2:].capitalize() in ['788', '7ED', '7DF', '7EA', '40E', '40C', '7E6', '41B', '7E9', '7EE', '14DA97F4x', '14DAF497x', '14DA80F3x']: # Messages that give errors
            """
            788 is a multiframe message that we do not need and even vehicle spy didn't translate it
            7E6/7E9/7EA/7ED/7EE/7DF/40E/41B is an unknown message, not in ARXML/DBC
            14DA97F4x, 14DAF497x, CAN Xtd 29 bit messages and not in databse
            """
            return
        decoded = self.db_can5.decode_message(msg.arbitration_id, msg.data)
        if frame_id == 1490:
            turn_signal = 'S'
            is_left_turn_signal_on = decoded['TrnSigSwLtActv']
            is_right_turn_signal_on = decoded['TrnSigSwRtActv']
            if is_left_turn_signal_on == 'TRUE':
                # self.get_logger().info(f"on left: {is_left_turn_signal_on}")
                turn_signal = 'L'
            elif is_right_turn_signal_on == 'TRUE':
                # self.get_logger().info(f"on right: {is_right_turn_signal_on}")
                turn_signal = 'R'
            if turn_signal != self.turn_signal:
                msg = String()
                msg.data = turn_signal
                self.publisher_turn_sig.publish(msg)
                self.turn_signal = turn_signal

        if frame_id == 936:
            self.rwa = decoded['RdWhlAng']
            print(f"***********************{self.get_clock().now().nanoseconds / 1e9}**{self.rwa}****************************")
            # rear_rwa = decoded['RrStrgRdWhlAngAuth']
            # self.get_logger().info(f"front wheel angle: {rwa}")
        if frame_id == 1470:
            rear_rwa = decoded['RrStrgRdWhlAngAuth']
            # self.get_logger().info(f"rear wheel angle: {rear_rwa}")
        if self.get_clock().now().nanoseconds/1e9 - self.first_gps_time > 0.5:
            self.gps_msg_recv = 0
            self.avg_gps_time = 0
            self.first_gps_time = 0
        if frame_id == self.speed_heading: # Speed and Heading
            if self.first_gps_time == 0:
                self.first_gps_time = self.get_clock().now().nanoseconds/1e9
            # print(decoded)
            now_sec = self.get_clock().now().nanoseconds / 1e9
            self.avg_gps_time += now_sec
            self.gps_heading = decoded['GPSV_PPSHdg']

            self.gps_speed = decoded['GPSV_PPSCalcdSpd']
            print(self.get_clock().now().nanoseconds / 1e9, self.gps_speed)
            self.gps_msg_recv += 1

            raw_heading = self.gps_heading/180*math.pi

            if self.prev_heading_time is None:
                self.prev_heading_raw = raw_heading
                self.prev_heading_time = now_sec
            else:
                dt = now_sec - self.prev_heading_time
                if self.prev_heading_raw != raw_heading or dt >= 1.0: # since refresh rate for heading is around 1Hz
                    if dt > 1e-3:
                        dtheta = math.atan2(
                            math.sin(raw_heading-self.prev_heading_raw),
                            math.cos(raw_heading-self.prev_heading_raw)
                        )
                        self.heading_rate = dtheta/dt
                    else:
                        self.heading_rate = 0.0
                    self.prev_heading_raw = raw_heading
                    self.prev_heading_time = now_sec

        elif frame_id == self.time_day: # Time and Day
            # print(decoded)
            if self.first_gps_time == 0:
                self.first_gps_time = self.get_clock().now().nanoseconds/1e9
            self.avg_gps_time += self.get_clock().now().nanoseconds/1e9
            self.gps_year = decoded['GPST_PPSCldrYr']
            self.gps_day = decoded['GPST_PPSCldrDy']
            self.gps_ms = decoded['GPST_PPSTmOfDy']
            self.gps_msg_recv += 1
        elif frame_id == self.lat_lon: # Lat and Lon
            # print(decoded)
            if self.first_gps_time == 0:
                self.first_gps_time = self.get_clock().now().nanoseconds/1e9
            self.avg_gps_time += self.get_clock().now().nanoseconds/1e9
            self.gps_lat = decoded['GPSC_PPSLat']
            self.gps_lon = decoded['GPSC_PPSLong']
            self.gps_msg_recv += 1

        if self.gps_msg_recv == 3:

            now_sec = self.get_clock().now().nanoseconds / 1e9
            pred_heading = self.gps_heading/180*math.pi + self.heading_rate*(now_sec-self.prev_heading_time)
            pred_heading = (math.atan2(math.sin(pred_heading), math.cos(pred_heading))) % (2*math.pi)

            # print("HI")
            self.gps_file.write(f"{round((self.avg_gps_time/3.0),6)}, {self.gps_lat/3600000.0}, {self.gps_lon/3600000.0}, {self.gps_heading / 180.0 * math.pi}, {self.gps_speed}, {self.gps_year}, {self.gps_day}, {self.gps_ms}\n")
            g = GPS()
            g.timestamp = self.get_clock().now().to_msg()
            g.latitude = self.gps_lat/3600000.0
            g.longitude = self.gps_lon/3600000.0
            g.heading = self.gps_heading/180*math.pi
            # g.heading = (g.heading + 2*math.pi/180) % (2*math.pi)
            g.speed = float(self.gps_speed)
            g.valid = True
            g.heading_valid = True

            # lat_rad, lon_rad = convert_xy_to_lat_lon(g.latitude*np.pi/180, g.longitude*np.pi/180, g.heading, -0.6803350839019767, -0.7967093466840423)
            # g.latitude = lat_rad*180/np.pi
            # g.longitude = lon_rad*180/np.pi
            g.speed_f = g.speed
            g.heading_f = pred_heading
            g.rwa = -self.rwa

             ################ justin need heading for srp ##########
            self.gps_heading = g.heading * 180 / math.pi
            srp_can_h_log = open("/home/connau/srp/birdview/hummer_path/can_heading.txt", "w")
            srp_can_h_log.write("[%f,%f,%f]"%(g.latitude, g.longitude, self.gps_heading))
            srp_can_h_log.close()
            ######################################################

            self.publisher_gps.publish(g)
            self.gps_msg_recv = 0
            self.avg_gps_time = 0
            self.first_gps_time = 0

    def read_can(self):
        while(True):
            with can.Bus() as bus:
                for msg in bus:
                    self.publish_gps(msg)

def main(args=None):
    rclpy.init(args=args)

    publisher = CanGPSReader(sys.argv)

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)