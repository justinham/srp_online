import rclpy
from rclpy.node import Node

from fusion.msg import GPS
from fusion.msg import Acc
from fusion.msg import Vel
from fusion.msg import Track

import cantools
import can
import sys
import time
import math

from .can_to_message import *

import rclpy
import rclpy.duration


"""
General Notes:
    1.  Currently using the time at message creation for timestamp, which may result in some in-accuracy
        This may need to be changed for using Aquisition timestamp, but I have no idea as of now how it works as it is reported
        in ms, and ranges from 0-2047 and is only reported in a single frame per burst
    2. Sensor Ids: 0 - Infra, 1 - LRR, 2 - FCM, 3- SRRLF, 4 - SRRLR

"""

time_factor = 1

class DataPublisher(Node):
    def __init__(self, args):
        super().__init__('data_publisher')
        self.publisher_track = self.create_publisher(Track, 'track', 100)
        self.publisher_gps = self.create_publisher(GPS, 'gps', 100)
        self.publisher_vel = self.create_publisher(Vel, 'vel', 100)
        self.publisher_acc = self.create_publisher(Acc, 'Acc', 100)
        self.publisher_visual = self.create_publisher(Marker, "object_marker", 50)

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
        if (len(args) > 2 and (args[2] is None or args[2] == "MY24")) or len(args) < 3:
            self.convert_8 = can_to_message_24Lyriq_8
            self.db_can8 = cantools.database.load_file('/home/kzb068/Code/ConnAu/Data/SampleData/DBC-ARXML/MY24CAN8.dbc')
            self.db_can5 = cantools.database.load_file('/home/kzb068/Code/ConnAu/Data/SampleData/DBC-ARXML/MY24CAN5.dbc')
            self.speed_heading = '2D7'
            self.time_day = '2D6'
            self.lat_lon = '2D5'
        elif (len(args) > 2 and (args[2] is None or args[2] == "MY22")):
            self.convert_8 = can_to_message_22Escalade_8
            self.db_can8 = cantools.database.load_file('/home/kzb068/Code/ConnAu/Data/SampleData/DBC-ARXML/MY22CAN8.dbc')
            self.db_can5 = cantools.database.load_file('/home/kzb068/Code/ConnAu/Data/SampleData/DBC-ARXML/MY22CAN5.dbc')
            self.speed_heading = '26C'
            self.time_day = '26B'
            self.lat_lon = '26A'
        if len(args) > 1:
            self.get_logger().info('Openening file: "%s"' % args[1])
            self.f_in = open(args[1], 'r')
        else:
            self.get_logger().info('No Input file given')
            exit(0)
        

        file_path = args[1][:-4]
        self.track_file = open(file_path + "-tracks.txt", 'w+')
        self.gps_file = open(file_path + "-gps.txt", 'w+')
        
        self.track_file.write("timestamp(s),sensor_id,track_id,track_status,confidence,object_type,lat offset from ego (m),lon offset from ego (m),lat_vel w.r.t ego (m/s),lon_vel w.r.t. ego (m/s),theta w.r.t. ego,theta_valid,theta_cov,lat covariance,lon covariance,lat_vel covariance,lon_vel covariance\n")
        self.gps_file.write("timestamp (s),lat (ms arc),lon (msg arc),heading (deg),calculated speed (km/h),year,day of year,time of day in ms\n")

        self.publish()

    """
    Reads the file, which must be in a format as created by the FileTransformer.py, 
    publishes the messages created by the parsing method
    """
    def publish(self):

        # Read the header line
        line = self.f_in.readline()
        c = 0

        # GPS Variables
        first_gps_time = 0
        gps_msg_recv = 0
        avg_gps_time = 0
        gps_lat = 0
        gps_lon = 0
        gps_heading = 0
        gps_speed = 0
        gps_year = 0
        gps_day = 0
        gps_ms = 0

        # Start the processing timer for the first message
        t1 = time.perf_counter_ns()
        while True:
            
            # Read data line
            line = self.f_in.readline()
            
            if not line:
                break 


            vals = line.split()
            
            # Print an time message to keep track of progress through file in 'Sim' Time
            self.i += 1
            if self.i >= 1000:
                self.get_logger().info(f'\'Sim Time\': {vals[0]}, Lost time: {str(c)}')
                self.i = 0
            
            # An interesting case of message 54b not having data or a translation, as such we ignore it.
            # potential issue when not working with MY24 Lyriq, so we may need to address this issue differently
            if len(vals) < 5:
                if vals[2] == '8' and vals[3] != '54b':
                    print(vals)
                    exit(0)
            elif vals[2] == '5':
                data = vals[4]
                frame_id = vals[3]
                if 'x' in frame_id:
                    
                    """
                    This is CanXtd bit messages and none should be important to our work
                    """
                    continue

                if frame_id in ['788', '7ED', '7DF', '7EA', '40E', '40C', '7E6', '41B', '7E9', '7EE', '14DA97F4x', '14DAF497x', '14DA80F3x']: # Messages that give errors
                    """
                    788 is a multiframe message that we do not need and even vehicle spy didn't translate it
                    7E6/7E9/7EA/7ED/7EE/7DF/40E/41B is an unknown message, not in ARXML/DBC
                    14DA97F4x, 14DAF497x, CAN Xtd 29 bit messages and not in databse
                    """
                    continue
                binary_data = bytes.fromhex(data)
                decoded = self.db_can5.decode_message(int(frame_id, 16), binary_data)
                if float(vals[0]) - first_gps_time > 0.05:
                    gps_msg_recv = 0
                    avg_gps_time = 0
                    first_gps_time = 0
                if frame_id == self.speed_heading: # Speed and Heading
                    if first_gps_time == 0:
                        first_gps_time = float(vals[0])
                    avg_gps_time += float(vals[0])
                    gps_heading = decoded['GPSV_PPSHdg']
                    gps_speed = decoded['GPSV_PPSCalcdSpd']
                    gps_msg_recv += 1
                elif frame_id == self.time_day: # Time and Day
                    if first_gps_time == 0:
                        first_gps_time = float(vals[0])
                    avg_gps_time += float(vals[0])
                    gps_year = decoded['GPST_PPSCldrYr']
                    gps_day = decoded['GPST_PPSCldrDy']
                    gps_ms = decoded['GPST_PPSTmOfDy']
                    gps_msg_recv += 1
                elif frame_id == self.lat_lon: # Lat and Lon
                    if first_gps_time == 0:
                        first_gps_time = float(vals[0])
                    avg_gps_time += float(vals[0])
                    gps_lat = decoded['GPSC_PPSLat']
                    gps_lon = decoded['GPSC_PPSLong']
                    gps_msg_recv += 1
                
                if gps_msg_recv == 3:
                    self.gps_file.write(f"{round((avg_gps_time/3.0),6)}, {gps_lat/3600000.0}, {gps_lon/3600000.0}, {gps_heading / 180.0 * math.pi}, {gps_speed}, {gps_year}, {gps_day}, {gps_ms}\n")
                    g = GPS()
                    g.timestamp = self.get_clock().now().to_msg()
                    g.latitude = gps_lat/3600000.0
                    g.longitude = gps_lon/3600000.0
                    g.heading = (gps_heading / 180.0 * math.pi)
                    g.speed = gps_speed
                    g.valid = True
                    g.heading_valid = True

                    self.publisher_gps.publish(g)
                    gps_msg_recv = 0
                    avg_gps_time = 0
                    first_gps_time = 0

            else:

                # Convert the messages to ROS messages from CAN
                msgs = self.convert_8(self, vals[3], vals[4])

                # Take second time reading and calculate the number of elapsed seconds
                t2 = time.perf_counter_ns()
                tdif = (t2-t1)/1000000000

                
                # If we did not overrun the time until it needed to be published, sleep the difference
                if (float(vals[1])*time_factor - tdif) > 0:
                    time_to_sleep = (float(vals[1])*time_factor - tdif) #* 0.85
                    time.sleep(time_to_sleep)
                    pass
                else:
                    # Lost time, total amount of time we are falling behind due to calculations taking too long
                    c += (float(vals[1])- tdif)
                
                # Take updated time stamp for timing calc
                t1 = time.perf_counter_ns()

                for m in msgs:
                    
                    # Need to ensure no covariance value is 0
                    
                    if m.covariance[0] <= 0:
                        m.covariance[0] = 0.000001 #lat cov
                    if m.covariance[5] <= 0:
                        m.covariance[5] = 0.000001 #lon cov
                    if m.covariance[10] <= 0:
                        m.covariance[10] = 0.000001 #theta cov
                    if m.covariance[15] <= 0:
                        m.covariance[15] = 0.000001 #lat vel cov
                    
                    if m.theta_cov == 0:
                        m.theta_cov = 0.000001 #lon vel cov
                    
                    self.print_track(m, vals[0])
                    #if m.confidence < 2:
                    #    continue

                    # Publish message
                    self.publisher_track.publish(m)
           

        exit(0)

    def print_track(self, t, time):
        out = f"{time},{t.sensor_id},{t.track_id},{t.track_status},{t.confidence},{t.object_type},{t.lat},{t.lon},{t.lat_vel},{t.lon_vel},{t.theta},{t.theta_valid},{t.theta_cov},{t.covariance[0]},{t.covariance[5]},{t.covariance[10]},{t.covariance[15]}\n"

        self.track_file.write(out)


def main(args=None):
    rclpy.init(args=args)
    
    publisher = DataPublisher(sys.argv)

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)