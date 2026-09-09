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

class CanCapture(Node):
    def __init__(self, args):
        super().__init__('CanCapture')

        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = "can1"
        can.rc['fd'] = True
        #can.rc['bitrate'] = 500000
        #can.rc['dbitrate'] = 2000000
        #can.rc['sample-point']= 0.8
        #can.rc['dsample-point']= 0.8


        self.publisher_track = self.create_publisher(Track, 'track', 100)
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
        if (len(args) > 2 and (args[2] is None or args[2] == "Lyriq")) or len(args) < 3:
            self.convert_8 = can_to_message_24Lyriq_8
            self.db_can8 = cantools.database.load_file('/home/connau/ConnAu/DBC-ARXML/MY24CAN8.dbc')
            print("Using MY24+ DBC")

        elif (len(args) > 2 and (args[2] is None or args[2] == "MY22")):
            self.convert_8 = can_to_message_22Escalade_8
            self.db_can8 = cantools.database.load_file('../DBC-ARXML/MY22CAN8.dbc')
            print("Using MY22 DBC")
        
        file_path = "log"
        self.track_file = open(file_path + "-tracks.csv", 'w+')
  
        self.track_file.write("timestamp(s) - PC Time,sensor_id,track_id,track_status,confidence,object_type,lat offset from ego (m),lon offset from ego (m),lat_vel w.r.t ego (m/s),lon_vel w.r.t. ego (m/s),theta w.r.t. ego,theta_valid,theta_cov,lat covariance,lon covariance,lat_vel covariance,lon_vel covariance\n")
        self.read_can()

    """
    Reads the file, which must be in a format as created by the FileTransformer.py, 
    publishes the messages created by the parsing method
    """
    def publish_track(self, msg):
        # Convert the messages to ROS messages from CAN
        msgs = self.convert_8(self, msg.arbitration_id, msg.data)

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
            
            self.print_track(m, self.get_time())
            #if m.confidence < 2:
            #    continue
            if m.sensor_id > 2:
                continue
            # Publish message
            self.publisher_track.publish(m)
           


    def print_track(self, t, time):
        out = f"{time},{t.sensor_id},{t.track_id},{t.track_status},{t.confidence},{t.object_type},{t.lat},{t.lon},{t.lat_vel},{t.lon_vel},{t.theta},{t.theta_valid},{t.theta_cov},{t.covariance[0]},{t.covariance[5]},{t.covariance[10]},{t.covariance[15]}\n"

        self.track_file.write(out)

    def read_can(self):
        #t = can.BitTimingFd.from_sample_point(f_clock=80_000_000, nom_bitrate=500_000, nom_sample_point = 80.0, data_bitrate=2_000_000, data_sample_point=80.0)
        #bus = can.interfaces.PcanBus("PCAN_USBBUS2", timing = t)
        while(True):
            with can.Bus() as bus:
                for msg in bus:
                    #self.publish_track(msg)
                    #print(msg.arbitration_id, msg.data)
                    msg_id = hex(msg.arbitration_id)[2:]
                    msgs = self.convert_8(self, msg_id, msg.data)
                    #decoded = self.db_can8.decode_message(msg.arbitration_id, msg.data)
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
                        
                        self.print_track(m, self.get_clock().now().nanoseconds/1e9)
                        #if m.confidence < 2:
                        #    continue
                        if m.sensor_id > 2:
                            continue
                        # Publish message
                        self.publisher_track.publish(m)

def main(args=None):
    rclpy.init(args=args)
    
    publisher = CanCapture(sys.argv)

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
