import rclpy
from rclpy.node import Node

import sys
import time

import csv

from .can_to_message import *

import rclpy
import rclpy.duration

from fusion.msg import Track
from fusion.msg import RemoteObjectState
from fusion.msg import TrackedObjectList


"""
General Notes:
    1.  Currently using the time at message creation for timestamp, which may result in some in-accuracy
        This may need to be changed for using Aquisition timestamp, but I have no idea as of now how it works as it is reported
        in ms, and ranges from 0-2047 and is only reported in a single frame per burst
    2. Sensor Ids: 0 - Infra, 1 - LRR, 2 - FCM, 3- SRRLF, 4 - SRRLR

"""

time_factor = 1
publish_to_fusion = False

class ComparisonVisualizer(Node):
    def __init__(self, args):
        super().__init__('ComparisonVisualizerv2')
        self.publisher_visual = self.create_publisher(Marker, "object_marker", 50)
        if(publish_to_fusion):
            self.publisher_track = self.create_publisher(Track, 'track', 100)
            self.publisher_GT = self.create_publisher(TrackedObjectList, 'GT', 100)
            self.publisher_gps = self.create_publisher(GPS, 'gps', 100)

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
        ego.scale.x = 2.0
        ego.scale.y = 3.0
        ego.scale.z = 1.0

        #Set the color -- be sure to set alpha to something non-zero!
        ego.color.r = 1.0
        ego.color.g = 1.0
        ego.color.b = 1.0
        ego.color.a = 1.0

        self.publisher_visual.publish(ego)

        self.f_in = None

        if len(args) > 1:
            self.get_logger().info('Openening file: "%s"' % args[1])
            self.f_in = open(args[1], 'r')
        else:
            self.get_logger().info('No Input file given')
            exit(0)
        
        self.visualize()

    """
    Reads the file, which must be in a format as created by the FileTransformer.py, 
    publishes the messages created by the parsing method
    """
    def visualize(self):

        # Read the header line
        line = self.f_in.readline()
        c = 0
        lost_time = 0
        gps_ms = 0

        reader = csv.reader(self.f_in)
        header = next(reader)
        lines = []
        
        for l in reader:
            lines.append(l)

        
        t1 = time.time()
        for i in range(0, len(lines)):
            c += 1
            if c >= 1000:
                self.get_logger().info(f'\'Visualized Time\': {lines[i][0]}, Lost time: {str(lost_time)}')
                c = 0

            # TimeStamp in ms since epoch,sensor id,observed x offset in m,observed y offset in m,observed x velocity in m/s,observed y velocity in m/s,interpolated ground truth x in m,interpolated ground truth y in m,interpolated x velocity in m/s,interpolated y velocity in m/s
            #if int(lines[i][4]) < 2:
            #    continue
            if i == 0:
                self.to_markers(lines[i])
                if(publish_to_fusion):
                    self.to_fusion(lines[i])
                continue
            else:
                delta_t = (int(lines[i][0]) - int(lines[i-1][0]))/1000.0

            t2 = time.time()
            to_sleep = (delta_t - (t2 -t1)) * time_factor
            if to_sleep > 0:
                time.sleep(to_sleep)
            else:
                lost_time -= to_sleep
            if(publish_to_fusion):
                self.to_fusion(lines[i])
            self.to_markers(lines[i])
            t1 = time.time()
            

        exit(0)

    def to_markers(self, line):
        timestamp, sensor_id, track_id, track_status, confidence, object_type, obs_x_off, obs_y_off, obs_v_x, obs_v_y, obs_theta, obs_theta_valid, obs_theta_cov, obs_x_cov, obs_y_cov, obs_v_x_cov, obs_v_y_cov,int_x_off, int_y_off, int_v_x, int_v_y, lat_err, lon_err, v_lat_err, v_lon_err, int_heading, heading_err, ego_lat, ego_lon, ego_heading = line
        marker = Marker()
        marker.header.frame_id = "/my_frame"
        marker.header.stamp = self.get_clock().now().to_msg()

        #Set the namespace and id for this marker.  This serves to create a unique ID
        #Any marker sent with the same namespace and id will overwrite the old one
        sensor_id = sensor_id.strip()
        marker.ns = str(sensor_id)
        marker.id = int(track_id)

        #Set the marker type.
        marker.type = 2

        #Set the marker action.  Options are ADD, DELETE, and new in rclcpp Indigo: 3 (DELETEALL)
        marker.action = 0

        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()

        # Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = -float(obs_x_off)
        marker.pose.position.y = float(obs_y_off)
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        #Set the scale of the marker -- 1x1x1 here means 1m on a side
        if object_type == "True":
            marker.type = 2
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
        else:
            marker.type = 1
            marker.scale.x = 2.0
            marker.scale.y = 3.0
            marker.scale.z = 1.0

        #Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.50

        match int(sensor_id):
            case 0:
                pass
            case 1:
                marker.color.r = 1.0
            case 2:
                marker.color.b = 1.0
            case 3:
                marker.color.g = 1.0
            case 4:
                marker.color.r = 1.0
                marker.color.b = 1.0
            case _:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
        if not publish_to_fusion:
            self.publisher_visual.publish(marker)

        marker.text = str(sensor_id) + "-" + str(track_id)
        marker.ns = "text"
        marker.id = int(float(sensor_id)*1000) + int(track_id)
        marker.type = 9 # Text
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.color.g = 1.0
        marker.scale.z = 2.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 2.0
        if not publish_to_fusion:
            self.publisher_visual.publish(marker)

        gt = marker
        gt.text = "GT"
        gt.ns = "grount_truth"
        gt.id = 0
        
        gt.pose.position.x = -float(int_x_off) # Accounts for flipped Axis from EGO compared to RVIZ
        gt.pose.position.y = float(int_y_off)
        gt.pose.position.z = 0.0
        self.publisher_visual.publish(gt)

        marker.scale.z = 1.0
        gt.type = 2
        #gt.scale.x = 1.8288
        #gt.scale.y = 4.9784
        gt.scale.x = 1.0
        gt.scale.y = 1.0
        gt.scale.z = 1.0
        gt.id = 1
        gt.color.g = 1.0
        gt.color.b = 1.0
        gt.color.r = 0.0
        self.publisher_visual.publish(gt)

    def to_fusion(self, line):
        timestamp, sensor_id, track_id, track_status, confidence, object_type, obs_x_off, obs_y_off, obs_v_x, obs_v_y, obs_theta, obs_theta_valid, obs_theta_cov, obs_x_cov, obs_y_cov, obs_v_x_cov, obs_v_y_cov,int_x_off, int_y_off, int_v_x, int_v_y, lat_err, lon_err, v_lat_err, v_lon_err, int_heading, heading_err, ego_lat, ego_lon, ego_heading = line
        t = Track()
        t.timestamp = self.get_clock().now().to_msg()
        t.sensor_id = int(sensor_id)
        t.track_id = int(track_id)
        t.track_status = int(track_status)
        t.confidence = int(confidence)
        if object_type == "True":
            t.object_type = True
        else:
            t.object_type = False
        t.lat = float(obs_x_off)
        t.lon = float(obs_y_off)
        t.lat_vel = float(obs_v_x)
        t.lon_vel = float(obs_v_y)
        if obs_theta_valid == "True":
            t.theta = float(obs_theta)
            t.theta_valid = True
            t.theta_cov = float(obs_theta_cov)
        else:
            t.theta = 0.0
            t.theta_valid = False
            t.theta_cov = 100.0
        
        t.covariance[0] = float(obs_x_cov)
        t.covariance[5] = float(obs_y_cov)
        t.covariance[10] = float(obs_v_x_cov)
        t.covariance[15] = float(obs_v_y_cov)


        gt_l = TrackedObjectList()
        gt_l.timestamp = t.timestamp
        gt = RemoteObjectState()
        gt.timestamp = t.timestamp
        gt.lat = float(int_x_off)
        gt.lon = float(int_y_off)
        gt.lat_vel = float(int_v_x)
        gt.lon_vel = float(int_v_y)
        gt.theta = float(int_heading)
        gt.theta_valid = True

        gt_l.objs.append(gt)

        g = GPS()
        g.latitude = float(ego_lat)
        g.longitude = float(ego_lon)
        g.heading = float(ego_heading)

        self.publisher_gps.publish(g)
        self.publisher_GT.publish(gt_l)
        self.publisher_track.publish(t)



def main(args=None):
    rclpy.init(args=args)
    
    publisher = ComparisonVisualizer(sys.argv)

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)