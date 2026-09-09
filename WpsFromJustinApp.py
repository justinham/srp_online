import json

import rclpy
from rclpy.node import Node
from fusion.msg import TripleVectorWps
from std_msgs.msg import Float32MultiArray

import numpy as np
import math
from scipy.optimize import minimize
import pandas as pd

import sys
import json
import os
import time

waypoints, headings = [], []
pts_btw_anchors = 100

IS_INPUT_INTERPOLATED = False # true for interpulated points
filename = '/home/connau/srp/birdview/hummer_path/pathx5_can_heading.txt'
# filename = '/home/connau/srp/birdview/hummer_path/path_local_den_1_stage_can_heading.txt'
is_reverse = False

def convert_angle_to_0_2pi(angle):
    if angle < 0:
        return angle + 2 * np.pi
    else:
        return angle
    
class BezierPathFitter:
    def __init__(self, waypoints, headings, directions=None):
        """
        Args:
            waypoints: list of (x, y)
            headings: list of heading angles (rad)
            directions: list of 'f' or 'r' for each segment (len = len(waypoints) - 1)
        """
        self.waypoints = np.array(waypoints)
        self.headings = np.array(headings)
        self.directions = ['f'] * (len(waypoints) - 1) if directions is None else directions

        self.curves = []
        for i in range(len(waypoints) - 1):
            p0, p1 = waypoints[i], waypoints[i + 1]
            h0, h1 = headings[i], headings[i + 1]
            direction = self.directions[i]
            curve = self.fit_quintic_bezier(p0, p1, h0, h1, direction)
            self.curves.append(curve)

    def fit_quintic_bezier(self, p0, p1, h0, h1, direction='f'):
        x0, y0 = p0
        x1, y1 = p1

        d = np.linalg.norm(np.array(p1) - np.array(p0)) / 3

        # For reverse direction, flip control point offset
        sign = 1 if direction == 'f' else -1

        p2 = (x0 + sign * d * np.cos(h0), y0 + sign * d * np.sin(h0))
        p3 = (x1 - sign * d * np.cos(h1), y1 - sign * d * np.sin(h1))

        return [p0, p2, p3, p1]

    def evaluate(self, t, segment_idx):
        P = np.array(self.curves[segment_idx])

        position = self.bezier_point(P, t)

        d1 = self.bezier_derivative(P, t, order=1)
        d2 = self.bezier_derivative(P, t, order=2)

        heading = np.arctan2(d1[1], d1[0])

        # finding curvature: κ = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2)
        curvature = (d1[0] * d2[1] - d1[1] * d2[0]) / (d1[0]**2 + d1[1]**2) ** (3/2)

        return position, heading, curvature

    def bezier_point(self, P, t):
        "using de casteljau's algorithm"
        while len(P) > 1:
            P = [(1 - t) * np.array(P[i]) + t * np.array(P[i + 1]) for i in range(len(P) - 1)]
        return P[0]

    def bezier_derivative(self, P, t, order=1):
        n = len(P) - 1
        if order == 1:
            return n * (self.bezier_point(P[1:], t) - self.bezier_point(P[:-1], t))
        elif order == 2:
            return n * (n - 1) * (self.bezier_point(P[2:], t) - 2 * self.bezier_point(P[1:-1], t) + self.bezier_point(P[:-2], t))

def read_nested_list_from_file_json(filename):
    with open(filename, 'r') as file:
        content = file.read()
        nested_list = json.loads(content)
        return nested_list

class WpsFromJustinApp(Node):
    def __init__(self):
        super().__init__('WpsFromJustinApp')
        if IS_INPUT_INTERPOLATED:
            filename = '/home/connau/srp/birdview/hummer_path/path_local_den_1_stage_can_heading.txt'
        self.points = None
        self.publisher_ = self.create_publisher(TripleVectorWps, 'interpolated_wps', 10)
        self.goal_publisher_ = self.create_publisher(Float32MultiArray, 'goal_wp', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            new_points = read_nested_list_from_file_json(filename)
        except json.JSONDecodeError:
            print(f"Error: The file '{filename}' is empty or contains invalid JSON.")
            return
        except FileNotFoundError:
            print(f"Error: The file '{filename}' was not found.")
            return
        if self.points == new_points or new_points == None:
            return
        self.points = new_points
        if not IS_INPUT_INTERPOLATED:
            waypoints, headings = [], []
            for i, pt in enumerate(self.points):
                if i == 1:
                    continue
                waypoints.append((pt[1], pt[0])) # the coordinate justin using has x and y interchanged
                if not is_reverse:
                    headings.append(pt[2] * np.pi/180)
                else:
                    headings.append(convert_angle_to_0_2pi(pt[2] * np.pi/180 + np.pi))
            self.get_logger().info(f"Got points from Justinapp: {waypoints}")
            trajectory_data = []
            directions = ['f', 'f', 'f']
            # directions = ['r', 'r', 'r']
            bezier_path = BezierPathFitter(waypoints, headings, directions)
            msg = TripleVectorWps()
            msg.plan_x, msg.plan_y, msg.plan_h = [], [], []
            for segment_idx in range(len(waypoints)-1):
                for t in np.arange(0.0, 1.0, 1/pts_btw_anchors):
                    (x, y), ref_heading, curvature = bezier_path.evaluate(t=t, segment_idx=segment_idx)
                    ref_heading = convert_angle_to_0_2pi(ref_heading)
                    msg.plan_x.append(x)
                    msg.plan_y.append(y)
                    msg.plan_h.append(ref_heading)
                    entry = [y, x, math.degrees(ref_heading)]
                    trajectory_data.append(entry)
            
            json_path = "/home/connau/srp/birdview/interpolated_wps_from_ros.txt"
            directory = os.path.dirname(json_path)
            if not os.path.exists(directory):
                os.makedirs(directory)
            with open(json_path, 'w') as f:
                json.dump(trajectory_data, f, indent=2)
            self.get_logger().info(f"Saved trajectory to {json_path}")

        else:
            msg = TripleVectorWps()
            x_vals = np.array([entry[1] for entry in self.points], dtype=float)
            msg.plan_x = x_vals.tolist()
            y_vals = np.array([entry[0] for entry in self.points], dtype=float)
            msg.plan_y = y_vals.tolist()
            heading_vals = np.array([np.radians(entry[2]) for entry in self.points], dtype=float)
            msg.plan_h = heading_vals.tolist()

        goal_msg = Float32MultiArray()
        goal_msg.data = [float(self.points[3][1]), float(self.points[3][0]), float(self.points[3][2])]
        # goal_msg.data = [float(self.points[-1][1]), float(self.points[-1][0]), float(self.points[-1][2])]
        self.publisher_.publish(msg)
        time.sleep(0.5)
        self.goal_publisher_.publish(goal_msg)
        self.get_logger().info('Publishing triple vectors')
        self.get_logger().info(f"x_vals: {msg.plan_x}")
        self.get_logger().info(f"y_vals: {msg.plan_y}")
        self.get_logger().info(f"heading_vals: {msg.plan_h}")

def main(args=None):
    rclpy.init(args=args)
    node = WpsFromJustinApp()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()