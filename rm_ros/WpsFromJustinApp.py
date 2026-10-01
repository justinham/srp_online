import sys
import json
import os
import time
import math
import numpy as np
from scipy.optimize import minimize
import pandas as pd

waypoints, headings = [], []
pts_btw_anchors = 100

IS_INPUT_INTERPOLATED = False  # True for pre-interpolated points
filename = '/home/connau/srp/birdview/hummer_path/pathx5_can_heading.txt'
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

class WpsFromJustinApp:
    def __init__(self):
        global filename
        if IS_INPUT_INTERPOLATED:
            filename = '/home/connau/srp/birdview/hummer_path/path_local_den_1_stage_can_heading.txt'
        self.points = None

    def execute_interpolation(self):
        try:
            new_points = read_nested_list_from_file_json(filename)
        except json.JSONDecodeError:
            print(f"Error: The file '{filename}' is empty or contains invalid JSON.")
            return
        except FileNotFoundError:
            print(f"Error: The file '{filename}' was not found.")
            return

        if self.points == new_points or new_points is None:
            return
        
        self.points = new_points
        trajectory_data = []

        if not IS_INPUT_INTERPOLATED:
            waypoints_list, headings_list = [], []
            for i, pt in enumerate(self.points):
                if i == 1:
                    continue
                waypoints_list.append((pt[1], pt[0]))  # Interchange x and y coordinates
                if not is_reverse:
                    headings_list.append(pt[2] * np.pi / 180)
                else:
                    headings_list.append(convert_angle_to_0_2pi(pt[2] * np.pi / 180 + np.pi))
            
            print(f"Got points from Justinapp: {waypoints_list}")
            
            directions = ['f', 'f', 'f']
            bezier_path = BezierPathFitter(waypoints_list, headings_list, directions)
            
            # Local output mock objects replacing the legacy ROS payload arrays
            plan_x, plan_y, plan_h = [], [], []

            for segment_idx in range(len(waypoints_list) - 1):
                for t in np.arange(0.0, 1.0, 1 / pts_btw_anchors):
                    (x, y), ref_heading, curvature = bezier_path.evaluate(t=t, segment_idx=segment_idx)
                    ref_heading = convert_angle_to_0_2pi(ref_heading)
                    plan_x.append(x)
                    plan_y.append(y)
                    plan_h.append(ref_heading)
                    entry = [y, x, math.degrees(ref_heading)]
                    trajectory_data.append(entry)
            
            json_path = "/home/connau/srp/birdview/interpolated_wps_from_ros.txt"
            directory = os.path.dirname(json_path)
            if not os.path.exists(directory):
                os.makedirs(directory)
            with open(json_path, 'w') as f:
                json.dump(trajectory_data, f, indent=2)
            
            print(f"Saved trajectory to {json_path}")
            print(f"x_vals: {plan_x}")
            print(f"y_vals: {plan_y}")
            print(f"heading_vals: {plan_h}")

        else:
            # Handle the pre-interpolated execution branch natively
            x_vals = np.array([entry[1] for entry in self.points], dtype=float).tolist()
            y_vals = np.array([entry[0] for entry in self.points], dtype=float).tolist()
            heading_vals = np.array([np.radians(entry[2]) for entry in self.points], dtype=float).tolist()
            
            print("Processing pre-interpolated points branch.")
            print(f"x_vals: {x_vals}")
            print(f"y_vals: {y_vals}")
            print(f"heading_vals: {heading_vals}")

        # Extract target goal message properties natively
        goal_wp = [float(self.points[3][1]), float(self.points[3][0]), float(self.points[3][2])]
        print(f"Current Target Goal Waypoint calculated: {goal_wp}")


def main():
    app = WpsFromJustinApp()
    print("Starting Bezier path interpolator loop (1Hz). Press Ctrl+C to exit.")
    
    # 5. Replaced the 1.0s ROS Timer loop with a standard native execution cycle
    try:
        while True:
            start_time = time.time()
            app.execute_interpolation()
            
            # Accurately time step the loop at exactly 1Hz execution frequency
            elapsed = time.time() - start_time
            sleep_time = max(0.01, 1.0 - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\nExiting path interpolator loop.")

if __name__ == '__main__':
    main()
