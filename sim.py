import rclpy
from rclpy.node import Node
import rclpy.duration
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
import sys
import math
import numpy as np
from scipy.optimize import curve_fit
import time
import matplotlib.pyplot as plt

TICK_SPEED = 0.1
TIME_SCALE = 0.5
LANE_SIZE = 3.6 # meters
VEHICLE_LENGTH = 5.0 # meters
VEHICLE_WIDTH = 2.2 # meters

def calculate_angle_between_lines(A, B, C, D):
    """
    Calculate the angle between two lines defined by points A, B and C, D.

    :param A: Tuple (x1, y1) - Point A
    :param B: Tuple (x2, y2) - Point B
    :param C: Tuple (x3, y3) - Point C
    :param D: Tuple (x4, y4) - Point D
    :return: Angle in radians between the two lines
    """
    # Calculate direction vectors
    AB = (B[0] - A[0], B[1] - A[1])
    CD = (D[0] - C[0], D[1] - C[1])

    # Calculate dot product
    dot_product = AB[0] * CD[0] + AB[1] * CD[1]

    # Calculate magnitudes
    magnitude_AB = math.sqrt(AB[0]**2 + AB[1]**2)
    magnitude_CD = math.sqrt(CD[0]**2 + CD[1]**2)

    # Calculate the angle in radians
    if magnitude_AB == 0 or magnitude_CD == 0:
        raise ValueError("One of the lines is defined by a single point.")
    
    cos_theta = dot_product / (magnitude_AB * magnitude_CD)
    
    # Clamp the value to avoid numerical issues
    cos_theta = max(-1.0, min(1.0, cos_theta))
    
    angle_radians = math.acos(cos_theta)

    return angle_radians

def point_side_of_line(A, B, P):
    """
    Determine which side of the line AB the point P is on.

    :param A: Tuple (x1, y1) - Point A
    :param B: Tuple (x2, y2) - Point B
    :param P: Tuple (x, y) - Point P
    :return: 1 if P is on the left side, -1 if on the right side, 0 if on the line
    """
    x1, y1 = A
    x2, y2 = B
    x, y = P

    # Calculate the cross product
    cross_product = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)

    if cross_product > 0:
        return 1  # P is on the left side
    elif cross_product < 0:
        return -1  # P is on the right side
    else:
        return 0  # P is on the line

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def rotate_point(x, y, roation):
	 
    # Calculate the new coordinates after rotation
	x_new = x * math.cos(roation) - y * math.sin(roation)
	y_new = x * math.sin(roation) + y * math.cos(roation)
    
	return (x_new, y_new)

def calc_curve(p1, p2, p3, p4, target):
    x = np.array([ p1[0], p2[0], p3[0], p4[0]])
    y = np.array([ p1[1], p2[1], p3[1], p4[1]])

    for i in range(4):
        print(f"Point {i}: {x[i]},{y[i]}")
    
    def fun(x, a, b, c, d):
        return a *  x * x * x + b * x * x + c * x + d

    coef,_ = curve_fit(fun, x, y)
    der_one = [coef[0]*3, coef[1]*2, coef[2], 0]
    der_two = [der_one[0]*2, der_one[1], 0, 0]
    y2 = (target[0] * der_two[0]) + der_two[1]
    y1 = ((target[0]**2) * der_one[0]) + (target[0] * der_one[1]) + der_one[2]
    print(coef)
    k = y2 / ((1+y1**2)**1.5)

    """

    plt.plot(x, y, 'o', label='Original points')
    plt.plot(range(int(min(x)),int(max(x))+2,1), fun(range(int(min(x)),int(max(x))+2,1), *coef), 'r-', label='Fit')
    plt.legend()
    plt.show()
    """
    return k

class Sim(Node):
    def __init__(self, args):
        super().__init__('Sim')
        self.publisher_visual = self.create_publisher(Marker, "object_marker", 50)

        # Follows a Circle with a 20m diameter to the right
        
        # Too dense of points on the back end, results in undisireble behavior
        #self.plan = [(0,0),(1,4.3589),(2,6),(3,7.1414),(4,8),(5,8.6603),(6,9.1652),(7,9.5394),(8,9.7979),(9,9.9499),(10,10),(11,10),(15,10)]
        
        # Good Desnity of points
        #self.plan = [(0,0),(1,4.3589),(3,7.1414),(5,8.6603),(7,9.5394),(9,9.9499),(11,10),(15,10)]
        
        # Right Turn
        self.plan = [(0,0),(0.3,2.4311),(1,4.3589),(3,7.1414),(5,8.6603),(7,9.5394),(9,9.9499),(11,10),(15,10)]
        
        # Left Turn
        #self.plan = [(0,0),(0,5),(-0.3,12.4311),(-1,14.3589),(-3,17.1414),(-5,18.6603),(-7,19.5394),(-9,19.9499),(-11,20),(-15,20)]

        # Straight
        #self.plan = [(0,0), (0,3), (0,6), (0,9), (0,12), (0,15), (0,18)]

        plan_marker = Marker()

        plan_marker.header.frame_id = "/my_frame"
        plan_marker.header.stamp = self.get_clock().now().to_msg()

        plan_marker.type = 4 # Should be a line strip representing the curve
        plan_marker.action = 0 # Add/Modify
        plan_marker.id = 0
        plan_marker.ns = "Plan"
        for p in self.plan:
            plan_marker.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.0))
        plan_marker.color.r = 0.0
        plan_marker.color.g = 0.0
        plan_marker.color.b = 1.0
        plan_marker.color.a = 1.0

        plan_marker.scale.x = 3.6

        self.publisher_visual.publish(plan_marker)
        self.run(10)

    def run(self, duration):
        # id, x,y,speed,heading,max_speed,max_yaw_rate,plan,history
        vehicle = [0, 0,0,0,0,5,math.pi/4, self.plan[1:], []]
        
        # Set Vehicle Speed to max
        vehicle[3] = 5.0
        vehicles = [vehicle]

        self.to_rviz(vehicle)

        for i in range(int(duration/TICK_SPEED)):
            print(f"Time: {i*TICK_SPEED}")
            vehicles = self.tick(vehicles)
            #if i == 25:
            #    vehicles[0][4] -= math.pi/8
            time.sleep(TICK_SPEED/TIME_SCALE)

        exit(0)

    def tick(self, vehicles):
        for v in vehicles:
            v_cur_x = v[1]
            v_cur_y = v[2]
            v_cur_speed = v[3]
            v_cur_heading = v[4]
            # Close Enough to point to consider reached
            if len(v[7]) > 1:
                next_p = v[7][0]
                if abs(v_cur_x - next_p[0]) < 0.5 and abs(v_cur_y - next_p[1]) < 0.5:
                    v[7] = v[7][1:]

            # Predicted Next Point
            v_new_x, v_new_y = rotate_point(0, v_cur_speed*TICK_SPEED, v_cur_heading)
            print(f"Point Update: {v_new_x},{v_new_y}")
            v_new_x += v_cur_x
            v_new_y += v_cur_y

            x_for_calc, y_for_calc = rotate_point(0, v_cur_speed/2, v_cur_heading)
            x_for_calc += v_cur_x
            y_for_calc += v_cur_y

            v[8].append((v_cur_x,v_cur_y))
            print(v[7])
            if len(v[7]) > 1:
                heading_update = calculate_angle_between_lines((v_cur_x,v_cur_y), (x_for_calc,y_for_calc), (x_for_calc,y_for_calc), v[7][1])
                #heading_update = self.calc_new_heading(v, (x_for_calc,y_for_calc), v_cur_speed*TICK_SPEED)
                side = point_side_of_line((v_cur_x,v_cur_y), (x_for_calc,y_for_calc), v[7][0])
                if side == 1:
                    
                    
                    v[4] += heading_update/6
                    print(f"Heading Update: {heading_update}")
                    
                elif side == -1:

                    v[4] -= heading_update/6
                    print(f"Heading Update: {heading_update}")
                
                # If on line, do nothing, we are already pointing at target

            v[1] = v_new_x
            v[2] = v_new_y
            #print(v)
            self.to_rviz(v)

            
        return vehicles
    
    def calc_new_heading(self, v, v_new_p, dist):
        k = calc_curve((v[1],v[2]), v_new_p, v[7][0], v[7][1], target=v_new_p)
        print(k)
        return k * dist
    
    def to_rviz(self, v):
        marker = Marker()

        marker = Marker()
        marker.header.frame_id = "/my_frame"
        marker.header.stamp = self.get_clock().now().to_msg()

        #Set the namespace and id for this marker.  This serves to create a unique ID
        #Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "Vehicles"
        marker.id = int(v[0])

        #Set the marker type.
        marker.type = 2

        #Set the marker action.  Options are ADD, DELETE, and new in rclcpp Indigo: 3 (DELETEALL)
        marker.action = 0

        marker.lifetime = rclpy.duration.Duration(seconds=TICK_SPEED/TIME_SCALE).to_msg()

        # Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = float(v[1])
        marker.pose.position.y = float(v[2])
        marker.pose.position.z = 0.0
        
        q = quaternion_from_euler(0.0, 0.0, v[4])

        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        #Set the scale of the marker -- 1x1x1 here means 1m on a side

        marker.type = 1
        marker.scale.x = VEHICLE_WIDTH
        marker.scale.y = VEHICLE_LENGTH
        marker.scale.z = 1.0

        #Set the color -- be sure to set alpha to something non-zero!
        marker.color.a = 0.50
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        self.publisher_visual.publish(marker)
        history = Marker()

        history.header.frame_id = "/my_frame"
        history.header.stamp = self.get_clock().now().to_msg()

        history.type = 4 # Should be a line strip representing the curve
        history.action = 0 # Add/Modify
        history.id = 0
        history.ns = "History"
        for p in v[8]:
            history.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.2))
        history.color.r = 1.0
        history.color.g = 0.0
        history.color.b = 0.0
        history.color.a = 1.0

        history.scale.x = VEHICLE_WIDTH

        self.publisher_visual.publish(history)


def main(args=None):
    rclpy.init(args=args)
    
    publisher = Sim(sys.argv)

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)