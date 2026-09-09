# ./scripts/fusion_launch.sh 
 # 1986  ros2 run fusion_py VehicleCommanderSRP
 # 1987  history | grep ros2
 # 1988  ros2 run fusion_py SerialGPSReader 
 # 1989  history | grep ros2
 # 1990  ros2 run fusion_py CANGPSReader 



import numpy as np
import json
import math
import matplotlib.pyplot as plt




def intersection_tra(p0_3d, heading_start_deg, p3_3d, heading_end_deg, n=200):
    # Extract 2D positions (ignoring the 3rd dimension/heading component from your input array)
    p0 = p0_3d[:2]
    p3 = p3_3d[:2]
    
    # 1. Convert headings to 2D unit direction vectors
    # Based on your code's convention: 0 deg = +X, 90 deg = +Y (Counter-Clockwise)
    rad_start = np.radians(heading_start_deg)
    rad_end = np.radians(heading_end_deg)
    
    v_start = np.array([np.cos(rad_start), np.sin(rad_start)])
    v_end = np.array([np.cos(rad_end), np.sin(rad_end)])

    intersection_point = (p0 + p3) / 2.0   
    
    try:
        # 2. Find the intersection of the two heading lines: 
        # Line 1: p0 + t * v_start
        # Line 2: p3 + s * v_end
        # Set them equal: t * v_start - s * v_end = p3 - p0
        A = np.column_stack((v_start, -v_end))
        b = p3 - p0

        scales = np.linalg.solve(A, b)
        t, s = scales[0], scales[1]
        intersection_point = p0 + t * v_start

        print()

     
        is_ahead_of_p0 = t > 0
        is_between_points = (t > 0) and (s < 0) 
        # print("Is it ahead of p0?", t, s)    
            
        if is_ahead_of_p0:
            intersection_point = (p0 + p3) / 2.0   
            print("intersection ahead (0,0)")   
        elif is_between_points:
            intersection_point = p0 + scales[0] * v_start
            print("intersection in between")
        else:
            intersection_point = (p0 + p3) / 2.0   
            print("intersection behind target")

    
    except np.linalg.LinAlgError:
        # Handle parallel headings by placing control points halfway between endpoints
        intersection_point = (p0 + p3) / 2.0
        print("Warning: Headings are parallel. Using midpoint fallback.")

    return intersection_point

def get_heading_p_to_p0(p, p0, degrees=True):
    x, y = p
    x0, y0 = p0

    dx = x0 - x
    dy = y0 - y

    # Angle in radians in range [-pi, pi]
    rad = math.atan2(dy, dx)

    # Normalize to [0, 2*pi)
    rad_normalized = rad % (2 * math.pi)

    if degrees:
        return math.degrees(rad_normalized)  # Returns [0, 360)
    return rad_normalized





def plot_ch(points):
    # 1. Separate the list into X and Y coordinates, and Headings
    x_coords = [point[0] for point in points]
    y_coords = [point[1] for point in points]
    headings_deg = [point[2] for point in points]

    # Convert headings to radians for trigonometry functions
    headings_rad = np.radians(headings_deg)

    # 2. Calculate arrow vector components (U = delta X, V = delta Y)
    # Because +X is 0 deg and +Y is 90 deg, cos maps to X and sin maps to Y
    U = np.cos(headings_rad)
    V = np.sin(headings_rad)

    # 3. Create the scatter plot for positions
    plt.scatter(x_coords, y_coords, color='blue', marker='o', label='Data Points', zorder=3)

    # 4. Plot the heading arrows (quiver)
    # Adjust scale or angles depending on how dense your 200 points are
    plt.quiver(x_coords, y_coords, U, V, color='red', scale=20, width=0.005, 
               label='Headings (0° Front, 90° Left)', zorder=4)

    # 5. Add labels, gridlines, and a center origin
    plt.xlabel('X Axis (Front)')
    plt.ylabel('Y Axis (Left)')
    plt.title('Plot of Input Points with Headings')
    plt.axhline(0, color='black', linewidth=0.5)  # Add X-axis line
    plt.axvline(0, color='black', linewidth=0.5)  # Add Y-axis line
    plt.grid(True, linestyle='--', alpha=0.6)
    
    # Optional: Keep the axis aspect ratio equal so angles aren't visually distorted
    plt.gca().set_aspect('equal', adjustable='box')
    
    plt.legend()
    plt.show()

# rel_loc_ana_and_path_inter_trailer(50, "test_path.txt", "forward")


# import numpy as np
def compute_bezier_tra(p0, p1, p2, p3, n=200):
    t = np.linspace(0, 1, n)[:, None]

    # Position interpolation
    curve = (1 - t)**3 * p0 + \
            3 * (1 - t)**2 * t * p1 + \
            3 * (1 - t) * t**2 * p2 + \
            t**3 * p3

    # Corrected analytical derivative direction vectors
    dcurve = 3 * (1 - t)**2 * (p1 - p0) + \
             6 * (1 - t) * t * (p2 - p1) + \
             3 * t**2 * (p3 - p2)

    # Extract X (front) and Y (left) components
    dx = dcurve[:, 0]
    dy = dcurve[:, 1]

    # Calculate heading: 0 deg at +X, 90 deg at +Y (counter-clockwise)
    # headings = np.degrees(np.arctan2(dy, dx))
    headings = (np.degrees(np.arctan2(dcurve[:, 0], dcurve[:, 1])) + 360) % 360
    
    headings = (headings + 360) % 360

    return curve, headings


# dan's coordinate target [-9.1, -1.0, 10]
# Intersection [-3.42871818  0.        ] 6.271077449501146


# --- Example Evaluation ---
start_node = [0.0, 0.0, 0.0]
# end_node = [-30.0, -10.0, 90.0]
end_node = [-19.1100000000000003, -1.79, 10.0]
# end_node = [-30.0, -10.0, 0.0]

p2_loc = intersection_tra(
    p0_3d=np.array(start_node)[:2],
    heading_start_deg=start_node[2], 
    p3_3d=np.array(end_node)[:2], 
    heading_end_deg=end_node[2]
)
print("--", p2_loc)

# p2_h = start_node[2]
p2_h = get_heading_p_to_p0(end_node[0:2], start_node[:2])
inter_point = [p2_loc[0], p2_loc[1], p2_h]


control_points = np.array([
    # [0.0, 0.0, 0.0], [-7.0, 0.0, 0.0], [-15.0, 0.0, 0.0], [-20.0, 0.0, 0.0]
    start_node, inter_point, inter_point, end_node
])



# Pass points into the interpolation
p0, p1, p2, p3 = control_points[0], control_points[1], control_points[2], control_points[3]
curve_points, headings = compute_bezier_tra(p0, p1, p2, p3, n=200) ## start/end point on line, middle for curve

# points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(curve_points.tolist(), headings.tolist())]
points_one_stage_den = []
for i in range(200):
    points_one_stage_den.append([curve_points[i][0], curve_points[i][1], curve_points[i][2], 'r'])


# print(points_one_stage_den)
points_one_stage_den = [[0.0, -20.0, 180.0], [0.007727073917480755, -20.419294140950953, 176.03683518584114], [0.028575594109928694, -20.444572571878105, 9.52198856494067], [0.05904650823735257, -20.121666146998173, 4.050949182308273], [0.09564076395976111, -19.496405720527893, 2.894808043664284], [0.13485930893716314, -18.614622146683978, 2.2660900776879203], [0.1732030908295673, -17.52214627968313, 1.779528689724998], [0.20717305729698235, -16.26480897374209, 1.3242035641923735], [0.2332701559994171, -14.888441083077572, 0.8466793980085754], [0.2479953345968804, -13.438873461906296, 0.3066698556359597], [0.24784954074938068, -11.961936964444972, 359.6602850178807], [0.22933372211692699, -10.50346244491033, 358.8467344115456], [0.18894882635952798, -9.109280757519091, 357.7682017272584], [0.12319580113719247, -7.825222756487967, 356.24755194433635], [0.02857559410992888, -6.697119296033681, 353.9204373347088], [-0.09841084706225378, -5.77080123037295, 349.89818373738456], [-0.26126257471934644, -5.092099413722499, 341.36666052806225], [-0.46347864120134075, -4.706844700299043, 315.0243427765951], [-0.7085580988482283, -4.660867944319303, 242.5682340278568], [-1.0, -5.0, 210.0]]
points_one_stage_den = [[0.0, 0.0, -10.0], [-8.351834827664385, 1.4148392934201193, -4.197585741896016], [-12.68564977080845, 1.7743061214697289, -1.231250314855508], [-14.429001288021192, 0.9832300535762283, -0.1492894131524773], [-15.0, -1.0, 0.0]]
# plot_ch(points_one_stage_den[::])

with open("./hummer_path/pathx200_dan.txt", "r") as f:
    data = json.load(f)
    print(data)
    plot_ch(data[::10])
exit()      



# with open('hummer_path/back200.txt', 'w', encoding='utf-8') as f:
    # json.dump(points_one_stage_den, f, indent=2)


# with open('hummer_path/back200_curve.txt', 'w', encoding='utf-8') as f:
    # json.dump(points_one_stage_den, f, indent=2)


# with open('hummer_path/back200_new.txt', 'w', encoding='utf-8') as f:
    # json.dump(points_one_stage_den, f, indent=2)



def transform_to_local_frame_by_p0_J_coor(path):
    if not path:
        return []
    
    # 1. Establish the reference frame from the first element
    x0, y0, h0 = path[0]
    rad0 = math.radians(h0)
    
    cos_h0 = math.cos(rad0)
    sin_h0 = math.sin(rad0)
    
    transformed_path = []
    
    for x, y, heading in path:
        # 2. Translate relative to the first point
        dx = x - x0
        
        dy = y - y0
        
        # 3. Rotate using your custom coordinate rules:
        # Left = +X, Up = +Y
        # 0 deg = Up (+Y), 90 deg = Left (+X)
        local_x = dx * cos_h0 - dy * sin_h0
        local_y = dx * sin_h0 + dy * cos_h0
        
        # 4. Normalize the heading
        local_h = (heading - h0) % 360
        
        transformed_path.append([local_x, local_y, local_h])
        
    return transformed_path

path = [[0.0, 0.0, 31.8], [-2.8, -3.40, 263], [-7.0, -11.0, 256], [-3.1, -8.70, 294], [12.0, -6.0, 256]]
transformed = transform_to_local_frame_by_p0_J_coor(path)
## in J's coordinate
print(transformed)





