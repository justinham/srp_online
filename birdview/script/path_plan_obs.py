import numpy as np
import matplotlib.pyplot as plt
import heapq
import json
import math


#### from google map ##
# down right: 42.52015238777492, -83.04342263768308, 180
# up left: 42.52025568042295, -83.04342263768518, 0
# start around: 42.52018451230209, -83.04359496961165, 90
# heading in anchor: 42.52018945453538, -83.04349304567492, 90
#####



########### A-start searching ###############

# 4-connected neighbors
# MOVES = [
#     (-1,  0),  # N
#     ( 0,  1),  # E
#     ( 1,  0),  # S
#     ( 0, -1)  # W
# ]

# 8-connected neighbors
MOVES = [
    (-1,  0),  # N
    (-1,  1),  # NE
    ( 0,  1),  # E
    ( 1,  1),  # SE
    ( 1,  0),  # S
    ( 1, -1),  # SW
    ( 0, -1),  # W
    (-1, -1),  # NW
]

# 16-connected neighbors (also need to check goal close enough)
# MOVES = [
#     (-2,  -2),  
#     (-2,  -1),  
#     (-2,  0),  
#     (-2,  1),  
#     (-2,  2),  
#     (-1, -2), 
#     (-1, 2),  
#     (0, -2),  
#     (0, 2), 
#     (1, -2), 
#     (1, 2), 
#     (2, -2), 
#     (2, -1), 
#     (2, 0), 
#     (2, 1), 
#     (2, 2),  
# ]


def buffer_obs(obs):
    buffers = []
    for i in range(-buffer_size, buffer_size+1):
        for j in range(-buffer_size, buffer_size+1):
            b = (obs[0]+i, obs[1]+j)
            if b[0]>=0 and b[0]<20 and b[1]>=0 and b[1]<20:
                buffers.append(b)
    return buffers



# Heading from dy/dx: y-axis = 0°, x-axis = 90°
def heading_between(p1, p2):
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    angle = math.degrees(math.atan2(dx, dy))
    return (angle + 360) % 360

def angle_diff(h1, h2):
    diff = abs(h1 - h2) % 360
    return min(diff, 360 - diff)

def heuristic(a, b):
    # Euclidean distance for better behavior
    return math.hypot(a[0] - b[0], a[1] - b[1])

def get_neighbors(pos):
    neighbors = []
    for dx, dy in MOVES:
        nx, ny = pos[0] + dx, pos[1] + dy
        if 0 <= nx < width and 0 <= ny < height and grid_buff[nx, ny] == 0:
            neighbors.append((nx, ny))
    return neighbors

def a_star_with_heading(start, goal):
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start[:2], goal[:2]), 0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while open_set:
        _, current_cost, current = heapq.heappop(open_set)
        curr_pos = (current[0], current[1])
        curr_heading = current[2]

        if curr_pos == (goal[0], goal[1]):
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        for neighbor_pos in get_neighbors(curr_pos):
            new_heading = heading_between(curr_pos, neighbor_pos)
            turn_cost = angle_diff(curr_heading, new_heading) / 45  # turning penalty
            move_cost = math.hypot(neighbor_pos[0] - curr_pos[0], neighbor_pos[1] - curr_pos[1])
            new_cost = current_cost + move_cost + turn_cost
            neighbor_state = (neighbor_pos[0], neighbor_pos[1], new_heading)

            if neighbor_state not in cost_so_far or new_cost < cost_so_far[neighbor_state]:
                cost_so_far[neighbor_state] = new_cost
                priority = new_cost + heuristic(neighbor_pos, goal[:2])
                heapq.heappush(open_set, (priority, new_cost, neighbor_state))
                came_from[neighbor_state] = current

    return None



def show_astar_path(grid_buff, path, start, goal):
    plt.figure(figsize=(6, 6))
    plt.imshow(grid_buff.T, cmap='gray_r', origin='lower')
    if path:
        px, py = zip(*[(p[0], p[1]) for p in path])
        plt.plot(px, py, 'b-', label="Path")
        for p in path[::2]:
            angle = math.radians(p[2])
            dx = 0.4 * math.sin(angle)
            dy = 0.4 * math.cos(angle)
            plt.arrow(p[0], p[1], dx, dy, head_width=0.3, color='blue')
    plt.scatter(start[0], start[1], c='green', s=100, label="Start")
    plt.scatter(goal[0], goal[1], c='red', s=100, label="Goal")
    plt.legend()
    plt.grid(True)
    plt.title("A* Path with Heading")
    plt.show()





################# multi-stage b-curve #######

def normalize_heading(h):
    """Normalize heading to [0, 360)"""
    return h % 360

def heading_to_direction(heading_deg):
    """
    Converts heading in degrees (0 = North, clockwise) to unit direction vector.
    """
    heading_rad = np.radians(normalize_heading(heading_deg))
    dx = np.sin(heading_rad)  # X is sin
    dy = np.cos(heading_rad)  # Y is cos
    return np.array([dx, dy])

def compute_bezier(p0, p1, p2, p3, n=50):
    """
    Compute cubic Bezier curve from control points.
    """
    p0 = np.array(p0).reshape(1, 2)
    p1 = np.array(p1).reshape(1, 2)
    p2 = np.array(p2).reshape(1, 2)
    p3 = np.array(p3).reshape(1, 2)

    t = np.linspace(0, 1, n)[:, None]

    curve = (1 - t)**3 * p0 + \
           3 * (1 - t)**2 * t * p1 + \
           3 * (1 - t) * t**2 * p2 + \
           t**3 * p3

    #deviation on curve
    dcurve = -3 * (1 - t)**2 * p0 + \
             3 * (1 - t)**2 * p1 - 6 * (1 - t) * t * p1 + \
             6 * (1 - t) * t * p2 - 3 * t**2 * p2 + \
             3 * t**2 * p3

    # Heading from tangent: Y=0 deg, clockwise
    headings = (np.degrees(np.arctan2(dcurve[:, 0], dcurve[:, 1])) + 360) % 360  # shape (n,)

    # print(headings, curve)
    return curve, headings

def generate_path(points, heading_scale=2.0, resolution=50):
    """
    Generate smooth path using Bezier curves from points with headings.
    """
    curves = []
    headings = []
    for i in range(len(points) - 1):
        x0, y0, h0 = points[i]
        x1, y1, h1 = points[i + 1]
        
        p0 = np.array([x0, y0])
        p3 = np.array([x1, y1])
        
        d0 = heading_to_direction(h0)
        d1 = heading_to_direction(h1)
        
        p1 = p0 + d0 * heading_scale
        p2 = p3 - d1 * heading_scale
        
        bezier,heading = compute_bezier(p0, p1, p2, p3, n=resolution)
        curves.append(bezier)
        headings.append(heading)

    return np.vstack(curves), np.vstack(headings).reshape(len(points)-1*resolution, 1)

def plot_path_with_headings(points, path, arrow_scale=0.8):
    """
    Plot the Bezier path and show heading directions at control points.
    """
    plt.figure(figsize=(6, 6))
    plt.scatter(path[:, 0], path[:, 1], alpha=0.5, label='interpulation')
    plt.plot(path[:, 0], path[:, 1], alpha=0.5, label='Bezier Path')

    plt.imshow(grid.T, cmap='gray_r', origin='lower')
    for x, y, h in points:
        plt.plot(x, y, 'ro')
        dir_vec = heading_to_direction(h)
        plt.arrow(x, y, dir_vec[0]*arrow_scale, dir_vec[1]*arrow_scale,
                  head_width=0.3, head_length=0.4, fc='r', ec='r')

    plt.axis('equal')
    plt.grid(True)
    plt.title("Bezier Curve Path with Heading Tangents")
    plt.legend()
    plt.tight_layout()
    plt.show()




def heading_clockwise_y0(p1, p2):
    """Heading in degrees where 0° is north (Y+), increasing clockwise."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    angle = np.degrees(np.arctan2(dx, dy))  # note: dx, dy swapped
    return (angle + 360) % 360

def heading_deviations(points):
    """
    Calculate heading deviations (in degrees) between consecutive segments.
    Returns a list of deviations and all headings.
    """
    headings = [heading_clockwise_y0(points[i], points[i+1]) for i in range(len(points)-1)]
    deviations = [abs((headings[i+1] - headings[i] + 180) % 360 - 180) for i in range(len(headings)-1)]
    return deviations, headings







############ main ################


# Define map size
width, height = 30, 30


# testing Start and goal
start = (0, 0, 45)
goal = (19, 19, 50)
goal2 = (28,19,140)
# obstacles_cen = [(0,17), (5,5), (18,10)]
obstacles_cen = [(0,17), (5,5), (5,11), (16,10)]
buffer_size = 3



######## load real data ######
# offset = 0
# # with open("./hummer_path/pathx5.txt", 'r') as f:
# with open("./hummer_path/pathx5_can_heading.txt", 'r') as f:
#     data = json.load(f)
#     start = (int(data[0][0])+offset, int(data[0][1])+offset, int(data[0][2]))
#     goal = (int(data[2][0])+offset, int(data[2][1])+offset, int(data[2][2]))
#     goal2 = (data[3][0]+offset, data[3][1]+offset, data[3][2])

# obstacles_cen = []
# # with open("./hummer_path/obs.txt", 'r') as f:
# with open("./hummer_path/obs_can_heading.txt", 'r') as f:
#     obs_real = json.load(f)
#     for ob in obs_real:
#         obstacles_cen.append((int(ob[0])+offset, int(ob[1])+offset))

# # obstacles_cen = []
# print(start, goal, obstacles_cen)
# # exit()


obstacles = []
for obs in obstacles_cen:
    print(obs)
    tmp = buffer_obs(obs)
    obstacles += tmp


# Create grid
grid = np.zeros((width, height))
grid_buff = np.zeros((width, height))
for ox, oy in obstacles:
    grid_buff[ox, oy] += 1  # mark obstacle buffer
for ox, oy in obstacles_cen:
    grid[ox, oy] += 1  # mark obstacle center
    grid_buff[ox, oy] += 1




path = a_star_with_heading(start, goal)
print(path)
show_astar_path(grid_buff, path, start, goal)
print(grid_buff.T)


deviations, headings = heading_deviations(path)

print("Headings:", headings, len(headings))
print("Deviations:", deviations, len(deviations))



f_vs_r="forward"
le_vs_ri="right"
stage_resolution = 20
path_wh = [[p[0], p[1], h] for p,h in zip(path,headings)]
print("wh", path_wh)

## select point with deviation only
path_wh_sel = []
pre = 0
lat = 0
turn_buf = 3
sel_idx = []
rm_idx = []
for i in range(turn_buf, len(path_wh)-turn_buf):
    if deviations[i]>0:

        ## raw turning
        # path_wh_sel.append(path_wh[i])

        ## with turning buffer
        pre = i-turn_buf+1
        lat = i+turn_buf+1
        # sel_idx += [pre, i+1, lat]
        sel_idx += [pre, lat]
        rm_idx += [i+1]

# sel_idx += [0, len(path_wh)-1]
sel_idx += [0]
# unique_sel = list(set(sel_idx))
unique_sel = sorted(list(set(sel_idx)-set(rm_idx)))
for idx in unique_sel:
    path_wh_sel.append(path_wh[idx])

print(sel_idx, unique_sel)
# exit()
# path_wh_sel = [path_wh[0]]+path_wh_sel

path1, headings1 = generate_path(path_wh_sel[:], heading_scale=2.0, resolution=stage_resolution)
# headings1 = np.array(headings1).reshape(40,1)
# print(path1.shape, headings1.shape)
# exit()
points_one_stage_anchor = path_wh_sel+[goal, goal2]
anchor_h = goal[2]
# trans_p = [path_wh_sel[-1][0],path_wh_sel[-1][1], anchor_h]
trans_p = path_wh_sel[-1]
path2,headings2 = generate_path([trans_p, goal], heading_scale=4.0, resolution=stage_resolution)
path3,headings3 = generate_path([goal,goal2], heading_scale=4.0, resolution=stage_resolution)
path_sum = np.concatenate((path1, path2, path3))
# print(headings2.shape)
# exit()
heading_sum = np.concatenate((np.array(headings1), np.array(headings2), np.array(headings3)))
print(path1, path2, path3)
print(path_sum.shape, heading_sum.shape)
print(headings1.shape, headings2.shape, headings3.shape)
print(path1.shape, path2.shape, path3.shape)
# exit()
for xy,h in zip(path_sum, heading_sum):
    print(xy[0], xy[1], h[0])
points_one_stage_den = [[xy[0], xy[1], h[0], "f"] for xy,h in zip(path_sum, heading_sum)]
print(points_one_stage_den)
plot_path_with_headings(points_one_stage_anchor, path_sum)

fn = "test"
with open('./hummer_path/%s.txt'%fn, 'w', encoding='utf-8') as f:
    json.dump(points_one_stage_den, f, indent=2)
        















