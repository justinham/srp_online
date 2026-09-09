import matplotlib.pyplot as plt
import numpy as np



### gps
import matplotlib.pyplot as plt
import numpy as np
from geopy.distance import geodesic



ps = [[0,0, 248.40], [1.35, 2.18, 103.38], [4.27,1.49, 103.38]]
xs = [p[0] for p in ps]
ys = [p[1] for p in ps]
hs = [np.deg2rad(p[2]) for p in ps]  # convert degrees to radians

# Arrow direction (unit vector)
dx = [np.cos(h) for h in hs]
dy = [np.sin(h) for h in hs]

ds = ((ps[1][0]-ps[2][0])**2+(ps[1][1]-ps[2][1])**2)**0.5
print("dis", ds)

plt.figure()
plt.plot(xs, ys, 'bo-')  # plot points and connecting line
plt.quiver(xs, ys, dx, dy, angles='xy', scale_units='xy', scale=0.5, color='r')

plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.grid(True)
plt.show()
exit()



s = [1.132, -1.113] 
e = [13.021, 3.878]
off = 1.34

s = [0.103, -3.082]
e = [13.292, 0.568]
off = 1.22

s = [1.964, -2.641]
e = [11.290, 7.615]
off = 1.25

s = [5.201, -10.990]
e = [2.156, 0.686]
off = 1.35

s = [14.36, 0.98]
e = [0,0]
off = 3.48

s = [-13.78, -3.42]
off = 1.88

s = [-13.48, -3.13]
off = 1.98

s = [0.17, -2.73]
off = 1.42

def dis_tp(s,e,off):
	dis = ((s[0]-e[0])**2+(s[1]-e[1])**2-off**2)**0.5
	return dis

print(dis_tp(s,e,off))
exit()


# 0 to 3,4
# 42.60893100364071, -82.9938970270353 --> 42.608978468689564, -82.99404998720848
# 42.60893100364071, -82.9938970270353 --> 42.60900402677709, -82.99404667996156
# compass on, spot first: 
'''
spot 1st
0,0,281.156-->-11.0,6.446,271.388 // 12.750
car 1st
0,0,275.379-->-11.285,5.519,267.226 // 12.563
car 1st
0,0,283.622-->-11.165,9.729,272.457 //14.809

# heading off
car 1st
0,0,359.747-->5.095,11.880,358.425 // 12.926
car 1st
0,0,140.172-->1.407,-14.477,138.940 // 14.545
spot 1st
0,0,320.656-->-4.006,12.197,313.390 //12.837
spot 1st
0,0,340.700-->3.353,14.248,336.434 // 14.638
'''



# compass on:


# GPS coordinates: (latitude, longitude)
# 42.5172555436363, -83.04529016894166
# 42.517255935862956, -83.04522359241123
# 42.517259889834776, -83.04515653718131
# 42.51725890134181, -83.04508948195942
# 42.51726285531358, -83.04495537151563
# 42.517264832299375, -83.04488965739819
# 42.51726488714896, -83.04482063397562
# point1 = (lat1, lon1)  # Replace with your first point
# point2 = (lat2, lon2)  # Replace with your second point

# Distance in meters
# distance_meters = geodesic(point1, point2).meters
# print(f"Distance: {distance_meters:.2f} meters")

# exit()

# Two error lists in meters
errors1 = [1.01, 0.61, 0.78, 0.52, 0.87, 1.05, 1.58]
errors2 = [0.90, 0.41, 0.45, 1.12, 0.22, 0.30, 0.54]

# X-axis positions
indices = np.arange(len(errors1))

plt.figure(figsize=(8,5))

# Plot both error lists
plt.plot(indices, errors1, marker='o', label='Errors with CAN GPS')
plt.plot(indices, errors2, marker='s', label='Errors with UBLOX')

# Annotate values above each point
for i, val in enumerate(errors1):
    plt.text(i, val + 0.05, f"{val:.2f}", ha='center', fontsize=8)
for i, val in enumerate(errors2):
    plt.text(i, val - 0.15, f"{val:.2f}", ha='center', fontsize=8, color='blue')

# Labels & legend
plt.xticks(indices, [f"P{i+1}" for i in indices])
plt.xlabel("Point Index")
plt.ylabel("Error (m)")
plt.title("Comparison of Two Error Lists")
plt.legend()
plt.grid(True, linestyle='--', alpha=0.5)

plt.tight_layout()
plt.show()
exit()


### vrkit


def dis2D(dis_arr, height_offset):
	real_dis_arr = []
	for d in dis_arr:
		rd = (d**2-height_offset**2)**0.5
		real_dis_arr.append(rd)
	return real_dis_arr


cam_to_ground = 1
cam_to_veh_height_half = 0.2

# start from id 7, to direct 6,5,4,3,2,1, then other side with turns 8,9,10
## spot
gt1 = [2.75, 2.75*2, 2.75*3, 2.75*4, 2.75*5, 2.75*6, 13.15, 13.43, 14.25]
dis_with_compass_off1 = [2.993, 5.585, 8.339, 0, 14.328, 17.092, 12.704, 12.955, 13.884]
dis_with_compass_on1 = [3.061, 5.681, 8.531, 11.121, 14.337, 16.915, 13.110, 13.697, 14.876]

## veh
dis_with_compass_off1 = [3.299, 5.985, 8.676, 11.402, 14.454, 17.291, 14.249, 14.674, 15.266]
dis_with_compass_on1 = [3.110, 5.934, 8.590, 11.507, 14.598, 17.166, 13.617, 14.127, 15.159]


# start from id 1, to direct 2,3,4,5,6,7, then other side with turns 8,9,10
## spot
gt2 = [2.75, 2.75*2, 2.75*3, 2.75*4, 2.75*5, 2.75*6, 21.10, 19.03, 17.14]
dis_with_compass_off2 = [2.818, 5.671, 8.353, 11.108, 14.034, 17.081, 21.107, 18.760, 16.789]
dis_with_compass_on2 = [2.944, 5.632, 8.396, 11.146, 13.888, 16.533, 21.012, 18.964, 16.930]

## veh
dis_with_compass_off2 = [3.387, 5.945, 8.862, 11.576, 14.430, 17.380, 21.655, 19.484, 17.489]
dis_with_compass_on2 = [3.194, 5.753, 8.633, 11.478, 14.393, 17.185, 21.350, 19.320, 17.286]


# dis_compass_off1 = dis2D(dis_with_compass_off1, cam_to_ground)
# dis_compass_on1 = dis2D(dis_with_compass_on1, cam_to_ground)
dis_compass_off1 = dis2D(dis_with_compass_off1, cam_to_veh_height_half)
dis_compass_on1 = dis2D(dis_with_compass_on1, cam_to_veh_height_half)

arr1 = np.array(dis_compass_off1)
arr2 = np.array(dis_compass_on1)
arr3 = np.array(gt1)

# Element-wise difference
diff_off1 = np.abs(arr1 - arr3)
diff_on1 = np.abs(arr2 - arr3)

# print(diff_off1)
# print(diff_on1)

# plt.scatter(range(len(gt1)), dis_compass_off1, alpha=0.5, label='compass off test #1')
# plt.scatter(range(len(gt1)), dis_compass_on1, alpha=0.5, label='compass on test #1')
# plt.scatter(range(len(gt1)), gt1, alpha=0.5, label='estimated by google map')

# plt.scatter(range(len(gt1)), diff_off1, alpha=0.5, label='compass off test #1')
# plt.scatter(range(len(gt1)), diff_on1, alpha=0.5, label='compass on test #1')

# plt.legend()
# plt.show()
# exit()


# dis_compass_off2 = dis2D(dis_with_compass_off2, cam_to_ground)
# dis_compass_on2 = dis2D(dis_with_compass_on2, cam_to_ground)

dis_compass_off2 = dis2D(dis_with_compass_off2, cam_to_veh_cen)
dis_compass_on2 = dis2D(dis_with_compass_on2, cam_to_veh_cen)

arr21 = np.array(dis_compass_off2)
arr22 = np.array(dis_compass_on2)
arr23 = np.array(gt2)

# Element-wise difference
diff_off2 = np.abs(arr21 - arr23)
diff_on2 = np.abs(arr22 - arr23)

# plt.scatter(range(len(gt2)), dis_compass_off2, alpha=0.5, label='compass off test #2')
# plt.scatter(range(len(gt2)), dis_compass_on2, alpha=0.5, label='compass on test #2')
# plt.scatter(range(len(gt2)), gt2, alpha=0.5, label='estimated by google map')

plt.scatter(range(len(gt2)), diff_off2, alpha=0.5, label='compass off test #2')
plt.scatter(range(len(gt2)), diff_on2, alpha=0.5, label='compass on test #2')

plt.legend()
plt.show()
