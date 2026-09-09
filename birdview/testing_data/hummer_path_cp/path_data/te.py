
import matplotlib.pyplot as plt
import json





# distance between two gps p1: (42.520238, -83.043531) p2: (42.520313, -83.043407) --> 13.74m
# distance between two gps p1: (42.520314, -83.043398) p2: (42.520314, -83.043398) --> 

def cal_dis(p1, p2):
	dis = ((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)**0.5
	return dis

print("ori x20, local x20, x3, x5, x5_can_heading")

f = open("path.txt")
line = f.readline()
path = json.loads(line)
start = path1[0]
end = path1[-1]
print(cal_dis(start, end))
f.close()

f = open("path_local.txt")
line = f.readline()
path1 = json.loads(line)
start1 = path1[0]
end1 = path1[-1]
print(cal_dis(start1, end1))
f.close()

f = open("path_localx3_with_head.txt")
line = f.readline()
path2 = json.loads(line)
start2 = path2[0][:-1]
end2 = path2[-1][:-1]
print(cal_dis(start2, end2))
f.close()


f = open("pathx5.txt")
line = f.readline()
path3 = json.loads(line)
start3 = path3[0][:-1]
end3 = path3[-2][:-1]
print(cal_dis(start3, end3))
f.close()


f = open("pathx5_can_heading.txt")
line = f.readline()
path4 = json.loads(line)
start4 = path4[0][:-1]
end4 = path4[-2][:-1]
print(cal_dis(start4, end4))
f.close()


def getxy(start, end):
	px = [start[0], end[0]]
	py = [start[1], end[1]]
	return px, py

px1,py1 = getxy(start1, end1)
px2,py2 = getxy(start2, end2)
px3,py3 = getxy(start3, end3)
px4,py4 = getxy(start4, end4)

plt.plot(px1, py1, label="1")
plt.plot(px2, py2, label="2")
plt.plot(px3, py3, label="3")
plt.plot(px4, py4, label="4")
plt.legend()
plt.show()



