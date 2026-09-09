import json
import matplotlib.pyplot as plt

f = open("parking_test2_0730/logging_park_ublox2.csv", "r")
fo = open("parking_test2_0730/logging_park_ublox2_bri.csv", "w")
for line in f:
	eles = line.split(",")
	fo.write("%s,%s\n"%(eles[1], eles[2]))
f.close()
fo.close()


xs_real = []
ys_real = []
f = open("parking_test2_0730/logging_park_con2.csv", "r")
for line in f:
	# print(line)
	eles = line.split(",")
	x = float(eles[1])
	y = float(eles[2])
	# print(x,y)
	xs_real.append(x)
	ys_real.append(y)
f.close()



f = open("parking_test2_0730/logging_park_justin_wps2.txt", "r")
data = json.loads(f.readline())
jp = [data[0], data[3]]
print(data)
f.close()

plt.scatter(ys_real, xs_real)
plt.scatter([x[0] for x in jp], [x[1] for x in jp])
# plt.scatter(xs_real, ys_real)
plt.xlim(-5,15)
plt.ylim(-5,15)
plt.show()
