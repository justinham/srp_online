import json


fn = "pathx5"
f = open("hummer_path/%s.txt"%fn, "r")
message = json.loads(f.readline())
f.close()
# print("tt",message)


###### Justin 1024 reverse path ######
for msg in message:
	print(msg)

for i in range(len(message)):
	message[i][-1] = (message[i][-1]+180)%360

for msg in message:
	print(msg)
