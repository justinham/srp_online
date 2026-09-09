
1. path following client map server
app.py 
protocol: https (since realtime web gps only support https)
port: 5001

2. path generating phone server
app_iphone.py
protocol: http (since iphone doesn't support https TCP communication unlsess using a fixed DNS server)
port: 5000 

Both need to run togehter, they share the same local path file "path.txt" for read/write





#######
colcon build
 1377  source ~/.bashrc 
 1378  ./scripts/fusion_launch.sh 
 1379  ros2 run fusion_py SerialGPSReader 
 1380  ros2 run fusion_py CANGPSReader 

