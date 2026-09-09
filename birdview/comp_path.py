import subprocess

## read start execution tag, if 1, compile and start, 0 for stop

# commands = """
# cd ../../ConnAu/ros_ws
# source ~/.bashrc
# sudo colcon build
# ./scripts/fusion_launch.sh 
# ros2 run fusion_py CANGPSReader 
# ros2 run fusion_py VehicleCommander 
# """

commands = """
cd ../../ConnAu/ros_ws
source ~/.bashrc
sudo colcon build
./scripts/fusion_launch.sh 
ros2 run fusion_py VehicleCommander 
"""

## background
#ros2 run fusion_py CANGPSReader // need to restart after compile break it
#ros2 run fusion_py SerialGPSReader 
#ros2 run fusion_py WpsFromJustinApp 

## start/end
# ros2 run fusion_py VehicleCommander 


# Run all commands in a single shell session
process = subprocess.Popen(
    ["/bin/bash", "-c", commands],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True
)

# Print output live
for line in process.stdout:
    print(line, end='')

# Print errors if any
stderr = process.stderr.read()
if stderr:
    print("\nErrors:\n", stderr)

process.wait()
