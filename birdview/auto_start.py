## restart the current path

import subprocess
import signal
import time
import os
import sys

# Track subprocesses
processes = []

def launch_process(cmd, shell=False):
    print(f"Starting: {cmd}")
    p = subprocess.Popen(cmd, shell=shell, preexec_fn=os.setsid)
    processes.append(p)

def cleanup():
    print("\nStopping all ROS nodes and scripts...")
    for p in processes:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except Exception as e:
            print(f"Error stopping process: {e}")

def start_exc():
    # print("aaa")
    fn = "execution_tag"
    with open("./hummer_path/%s.txt"%fn, 'w') as f:
        f.write(str("1"))

    ## auto start execution
    try:
        # Setup environment and build
        os.chdir("../../ConnAu/ros_ws")

        # start execution
        # launch_process(["ros2", "run", "fusion_py", "CANGPSReader"])
        # launch_process(["ros2", "run", "fusion_py", "SerialGPSReader"])

        launch_process(["ros2", "run", "fusion_py", "VehicleCommander"])

        # Wait for all processes to finish
        for p in processes:
            p.wait()

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        cleanup()
        

    return jsonify({'status': 'success', 'echo': "start"}), 200


if __name__ == "__main__":
    start_exc()
    