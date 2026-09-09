## load new path, compile, start to run

import subprocess
import signal
import time
import os
import sys
import json

# Track subprocesses
processes = []


def launch_process_ps(cmd, shell=False):

    password = "ConnAu\n"
    p = subprocess.run(cmd, shell=True, executable="/bin/bash", input=password.encode(), check=True)
    # p = subprocess.Popen(cmd, shell=shell, preexec_fn=os.setsid)

    processes.append(p)

    
def launch_process_bg(cmd, shell=False):

    # password = "ConnAu\n"
    # p = subprocess.run(cmd, shell=True, executable="/bin/bash", input=password.encode(), check=True)
    p = subprocess.Popen(cmd, shell=shell, preexec_fn=os.setsid)

    processes_bg.append(p)


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

def path_gen():
    print("-- new path generation --")

    ## also consider init GPS here and save previous path

    # init gps
    os.system("cp ./hummer_path/gps.txt ./hummer_path/gps_ref_p2.txt")


    try:
        os.chdir("../../ConnAu/ros_ws")
    
        launch_process_ps("source ~/.bashrc")
        launch_process_ps("colcon build")
        launch_process_ps("./scripts/fusion_launch.sh")

        launch_process_bg(["ros2", "run", "fusion_py", "CANGPSReader"]) ## in case compile breaks it
        

    except:
        tag = "fail"
        print("\n Path generation fail.")


    finally:
        # os.chdir("../../srp/birdview_1021")
        os.chdir("../../srp/birdview")
        cleanup()
        return jsonify({'status': tag, 'echo': "new"}), 200

    

if __name__ == "__main__":
    path_gen()
