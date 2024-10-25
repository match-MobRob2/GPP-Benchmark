import os

from asyncio import sleep
from signal import SIGINT, SIGKILL
import subprocess

if __name__ == "__main__":
    for x in range(5):
        print("Start run: " + str(x))
        launch_process = subprocess.Popen(["ros2", "launch", "gpp_pipeline", "pipeline.launch.py"], text=True)
        # print("Wait Start ---------------------------------------------------------------")
        launch_process.wait()
        # print("Sleep Start ---------------------------------------------------------------")
        # sleep(20)
        print("Process End ---------------------------------------------------------------")
        launch_process.send_signal(SIGINT) # Stop launch file
        os.system("kill $(ps aux | grep 'ign gazebo gui' | awk '{print $2}')") # Kill Gazebo Ingition explicit
        launch_process.wait(timeout=30)
        print("End run: " + str(x))