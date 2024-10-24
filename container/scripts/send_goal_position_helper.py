from asyncio import sleep
from signal import SIGINT, SIGKILL
import subprocess
import rclpy




if __name__ == "__main__":
    launch_process = subprocess.Popen(["ros2", "launch", "gpp_pipeline", "pipeline.launch.py"], text=True)
    launch_process.wait(timeout = 20)
    launch_process.send_signal(SIGINT)
    launch_process.send_signal(SIGKILL)
    launch_process.wait(timeout=30)