from pathlib import Path

import rclpy

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore



if __name__ == "__main__":
    bagpath = Path('/home/lurz-match/cirp_ws/rosbag2_2024_10_03-22_57_46')

    # Create a type store to use if the bag has no message definitions.
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # Create reader instance and open for reading.
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == '/plan']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            print(msg.poses[0].pose.position.x)
            print(msg.poses[1].pose.position.x)
            print(msg.poses[2].pose.position.x)