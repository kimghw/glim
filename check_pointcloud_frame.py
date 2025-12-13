#!/usr/bin/env python3

import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

def check_pointcloud_frame(bag_path):
    """Check frame_id of PointCloud2 messages in rosbag"""

    storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    frames = set()
    count = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic == '/ouster/points':
            msg = deserialize_message(data, PointCloud2)
            frames.add(msg.header.frame_id)
            count += 1
            if count >= 5:  # Check first 5 messages
                break

    reader.close()

    print(f"PointCloud2 frame_id(s): {frames}")
    return frames

if __name__ == "__main__":
    if len(sys.argv) < 2:
        bag_path = "/home/kimghw/glim/rosbag_data/rosbag_20251211_170132/rosbag_20251211_170132_0.mcap"
    else:
        bag_path = sys.argv[1]

    check_pointcloud_frame(bag_path)