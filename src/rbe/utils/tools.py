import decimal
import os
from pathlib import Path

import matplotlib.pyplot as plt
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String


def check_topic_bag(topic, bags):
    if topic:
        for bag in bags:
            assert (
                topic in bag.get_type_and_topic_info().topics
            ), "topic: {} is not recorded in {}".format(topic, bag._filename)

        return True

    return False


def get_rosbag_options(path, storage_id, serialization_format="cdr"):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id=storage_id)

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    return storage_options, converter_options


def read_rosbag(bag_path):
    storage_options, converter_options = get_rosbag_options(bag_path, "mcap")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {
        topic_types[i].name: topic_types[i].type for i in range(len(topic_types))
    }

    # Set filter for topic of string type
    storage_filter = rosbag2_py.StorageFilter(topics=["/topic"])
    reader.set_filter(storage_filter)

    msg_counter = 0

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        assert isinstance(msg, String)
        assert msg.data == f"Hello, world! {msg_counter}"

        msg_counter += 1


def load_bags(path: Path):
    list_bags = []
    if path.is_dir():
        for bag in path.glob("*.bag"):
            f_bag = os.path.join(path, bag)
            list_bags.append(f_bag)
    else:
        list_bags.append(path)

    bags = []
    for path_bag in list_bags:
        _, bag_name = os.path.split(path_bag)
        print("Loading: {}".format(bag_name))
        bags.append(rosbag.Bag(path_bag))
    print()
    return bags


def create_folder(path_save, folder_name):
    path_folder = os.path.join(path_save, folder_name)
    os.makedirs(path_folder, exist_ok=True)
    print(">> Saving folder: {}\n".format(path_folder))

    return path_folder


def get_path_save(path_bags, path_save, multi):
    if path_save == "":
        bag_path = path_bags[:-1] if path_bags[-1] == "/" else path_bags
        path_save, _ = os.path.split(bag_path)

        # If the path direct to a specific bag file, then we need to
        # backtrace of two path back.
        # Ex: .../smth/bag/bag-name.bag -> .../smth/
        if not multi:
            path_save, _ = os.path.split(path_save)

    if not os.path.exists(path_save):
        os.makedirs(path_save, exist_ok=True)

    return path_save


def msg_to_timestamp(msg) -> int:
    # Trick to trim the message and parse the yaml
    # data more efficiently.
    msg = "\n".join(str(msg)[:200].split("\n")[:5])

    # Extract seconds and nanoseconds
    msg_dict = yaml.load(str(msg), Loader=yaml.FullLoader)
    stamp = msg_dict["header"]["stamp"]
    sec = float(str(stamp["secs"]))
    nsec = float(str(stamp["nsecs"]))

    # Compose time in seconds
    nsec_to_sec = nsec * 1e-9
    time = decimal.Decimal(sec + nsec_to_sec)

    # Convert time in nanoseconds
    to_nsec = decimal.Decimal(1e9)
    timestamp = int(time * to_nsec)

    return timestamp


def display_image(img):
    plt.imshow(img)
    plt.show()
