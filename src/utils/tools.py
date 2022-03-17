import os
import yaml
import rosbag
import decimal


def check_topic_bag(topic, bags):
    if topic:
        for bag in bags:
            assert topic in bag.get_type_and_topic_info().topics, (
                "topic: {} is not recorded in {}".format(topic, bag._filename))

        return True

    return False


def load_bags(path, multi):
    list_bags = []
    if multi:
        for bag in os.listdir(path):
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


def msg_to_timestamp(msg):
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
    to_nsec = decimal.Decimal(1e+9)
    timestamp = int(time * to_nsec)

    return timestamp
