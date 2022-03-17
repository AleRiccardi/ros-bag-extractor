#!/usr/bin/env python3

import os
import click
import cv2
import yaml
import decimal
import multiprocessing
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2

from utils.tools import load_bags
from sensors.lidar import LidarExtraction, Lidar

from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R

IMU_HEADER = "timestamp [ns], qat_x, qat_y, qat_z, qat_w, acc_x, acc_y, acc_z, ang_x, ang_y, ang_z"
GPS_HEADER = "timestamp [ns], latitude, longitude, altitude"
POSE_HEADER = "timestamp [ns], pose [affine]"

IMU_FORMAT = "%d, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f"
GPS_FORMAT = "%d, %10.20f, %10.20f, %10.20f"
POSE_FORMAT = "%d, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f"


def single_imu(msg, data):
    imu_dict = yaml.load(str(msg), Loader=yaml.FullLoader)
    single = [msg_to_timestamp(msg)]

    single += imu_dict["orientation"].values()
    single += imu_dict["linear_acceleration"].values()
    single += imu_dict["angular_velocity"].values()

    # single += [0, 0, 0, 0]
    # single += imu_dict["accel"].values()
    # single += imu_dict["gyro"].values()

    data.append(single)


def export_imu(bags, topic, dir_out):
    print(">> Extracting IMUs:")
    manager = multiprocessing.Manager()
    data = manager.list()

    for idx, bag in enumerate(bags):
        print("{}/{}: {}".format(idx+1, len(bags), bag.filename))
        bag_msgs = bag.read_messages(topics=[topic])
        msg_count = bag.get_message_count(topic)

        processes = []
        for idx, (_, msg, _) in tqdm(enumerate(bag_msgs), total=msg_count):

            # Start the process
            p = multiprocessing.Process(target=single_imu, args=(
                msg, data))
            proc_start_n_check(p, processes)
        proc_join_all(processes)

    data = np.array(data)
    data_idx_sort = np.argsort(data, axis=0)
    data = data[data_idx_sort[:, 0]]

    filename = os.path.join(dir_out, "data.csv")
    np.savetxt(filename,
               data,
               fmt=IMU_FORMAT,
               delimiter=',',
               newline='\n',
               header=IMU_HEADER,
               footer='',
               comments='# ',
               encoding=None)


def export_coordinates(bags, topic, dir_out):
    print(">> Extracting coordinates:")
    data = []
    for idx, bag in enumerate(bags):
        print("{}/{}: {}".format(idx+1, len(bags), bag.filename))
        bag_msgs = bag.read_messages(topics=[topic])
        msg_count = bag.get_message_count(topic)

        for idx, (_, msg, _) in tqdm(enumerate(bag_msgs), total=msg_count):
            coor_dict = yaml.load(str(msg), Loader=yaml.FullLoader)

            single = [msg_to_timestamp(msg)]
            single.append(coor_dict["latitude"])
            single.append(coor_dict["longitude"])
            single.append(coor_dict["altitude"])

            if (single[-1] == "nan"):
                single[-1] = 0

            data.append(single)

    data = np.array(data)
    filename = os.path.join(dir_out, "data.csv")
    np.savetxt(filename,
               data,
               fmt=GPS_FORMAT,
               delimiter=',',
               newline='\n',
               header=GPS_HEADER,
               footer='',
               comments='# ',
               encoding=None)


def single_image(msg, bridge, dir_out):
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    # Timestamp and file name
    timestamp = msg_to_timestamp(msg)
    filename = str(timestamp) + ".png"
    paht_file = os.path.join(dir_out, filename)
    cv2.imwrite(paht_file, img)


def export_images(bags, topic, dir_out):
    print(">> Extracting images:")

    for idx, bag in enumerate(bags):
        print("{}/{}: {}".format(idx+1, len(bags), bag.filename))
        bag_msgs = bag.read_messages(topics=[topic])
        msg_count = bag.get_message_count(topic)
        bridge = CvBridge()

        processes = []
        for idx, (_, msg, _) in tqdm(enumerate(bag_msgs), total=msg_count):

            # Start the process
            p = multiprocessing.Process(target=single_image, args=(
                msg, bridge, dir_out))
            proc_start_n_check(p, processes)
        proc_join_all(processes)


def export_nav_quat(bags, topic_nav, topic_quat, dir_out):
    print(">> Extracting Navigation and Quaternion:")

    nav = []
    nav_ts = []
    rots = []
    quat_ts = []
    for idx, bag in enumerate(bags):
        print("{}/{}: {}".format(idx+1, len(bags), bag.filename))
        bag_nav_msgs = bag.read_messages(topics=[topic_nav])
        msg_nav_count = bag.get_message_count(topic_nav)

        bag_quat_msgs = bag.read_messages(topics=[topic_quat])
        msg_quat_count = bag.get_message_count(topic_quat)

        print("Nav:")
        for idx, (_, msg_nav, _) in tqdm(enumerate(bag_nav_msgs), total=msg_nav_count):
            ts = msg_to_timestamp(msg_nav)
            nav_dict = yaml.load(str(msg_nav), Loader=yaml.FullLoader)
            xyz = [nav_dict["latitude"],
                   nav_dict["longitude"], nav_dict["altitude"]]
            nav_ts.append(ts)
            nav.append(xyz)

        print("Quat:")
        for idx, (_, msg, _) in tqdm(enumerate(bag_quat_msgs), total=msg_quat_count):
            ts = msg_to_timestamp(msg)
            quat_dict = yaml.load(str(msg), Loader=yaml.FullLoader)
            xyzw = [quat_dict["quaternion"]["x"], quat_dict["quaternion"]["y"],
                    quat_dict["quaternion"]["z"], quat_dict["quaternion"]["w"]]
            # rotate around x (we want to have z facing upwards, and not downwards)
            calib = R.from_euler('x', [180], degrees=True).as_matrix()
            # some arbitrary random rotation (seems like we need it)
            rotz = R.from_euler('z', [90], degrees=True).as_matrix()
            # rotation in the LiDAR Frame!
            rot = rotz @ calib @ R.from_quat(
                xyzw).as_matrix() @ np.linalg.inv(calib)
            quat_ts.append(ts)
            rots.append(rot[0])

    nav_ts = np.array(nav_ts)
    nav = np.array(nav)
    rots = np.array(rots)

    poses = np.tile(np.eye(4), (rots.shape[0], 1, 1))
    poses[:, :3, :3] = rots
    poses[:, :3, -1] = nav
    poses = poses.reshape(-1, 16)
    time_poses = np.zeros((poses.shape[0], poses.shape[1] + 1))
    time_poses[:, 0] = nav_ts
    time_poses[:, 1:] = poses

    #### Store to disk ####
    dir_out = dir_out if dir_out[-1] == "/" else dir_out + "/"
    os.makedirs(dir_out, exist_ok=True)
    file_path = dir_out + "poses.txt"

    np.savetxt(file_path,
               time_poses,
               fmt=POSE_FORMAT,
               delimiter=',',
               newline='\n',
               header=POSE_HEADER,
               footer='',
               comments='# ',
               encoding=None)


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


@click.command()
@click.argument("path_bags", type=click.Path(exists=True))
# CLOUD
@click.option(
    "--velodyne",
    type=str,
    default="",
    help="Topic name of the point cloud messages",
)
@click.option(
    "--ouster",
    type=str,
    default="",
    help="Topic name of the point cloud messages",
)
@click.option(
    "--hokuyo",
    type=str,
    default="",
    help="Topic name of the point cloud messages",
)
# IMU
@click.option(
    "--imu",
    type=str,
    default="",
    help="Topic name of the IMU messages",
)
# COORDINATES
@click.option(
    "--coordinate",
    type=str,
    default="",
    help="Topic name of the coordinate messages",
)
# IMAGES
@click.option(
    "--image",
    type=str,
    default="",
    help="Topic name of the image messages",
)
@click.option(
    "--nav",
    type=click.Path(exists=False),
    default="",
    help="Topic name of the sbg ekf navigation sistem",
)
@click.option(
    "--quat",
    type=click.Path(exists=False),
    default="",
    help="Topic name of the sbg ekf quaternion sistem",
)
@click.option(
    "--path_save",
    type=click.Path(exists=False),
    default="",
    help="Where to store the results",
)
@click.option(
    "--multi",
    type=str,
    default=False,
    help="Does the path containe multiple bag files?",
)
def main(path_bags, velodyne, ouster, hokuyo, imu, coordinate, image, nav, quat, path_save, multi):
    """Convert a .bag file into multiple .ply files, one for each scan,
    including intensity information encoded on the color channel of the
    PointCloud. We use Open3D to convert the data from ROS to
    o3d.geometry.PointCloud.
    """
    print("\n########################")
    print("# RosBagExtraction     #")
    print("########################\n")

    bags = load_bags(path_bags, multi)
    path_root, _ = os.path.split(path_bags)

    # Velodyne
    if velodyne:
        velodyne_ext = LidarExtraction(
            velodyne, bags, path_root, Lidar.VELODYNE)
        velodyne_ext.extract()
    # Ouster
    if ouster:
        ouster_ext = LidarExtraction(ouster, bags, path_root, Lidar.OUSTER)
        ouster_ext.extract()
    # Hokuyo
    if hokuyo:
        hokuyo_ext = LidarExtraction(hokuyo, bags, path_root, Lidar.HOKUYO)
        hokuyo_ext.extract()

    ############
    # IMUs
    #  if check_topic_bag(imu, bags):
    #  path_save_imu = create_folder(path_save, "imu")
    #  export_imu(bags, imu, path_save_imu)

    ############
    # GPSs
    #  if check_topic_bag(coordinate, bags):
    #  path_save_lidar = create_folder(path_save, "coordinated")
    #  export_coordinates(bags, coordinate, path_save_lidar)

    ############
    # Images
    #  if check_topic_bag(image, bags):
    #  path_save_lidar = create_folder(path_save, "images")
    #  export_images(bags, image, path_save_lidar)

    ############
    # Nav + Quat
    #  if check_topic_bag(nav, bags) and check_topic_bag(quat, bags):
    #  path_save_lidar = create_folder(path_save, "nav")
    #  export_nav_quat(bags, nav, quat, path_save_lidar)

    # Close all the bags
    for bag in bags:
        bag.close()


if __name__ == "__main__":
    main()
