#!/usr/bin/env python3
from pathlib import Path

import click

from sensors.gps import GPSSensor
from sensors.image import ImageSensor
from sensors.imu import IMUSensor
from sensors.ins import INSSensor
from sensors.lidar import Lidar, LidarSensor
from sensors.pose import PoseSensor
from utils.tools import check_topic_bag, load_bags


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
    "--pose",
    type=click.Path(exists=False),
    default="",
    help="Topic name of the poses",
)
def main(path_bags, velodyne, ouster, hokuyo, imu, coordinate, image, nav, quat, pose):
    """Convert a .bag file into multiple .ply files, one for each scan,
    including intensity information encoded on the color channel of the
    PointCloud. We use Open3D to convert the data from ROS to
    o3d.geometry.PointCloud.
    """
    print("\n########################")
    print("# RosBagExtraction     #")
    print("########################\n")

    bags = load_bags(Path(path_bags))

    # Velodyne
    if velodyne:
        velodyne_ext = LidarSensor(velodyne, bags, Lidar.VELODYNE)
        velodyne_ext.extract()
    # Ouster
    if ouster:
        ouster_ext = LidarSensor(ouster, bags, Lidar.OUSTER)
        ouster_ext.extract()
    # Hokuyo
    if hokuyo:
        hokuyo_ext = LidarSensor(hokuyo, bags, Lidar.HOKUYO)
        hokuyo_ext.extract()

    ############
    # IMUs
    if check_topic_bag(imu, bags):
        imu_ext = IMUSensor(imu, bags)
        imu_ext.extract()

    ############
    # GPSs
    if check_topic_bag(coordinate, bags):
        gps_ext = GPSSensor(coordinate, bags)
        gps_ext.extract()

    ############
    # Images
    if check_topic_bag(image, bags):
        image_ext = ImageSensor(image, bags, encoding="yuv422")
        image_ext.extract()

    ############
    # Nav + Quat
    if check_topic_bag(nav, bags) and check_topic_bag(quat, bags):
        ins_ext = INSSensor(image, bags)
        ins_ext.extract()

    ############
    # Poses
    if check_topic_bag(pose, bags):
        pose_ext = PoseSensor(pose, bags)
        pose_ext.extract()

    # Close all the bags
    for bag in bags:
        bag.close()


if __name__ == "__main__":
    main()
