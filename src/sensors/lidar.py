import os

import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
import yaml
from laser_geometry import LaserProjection
from sensor_msgs.msg import LaserScan
from tqdm import tqdm

from sensors.base import BaseSensor
from utils.tools import msg_to_timestamp


class Lidar:
    HOKUYO = "hokuyo"
    OUSTER = "ouster"
    VELODYNE = "velodyne"

    @staticmethod
    def list():
        sensors_list = [Lidar.HOKUYO, Lidar.OUSTER, Lidar.VELODYNE]
        return sensors_list


class LidarSensor(BaseSensor):
    _HEADER = "timestamp [ns]"
    _FORMAT = "%s"
    _POINT_TIME_AVAILABLE = False

    def __init__(self, topic, bags, lidar=Lidar.VELODYNE) -> None:
        super().__init__(topic, bags, lidar)

        if lidar not in Lidar.list():
            raise ValueError('"{}" lidar sensor type not supported'.format(lidar))

    def _print_init_msg(self):
        print(">> Extracting clouds:")

    def _extract(self):
        data = []

        for idx, bag in enumerate(self._bags):
            print("{}/{}: {}".format(idx + 1, len(self._bags), bag.filename))
            bag_msgs = bag.read_messages(topics=[self._topic])
            msg_count = bag.get_message_count(self._topic)

            for _, msg, _ in tqdm(bag_msgs, total=msg_count):
                if self._name == Lidar.HOKUYO:
                    self._single_hokuyo(msg, data)
                elif self._name == Lidar.OUSTER:
                    self._single_ouster(msg, data)
                elif self._name == Lidar.VELODYNE:
                    self._single_velodyne(msg, data)

        data = np.sort(np.array(data))
        self._save_csv(data)

    def _load_hokuyo(self, msg):
        data_yaml = yaml.safe_load(msg)
        scan = LaserScan(
            header=data_yaml["header"],
            angle_min=data_yaml["angle_min"],
            angle_max=data_yaml["angle_max"],
            angle_increment=data_yaml["angle_increment"],
            time_increment=data_yaml["time_increment"],
            scan_time=data_yaml["scan_time"],
            range_min=data_yaml["range_min"],
            range_max=data_yaml["range_max"],
            ranges=data_yaml["ranges"],
            intensities=data_yaml["intensities"],
        )

        projector = LaserProjection()
        cloud = projector.projectLaser(scan)
        return cloud

    def _single_hokuyo(self, msg, data):
        msg = str(msg)

        try:
            cloud = self._load_hokuyo(msg)
        except Exception as _:
            msg = msg.replace("nan", "60")
            cloud = self._load_hokuyo(msg)

        # Timestamp and file name
        timestamp = msg_to_timestamp(msg)
        filename = str(timestamp) + ".ply"
        data.append(filename)

        xyz_s = []
        for pt in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = pt
            xyz = [x, y, z]
            xyz_s.append(xyz)

        xyz_s = np.array(xyz_s)
        irt_s = np.array([])
        self._save_ply(filename, xyz_s, irt_s)

    def _single_ouster(self, msg, data):
        # Ouster fields
        field_names = ["x", "y", "z", "intensity", "t", "ring"]

        # Timestamp and file name
        timestamp = msg_to_timestamp(msg)
        filename = str(timestamp) + ".ply"
        data.append(filename)

        points = pc2.read_points_list(
            cloud=msg, field_names=field_names, skip_nans=True
        )
        xyz_s = []
        irt_s = []

        for pt in points:
            x, y, z, i, t, r = pt
            t_ms = t
            xyz = [x, y, z]
            irt = [i, r, t_ms]

            xyz_s.append(xyz)
            irt_s.append(irt)

        xyz_s = np.array(xyz_s)
        irt_s = np.array(irt_s)

        self._save_ply(filename, xyz_s, irt_s)

    def _single_velodyne(self, msg, data):
        ## Velodyne (standard)
        #  field_names = ["x", "y", "z", "intensity", "ring", "time"]
        # Velodyne (no point's time)
        field_names = ["x", "y", "z", "intensity", "ring"]

        # Timestamp and file name
        timestamp = msg_to_timestamp(msg)
        filename = str(timestamp) + ".ply"
        data.append(filename)

        points = pc2.read_points_list(
            cloud=msg, field_names=field_names, skip_nans=True
        )

        xyz_s = []
        irt_s = []
        for point in points:
            if _POINT_TIME_AVAILABLE:
                x, y, z, i, r = point
                xyz = [x, y, z]
                irt = [i, r, 0]
            else:
                x, y, z, i, r, t_s = point
                xyz = [x, y, z]
                t_ms = int(t_s * 1e9)
                irt = [i, r, t_ms]

            xyz_s.append(xyz)
            irt_s.append(irt)

        xyz_s = np.array(xyz_s)
        irt_s = np.array(irt_s)

        self._save_ply(filename, xyz_s, irt_s)

    def _save_ply(self, name, points, normals):
        filename = os.path.join(self._path_save, name)
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(points)

        if normals.any():
            o3d_cloud.normals = o3d.utility.Vector3dVector(normals)

        o3d.io.write_point_cloud(filename, o3d_cloud)

    def _save_csv(self, data: np.ndarray):
        filename = os.path.join(self._path_save, "data.csv")
        np.savetxt(
            filename,
            data,
            fmt=self._FORMAT,
            delimiter=",",
            newline="\n",
            header=self._HEADER,
            footer="",
            comments="# ",
            encoding=None,
        )
