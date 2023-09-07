import os

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

from sensors.base import BaseSensor
from utils.tools import msg_to_timestamp


class INSSensor(BaseSensor):
    _HEADER = "timestamp [ns], pose [affine]"
    _FORMAT = "%d, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f"

    def __init__(self, topic, bags) -> None:
        super().__init__(topic, bags, "ins")

    def _print_init_msg(self):
        print(">> Extracting Navigation and Quaternion:")

    def _extract(self):
        nav = []
        nav_ts = []
        rots = []
        quat_ts = []
        for idx, bag in enumerate(self._bags):
            print("{}/{}: {}".format(idx + 1, len(self._bags), bag.filename))
            bag_nav_msgs = bag.read_messages(topics=[self._topic])
            msg_nav_count = bag.get_message_count(self._topic)

            bag_quat_msgs = bag.read_messages(topics=[self._topic])
            msg_quat_count = bag.get_message_count(self._topic)

            print("Nav:")
            for idx, (_, msg_nav, _) in tqdm(
                enumerate(bag_nav_msgs), total=msg_nav_count
            ):
                ts = msg_to_timestamp(msg_nav)
                nav_dict = yaml.load(str(msg_nav), Loader=yaml.FullLoader)
                xyz = [
                    nav_dict["latitude"],
                    nav_dict["longitude"],
                    nav_dict["altitude"],
                ]
                nav_ts.append(ts)
                nav.append(xyz)

            print("Quat:")
            for idx, (_, msg, _) in tqdm(
                enumerate(bag_quat_msgs), total=msg_quat_count
            ):
                ts = msg_to_timestamp(msg)
                quat_dict = yaml.load(str(msg), Loader=yaml.FullLoader)
                xyzw = [
                    quat_dict["quaternion"]["x"],
                    quat_dict["quaternion"]["y"],
                    quat_dict["quaternion"]["z"],
                    quat_dict["quaternion"]["w"],
                ]
                # rotate around x (we want to have z facing upwards, and not downwards)
                calib = R.from_euler("x", [180], degrees=True).as_matrix()
                # some arbitrary random rotation (seems like we need it)
                rotz = R.from_euler("z", [90], degrees=True).as_matrix()
                # rotation in the LiDAR Frame!
                rot = (
                    rotz @ calib @ R.from_quat(xyzw).as_matrix() @ np.linalg.inv(calib)
                )
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

        file_path = self._path_save / "poses.txt"

        np.savetxt(
            file_path,
            time_poses,
            fmt=self._FORMAT,
            delimiter=",",
            newline="\n",
            header=self._HEADER,
            footer="",
            comments="# ",
            encoding=None,
        )
