import os
from typing import Any, List

import numpy as np
import yaml
from tqdm import tqdm

from rbe.sensors.base import BaseSensor
from rbe.utils.pose import Pose


class PoseSensor(BaseSensor):
    _HEADER = "timestamp [ns], pose [affine]"
    _FORMAT = "%d, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f"

    def __init__(self, topic, bags) -> None:
        super().__init__(topic, bags, "poses")

    def _print_init_msg(self):
        print(">> Extracting Poses:")

    def _extract(self):
        data = []
        for idx, bag in enumerate(self._bags):
            print("{}/{}: {}".format(idx + 1, len(self._bags), bag.filename))
            bag_msgs = bag.read_messages(topics=[self._topic])
            msg_count = bag.get_message_count(self._topic)

            for idx, (topic, msg, t) in tqdm(enumerate(bag_msgs), total=msg_count):
                coor_dict = yaml.load(str(msg), Loader=yaml.FullLoader)

                single: List[Any] = []
                single.append(t.to_nsec())
                # single.append(msg_to_timestamp(msg))

                x = coor_dict["pose"]["position"]["x"]
                y = coor_dict["pose"]["position"]["y"]
                z = coor_dict["pose"]["position"]["z"]
                position = np.array([x, y, z])
                x = coor_dict["pose"]["orientation"]["x"]
                y = coor_dict["pose"]["orientation"]["y"]
                z = coor_dict["pose"]["orientation"]["z"]
                w = coor_dict["pose"]["orientation"]["z"]
                quaternion = np.array([x, y, z, w])

                pose = Pose()
                pose.setPosition(position)
                pose.setQuaternion(quaternion)
                single.extend(pose.getList())

                data.append(single)

        data = np.array(data)
        filename = os.path.join(self._path_save, "poses.csv")
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
