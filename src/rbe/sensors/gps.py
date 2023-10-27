import os

import numpy as np
import yaml
from tqdm import tqdm

from rbe.sensors.base import BaseSensor
from rbe.utils.tools import msg_to_timestamp


class GPSSensor(BaseSensor):
    _HEADER = "timestamp [ns], latitude, longitude, altitude"
    _FORMAT = "%d, %10.20f, %10.20f, %10.20f"

    def __init__(self, topic, bags) -> None:
        super().__init__(topic, bags, "gps")

    def _print_init_msg(self):
        print(">> Extracting GPS:")

    def _extract(self):
        data = []
        for idx, bag in enumerate(self._bags):
            print("{}/{}: {}".format(idx + 1, len(self._bags), bag.filename))
            bag_msgs = bag.read_messages(topics=[self._topic])
            msg_count = bag.get_message_count(self._topic)

            for idx, (_, msg, _) in tqdm(enumerate(bag_msgs), total=msg_count):
                coor_dict = yaml.load(str(msg), Loader=yaml.FullLoader)

                single = [msg_to_timestamp(msg)]
                single.append(coor_dict["latitude"])
                single.append(coor_dict["longitude"])
                single.append(coor_dict["altitude"])

                if single[-1] == "nan":
                    single[-1] = 0

                data.append(single)

        data = np.array(data)
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
