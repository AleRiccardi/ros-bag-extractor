import multiprocessing
import os

import numpy as np
import yaml
from sensors.base import BaseSensor
from tqdm import tqdm
from utils.multiprocessors import proc_join_all, proc_start_check
from utils.tools import msg_to_timestamp


class IMUSensor(BaseSensor):
    _HEADER = "timestamp [ns], qat_x, qat_y, qat_z, qat_w, acc_x, acc_y, acc_z, ang_x, ang_y, ang_z"
    _FORMAT = "%d, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f, %10.20f"

    def __init__(self, topic, bags) -> None:
        super().__init__(topic, bags, "imu")

    def _print_init_msg(self):
        print(">> Extracting IMUs:")

    def _extract(self):
        manager = multiprocessing.Manager()
        data = manager.list()

        for idx, bag in enumerate(self._bags):
            print("{}/{}: {}".format(idx + 1, len(self._bags), bag.filename))
            bag_msgs = bag.read_messages(topics=[self._topic])
            msg_count = bag.get_message_count(self._topic)

            processes = []
            for idx, (_, msg, _) in tqdm(enumerate(bag_msgs), total=msg_count):
                p = multiprocessing.Process(target=self._single_imu, args=(msg, data))
                proc_start_check(p, processes)
            proc_join_all(processes)

        data = np.array(data)
        data_idx_sort = np.argsort(data, axis=0)
        data = data[data_idx_sort[:, 0]]

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

    def _single_imu(self, msg, data):
        imu_dict = yaml.load(str(msg), Loader=yaml.FullLoader)
        single = [msg_to_timestamp(msg)]

        single += imu_dict["orientation"].values()
        single += imu_dict["linear_acceleration"].values()
        single += imu_dict["angular_velocity"].values()

        # single += [0, 0, 0, 0]
        # single += imu_dict["accel"].values()
        # single += imu_dict["gyro"].values()

        data.append(single)
