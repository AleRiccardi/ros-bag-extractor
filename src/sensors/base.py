import os
from datetime import date
from pathlib import Path
from typing import List

import rosbag


class BaseSensor:
    def __init__(self, topic: str, bags: List[rosbag.Bag], name: str = "base") -> None:
        self._topic: str = topic
        self._bags: List[rosbag.Bag] = bags
        self._name: str = name if name else self._topic

        self._check_topic_data()
        self._path_save: Path = self._get_path_save()

    def _check_topic_data(self) -> bool:
        for bag in self._bags:
            if not self._topic in bag.get_type_and_topic_info().topics:
                _, bag_name = os.path.split(bag.filename)
                raise RuntimeError(
                    'The topic "{}" is not recorded in the rosbag file "{}"'.format(
                        self._topic, bag_name
                    )
                )
        return True

    def _get_path_save(self) -> Path:
        filename = Path(self._bags[0].filename)

        if len(self._bags) > 1:
            path_root = filename.parent / "all"
        else:
            path_root = filename.with_suffix("")

        path_save = path_root / self._name

        if not os.path.exists(path_save):
            path_save.mkdir(parents=True)
        else:
            today = date.today()
            path_save = Path(path_save.as_posix() + "_" + today.strftime("%Y%m%d"))
            if not os.path.exists(path_save):
                path_save.mkdir(parents=True)
        return path_save

    def _print_init_msg(self):
        print(">> Extracting [BASE SENSOR]:")

    def extract(self):
        self._print_init_msg()
        self._extract()

    def _extract(self):
        raise NotImplementedError()
