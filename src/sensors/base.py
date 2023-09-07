import os
from datetime import date


class BaseSensor:
    def __init__(self, topic, bags, path_root, name="") -> None:
        self._topic = topic
        self._bags = bags
        self._path_root = path_root
        self._name = name if name else self._topic

        self.check_topic_data()
        self._path_save = self.get_path_save()

    def check_topic_data(self) -> bool:
        for bag in self._bags:
            if not self._topic in bag.get_type_and_topic_info().topics:
                _, bag_name = os.path.split(bag._filename)
                raise RuntimeError(
                    'The topic "{}" is not recorded in the rosbag file "{}"'.format(
                        self._topic, bag_name
                    )
                )
        return True

    def get_path_save(self) -> str:
        path_save = os.path.join(self._path_root, self._name)
        if not os.path.exists(path_save):
            os.mkdir(path_save)
        else:
            today = date.today()
            path_save += today.strftime("%Y%m%d")
            if not os.path.exists(path_save):
                os.mkdir(path_save)
        return path_save

    def extract(self):
        self._print_init_msg()
        self._extract()

    def _print_init_msg(self):
        print(">> Extracting [BASE SENSOR]:")

    def _extract(self):
        raise NotImplementedError()
