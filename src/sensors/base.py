import os
from datetime import date


class BaseExtraction:

    def __init__(self, topic, bags, path_root, name="") -> None:
        self.topic = topic
        self.bags = bags
        self.path_root = path_root
        self.name = name if name else self.topic


    def getPathSave(self) -> str:
        path_save = os.path.join(self.path_root, self.name)
        if not os.path.exists(path_save):
            os.mkdir(path_save)
        else:
            today = date.today()
            path_save += today.strftime("%Y%m%d")
            if not os.path.exists(path_save):
                os.mkdir(path_save)

        return path_save

    def isAvailable(self) -> bool:
        for bag in self.bags:
            if not self.topic in bag.get_type_and_topic_info().topics:
                _, bag_name = os.path.split(bag._filename)
                self.printMsg("topic \"{}\" is not recorded in \"{}\"".format(
                    self.topic, bag_name))
                return False

        return True

    def printMsg(self, msg):
        print("{}: {}".format(self.name, msg))

    def extract(self) -> bool:
        raise NotImplementedError("Implement this method")
