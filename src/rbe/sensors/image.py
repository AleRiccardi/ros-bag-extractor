import multiprocessing
import os
from typing import Any, Optional

import cv2
import numpy as np
from cv_bridge import CvBridge
from tqdm import tqdm

from rbe.sensors.base import BaseSensor
from rbe.utils.multiprocessors import proc_join_all, proc_start_check
from rbe.utils.tools import msg_to_timestamp


class ImageSensor(BaseSensor):
    def __init__(self, topic, bags, encoding="yuv422") -> None:
        super().__init__(topic, bags, "images")
        self._encoding = encoding

    def _print_init_msg(self):
        print(">> Extracting images:")

    def _extract(self):
        for idx, bag in enumerate(self._bags):
            print("{}/{}: {}".format(idx + 1, len(self._bags), bag.filename))
            bag_msgs = bag.read_messages(topics=[self._topic])
            msg_count = bag.get_message_count(self._topic)
            bridge = CvBridge()

            processes = []
            for idx, (topic, msg, ts_bag) in tqdm(enumerate(bag_msgs), total=msg_count):
                p = multiprocessing.Process(
                    target=self._single_image, args=(msg, ts_bag, bridge)
                )
                proc_start_check(p, processes)
            proc_join_all(processes)

    def _single_image(self, msg: str, ts_bag: Optional[Any], bridge):
        if self._encoding == "standard":
            img_rgb = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        elif self._encoding == "yuv422":
            img_yuv422 = bridge.imgmsg_to_cv2(msg, desired_encoding="yuv422")
            img_bgr = cv2.cvtColor(img_yuv422, cv2.COLOR_YUV2BGR_UYVY)
        else:
            raise RuntimeError(
                'Image encoding "{}" not supported.'.format(self._encoding)
            )
        if ts_bag is not None:
            timestamp = ts_bag.to_nsec()
        else:
            timestamp = msg_to_timestamp(msg)
        self._save_image(img_bgr, timestamp)

    def _save_image(self, img: np.ndarray, timestamp: int):
        filename = str(timestamp) + ".png"
        path_file = os.path.join(self._path_save, filename)
        cv2.imwrite(path_file, img)
