from __future__ import annotations
from typing import Tuple
from msg.message import Message
from msg.utils import extract_payload_id
from coms.constants import ENCODING
import numpy as np
import rospy
import pickle
import pandas as pd


class Merge(Message):
    id = 0

    map: np.ndarray = None
    shape = 0, 0
    target: Tuple[str, int] = None
    source: Tuple[str, int] = None
    score: int = 0

    def __init__(self: Message,
                 map: np.ndarray = None,
                 source: Tuple[str, int] = None,
                 target: Tuple[str, int] = None,
                 score: int = 0) -> None:
        super().__init__()
        self.map: np.ndarray = map
        self.shape = None
        if map is not None:
            self.shape = map.shape
        self.source = source
        self.target = target
        self.score = score

    def produce_payload(self: Message) -> bytes:
        map_str = self.map.tostring()
        msg = f"{self.id}|{str(self.score)}|{self.source}|{self.target}|{self.shape[0]}|{self.shape[1]}|{map_str}"  # noqa: E501
        return msg.encode(ENCODING)

    def consume_payload(self: Message, payload: bytes) -> Message:
        if extract_payload_id(payload) != self.id:
            raise Exception("Merge message can't consume the following payload\n{0}".format(payload))  # noqa: E501

        def dissect_tuple_str(s: str) -> Tuple[str, int]:
            if s == '()':
                return ()
            s = s.replace("'", '')
            s = s.replace('"', '')
            s = s.replace("(", '')
            s = s.replace(")", '')
            parts = s.split(',')
            return (parts[0], int(parts[1], 10))

        msg: str = payload.decode(ENCODING)
        parts = msg.split("|")

        print("\n\n length of parts:", len(parts))
        # print(parts)


        source = dissect_tuple_str(parts[2])
        target = dissect_tuple_str(parts[3])
        # shape = parts[4], parts[5]
        # map_bytes = parts[6]
        score = int(parts[1], 10)

        # n_brackets = 0

        # raw_map: bytearray = None
        # for idx, b in enumerate(bytearray(payload)):
        #     if b == 124:
        #         n_brackets += 1
        #         if n_brackets == 6:
        #             raw_map = bytearray(payload[idx+1:])
        #             break
            
        # print(parts[6])
        # map = np.frombuffer(raw_map, dtype=np.uint8).reshape(parts[4], parts[5])
        map = np.fromstring(parts[6], dtype=np.uint8)
        return Merge(map, source, target, score)

    # NOTICE: The Listener must call this function on the Merge message is recieves.
    # Do NOT call this method without the values msg.target, msg.source, msg.map
    # THIS METHOD WILL BE CALLED FROM THE ROS MODULE TO OBTAIN OWN LOCAL MAP
    def handle(self: Message, ros: rospy) -> Message:
        pass
        # foreign_map: np.ndarray = self.map
        # foreign_map = self.create_occupancy_msg(foreign_map)
        # # send new map to map handler to perform merge
        # request_merge(ros, foreign_map)
        # # call NetSimMerger.request_merge() with foreign map
        # local_map = get_latest(ros)
        # # Construct Merge message as response
        # return Merge(map=local_map, source=self.target, target=self.source)

    def on_failure() -> None:
        pass