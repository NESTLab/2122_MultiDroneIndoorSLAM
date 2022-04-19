import unittest
import socket
import rospy
import numpy as np
import sys
from typing import List
from subprocess import check_output, call
from coms.utils import pack_bytes, add_padding, map_to_chunks, gen_id_chunk, decompress_map, compress_map, readable, writable, get_ip_list, get_interface_from_ip, get_device_numbers, gen_bound_socket, start_roscore, stop_roscore, addr_to_str # noqa: E501
from coms.constants import PADDING_CHAR, CHUNK_SIZE, CATKIN_WS, ENCODING
from numcompress import compress_ndarray
from roslaunch.parent import ROSLaunchParent


def create_tunnels(device_names: List) -> None:
    for name in device_names:
        call(["sudo", "-S", "ip", "tuntap", "add", "dev", name, "mode", "tun"])


def destroy_tunnels(device_names: List) -> None:
    for name in device_names:
        call(["sudo", "-S", "ip", "tuntap", "del", "dev", name, "mode", "tun"])


class TestUtils(unittest.TestCase):

    def test_readable(self: unittest.TestCase) -> None:
        sock = socket.socket()
        self.assertEqual(readable(sock), True, msg="Socket not found to be readable")
        sock.close()
        self.assertEqual(readable(sock), False, msg="Closed socket found to be readable")

    def test_writable(self: unittest.TestCase) -> None:
        sock = socket.socket()
        self.assertEqual(writable(sock), True, msg="Socket not found to be writable")
        sock.close()
        self.assertEqual(writable(sock), False, msg="Closed socket found to be writable")

    def test_get_ip_list(self: unittest.TestCase) -> None:
        path_to_config = check_output("find {0} -type f -name '{1}'".format(CATKIN_WS, "testing.yaml"), shell=True)
        local_ips = get_ip_list(path_to_config.decode(ENCODING).strip())
        self.assertEqual(local_ips, ["192.168.0.1", "192.168.0.2", "192.168.0.3"])

    def test_get_interface_from_ip(self: unittest.TestCase) -> None:
        # Loopback device will always have the same IP address
        self.assertEqual(
            get_interface_from_ip('127.0.0.1'),
            'lo',
            "Unexpected network interface found for loopback device")

    def test_get_device_numbers(self: unittest.TestCase) -> None:
        # If they are, they should be manually removed
        self.assertEqual(get_device_numbers(), [], "Detected unexpected tunnel device")
        # Create four tunnel devices
        create_tunnels(["tun0", "tun1", "tun2", "tun3"])
        self.assertEqual(get_device_numbers(), [0, 1, 2, 3], "Additional tunnel devices not observed")
        destroy_tunnels(["tun0", "tun1", "tun2", "tun3"])
        self.assertEqual(get_device_numbers(), [], "Detected unexpected tunnel device")

    def test_gen_bound_socket(self: unittest.TestCase) -> None:
        sock = gen_bound_socket('127.0.0.1')
        self.assertEqual(sock.family, socket.AF_INET, "Unexpected socket family")
        self.assertEqual(sock.type, socket.SOCK_STREAM, "Unexpected socket type")
        sock.close()
        self.assertRaises(Exception, gen_bound_socket, '')

    def test_start_roscore(self: unittest.TestCase) -> None:
        parent: ROSLaunchParent = start_roscore()
        self.assertEqual(parent.is_core, True, "is_core option NOT set in ROSLaunchParent")
        self.assertEqual(rospy.get_published_topics("/"), [], "roscore not running")
        parent.shutdown()

    def test_stop_roscore(self: unittest.TestCase) -> None:
        parent: ROSLaunchParent = start_roscore()
        self.assertEqual(rospy.get_published_topics("/"), [], "roscore not running")
        stop_roscore(parent)
        self.assertRaises(OSError, rospy.get_published_topics, "/")

    def test_addr_to_str(self: unittest.TestCase) -> None:
        tests = [
            (("", 0), ":0"),
            (("192.168.0.1", 177), "192.168.0.1:177"),
            (("1", 1), "1:1")
        ]
        for t in tests:
            got = addr_to_str(t[0])
            expects = t[1]
            self.assertEqual(got, expects)
    
    def test_compress_map(self: unittest.TestCase) -> None:
        maps: List[np.ndarray] = [
            np.random.randint(99999, size=(300,300)),
            np.random.randint(9999, size=(1,900)),
            np.random.randint(999999, size=(0,0)),
            np.random.randint(sys.maxsize, size=(800,800))
        ]
        for map in maps:
            raw_map: bytes = compress_map(map)
            true_raw_map: str = compress_ndarray(map, precision=0)
            real_map = decompress_map(raw_map)
            self.assertEqual(raw_map, true_raw_map.encode(ENCODING), "Map compression bytes is not consistent")
            self.assertEqual(raw_map.decode(ENCODING), true_raw_map, "Map compression is not consistent")
            self.assertEqual(real_map.shape, map.shape, "Decompressed maps are not the same shape")
            self.assertEqual((real_map == map).all(), True, "Decompressed maps are not equal")

    def test_decompress_map(self: unittest.TestCase) -> None:
        maps = [
            np.random.randint(9999, size=(300,300)),
            np.random.randint(999999, size=(1,900)),
            np.random.randint(9999, size=(0,0)),
            np.random.randint(sys.maxsize, size=(800,800))
        ]
        for map in maps:
            raw_map: str = compress_ndarray(map, precision=0)
            raw_map_bytes: bytes = raw_map.encode(ENCODING)
            dec_map = decompress_map(raw_map_bytes)
            self.assertEqual(dec_map.shape, map.shape, "Decompressed maps are not the same shape")
            self.assertEqual((dec_map == map).all(), True, "Decompressed maps are not equal")

    def test_gen_id_chunk(self: unittest.TestCase) -> None:
        ids = [0, 1, 9, 10, -18392732, sys.maxsize]
        for id in ids:
            id_str: str = str(id)
            pad: str = PADDING_CHAR * (CHUNK_SIZE - len(id_str))
            chunk: str = id_str + pad
            byte_chunk: bytes = chunk.encode(ENCODING)
            ref: bytes = gen_id_chunk(id)
            self.assertEqual(len(byte_chunk), len(ref), f"Bytes are different sizes. Should be CHUNK_SIZE: {CHUNK_SIZE}")
            self.assertEqual(byte_chunk, ref, f"Unequal bytes for id: {id}")
            self.assertEqual(chunk, ref.decode(ENCODING), f"Unequal bytes for id: {id}")

    def test_map_to_chunks(self: unittest.TestCase) -> None:
        map = np.random.randint(9999, size=(300,300))
        id = 4
        ref_chunks: List[bytes] = map_to_chunks(map, id, 'explorer')
        chunks: List[bytes] = [gen_id_chunk(id), add_padding('explorer')] + pack_bytes(compress_ndarray(map, precision=0).encode(ENCODING))
        self.assertEqual(len(chunks), len(ref_chunks))
        for idx, ref_chunk in enumerate(ref_chunks):
            real_chunk = chunks[idx]
            self.assertEqual(real_chunk, ref_chunk)

    def test_pack_bytes(self: unittest.TestCase) -> None:
        odd_len_bytes = ("\0" * (CHUNK_SIZE + 3)).encode(ENCODING)
        self.assertEqual(len(pack_bytes(odd_len_bytes)), 2)
        odd_len_bytes = ("\0" * (CHUNK_SIZE * 729 + 3)).encode(ENCODING)
        self.assertEqual(len(pack_bytes(odd_len_bytes)), 730)

        map = np.random.randint(sys.maxsize, size=(300,300))
        comp_map = compress_ndarray(map, precision=0)
        raw_map = comp_map.encode(ENCODING)
        ref_chunks = pack_bytes(raw_map)
        for chunk in ref_chunks:
            self.assertEqual((chunk in raw_map), True, "Raw chunk not found in raw map")


if __name__ == '__main__':
    unittest.main()
