import socket
import select
import yaml
import time
import netifaces
import rospy
from std_msgs.msg import String
from coms.msg import nearby
from typing import List, Dict, Tuple
from coms.constants import PADDING_CHAR, RESPONSE_TIMEOUT, ENCODING, STATIC_LISTENER_PORT, CHUNK_SIZE, MAP_MSG_ID # noqa: E501
from subprocess import check_output
from roslaunch.parent import ROSLaunchParent
from mapmerge.ros_utils import ros_to_numpy
from numcompress import compress_ndarray, decompress_ndarray
from nav_msgs.srv import GetMap
from trio import SocketStream
import numpy as np

def debug(pub: rospy.Publisher, data: str) -> None:
    filter = [
        "all attempts to connect"
    ]
    if pub is None:
        return
    for f in filter:
        if f in data:
            return
    pub.publish(String(data))

def readable(sock: socket.socket, timeout: float = RESPONSE_TIMEOUT) -> bool:
    try:
        ready_to_read, _, _ = select.select([sock], [], [], timeout)
        return len(ready_to_read) > 0
    except Exception:
        return False


def writable(sock: socket.socket, timeout: float = RESPONSE_TIMEOUT) -> bool:
    try:
        _, ready_to_write, _ = select.select([], [sock], [], timeout)
        return len(ready_to_write) > 0
    except Exception:
        return False


def get_ip_list(yaml_config_path: str) -> List[str]:
    with open(yaml_config_path, 'r') as file:
        return yaml.safe_load(file)['ip_list']


# Retrieves the name of the network interface bound to the given ip address.
# If no interface has the ip address given, an empty string will be returned.
def get_interface_from_ip(ip: str) -> str:
    af_inet: int = netifaces.AF_INET
    ifaces: List[str] = netifaces.interfaces()
    for interface in ifaces:
        iface_info: Dict = netifaces.ifaddresses(interface)
        if af_inet not in iface_info:
            continue
        ip_info_list: List = iface_info[af_inet]
        if len(ip_info_list) == 0:
            continue
        ip_info: Dict = ip_info_list[0]
        if 'addr' not in ip_info:
            continue
        addr: str = ip_info['addr']
        if addr == ip:
            return interface
    # Interface not found
    return ''


# Return list of tunnel device numbers
# If the OS had the following tunnels:  tun0, tun1, tun2
# get_device_numbers would return:      [0, 1, 2]
def get_device_numbers() -> List[int]:
    devices: List[int] = []
    out = check_output(["sudo", "-S", "ip", "tuntap", "list", "dev", "mode", "tun"])
    for line in out.splitlines():
        parts = line.decode(ENCODING).split(':')
        dev_number = int(parts[0][3], 10)
        devices.append(dev_number)
    return devices


# Returns TCP socket bound to the given ip address
def gen_bound_socket(ip: str) -> socket.socket:
    iface: str = get_interface_from_ip(ip)
    if iface == '':
        raise Exception("Unable to find network interface with given IP " + ip)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, 25, str(iface + '\0').encode('utf-8'))
    return sock


# IMPORTANT: This function is used as a function to manage ports for listener and broadcaster.
# Therefore, if we want to change the ports in any which way, we can change it once, right here!
def get_port_from(ip: str, listener: bool) -> int:
    # prefix: int = BROADCASTER_PORT_PREFIX
    # if listener:
    #     prefix = LISTENER_PORT_PREFIX
    # parts = ip.split('.')
    # postfix = parts[len(parts) - 1]
    # n = int(postfix, 10)
    # if n < 10:
    #     p = int(prefix / 10) * 10
    #     return p + n
    # p = int(prefix / 100) * 100
    # return p + n
    return STATIC_LISTENER_PORT


def start_roscore() -> ROSLaunchParent:
    parent = ROSLaunchParent("", [], is_core=True)     # run_id can be any string
    parent.start()
    return parent


def stop_roscore(parent: ROSLaunchParent) -> None:
    parent.shutdown()


def addr_to_str(addr: Tuple[str, int]) -> str:
    return "{0}:{1}".format(addr[0], addr[1])

def publish_nearby_robots(pub: rospy.Publisher, local_ip:str, addresses: List[str]) -> None:
    if len(addresses) == 0:
        return
    addr_strings = []
    for addr in addresses:
        if addr != local_ip:
            addr_strings.append(addr_to_str(f"{addr}:{STATIC_LISTENER_PORT}"))
    payload = nearby()
    payload.remote_addresses = addresses
    payload.local_address = local_ip
    pub.publish(payload)

def get_map(namespace: str) -> np.ndarray:
    return np.random.randint(2, size=(300,300))

def _get_map(namespace: str) -> np.ndarray:
    namespace = namespace.replace('/', '')
    pref = namespace.index('_')
    name = '/tb3' + namespace[pref:] + '/get_map'

    rospy.wait_for_service(name)
    while 1:
        try:
            map_service = rospy.ServiceProxy(name, GetMap)
            resp = map_service()
            return ros_to_numpy(resp.map.data).reshape(-1, resp.map.info.width)
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed: %s" % e)
        time.sleep(0.1)

def compress_map(map: np.ndarray) -> bytes:
    m: str = compress_ndarray(map, precision=0)
    return m.encode(ENCODING)

def decompress_map(raw: bytes) -> np.ndarray:
    m = raw.decode(ENCODING)
    return decompress_ndarray(m)

def gen_id_chunk(id: np.uint) -> bytes:
    result = str(id)
    padding = CHUNK_SIZE - len(result)
    return (result + PADDING_CHAR * padding).encode(ENCODING)

def add_padding(data: str) -> bytes:
    padding = CHUNK_SIZE - len(data)
    return (data + PADDING_CHAR * padding).encode(ENCODING)

def read_id_chunk(chunk: bytes) -> int:
    block = chunk.decode(ENCODING)
    block = block.replace(PADDING_CHAR, "")
    return int(block, 10)

def map_to_chunks(map: np.ndarray, id: int = MAP_MSG_ID, role: str = 'relay') -> List[bytes]:
    m: str = compress_ndarray(map, precision=0)
    raw_data = m.encode(ENCODING)
    return [gen_id_chunk(id), add_padding(role)] + pack_bytes(raw_data)

def pack_bytes(raw_data: bytes) -> List[bytes]:
    result = []
    while True:
        if len(raw_data) > CHUNK_SIZE:
            result.append(raw_data[:CHUNK_SIZE])
            raw_data = raw_data[CHUNK_SIZE:]
        else:
            result.append(raw_data)
            break
    return result


async def read_all_chunks(stream: SocketStream) -> Tuple[int, bytes]:
    msg_id = -1
    raw_resp = b''
    chunk = b''
    async for data in stream:
        chunk += data
        if len(chunk) >= CHUNK_SIZE:
            c = chunk[:CHUNK_SIZE]
            chunk = chunk[CHUNK_SIZE:]
            if msg_id == -1:
                msg_id = read_id_chunk(c)
            else:
                raw_resp += c
    if msg_id == -1:
        msg_id = read_id_chunk(chunk)
    else:
        raw_resp += chunk
    return (msg_id, raw_resp)

