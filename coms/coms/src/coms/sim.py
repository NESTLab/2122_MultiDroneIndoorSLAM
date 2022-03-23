from __future__ import annotations
import time
import socket
import roslaunch
import rospy
from threading import Lock, Thread
from socketserver import BaseRequestHandler, TCPServer, ThreadingMixIn, ThreadingTCPServer
from typing import List, Tuple
from subprocess import check_output, call
from std_msgs.msg import String
from coms.msg import nearby
from msg.ping import Ping
from coms.constants import QUICK_WAIT_TIMER, RESPONSE_TIMEOUT, BROADCAST_INTERVAL, ENCODING, CATKIN_WS, SUB_TOPIC, PUB_TOPIC # noqa: E501
from coms.utils import get_interface_from_ip, get_port_from, get_ip_list, get_device_numbers, addr_to_str
from concurrent.futures import ThreadPoolExecutor, Future
from msg.message import Message
from msg.merge import Merge
from msg.utils import get_message_type
from mapmerge.service import mapmerge_pipeline
import numpy as np

def compute_score(map: np.ndarray) -> int:
    score = 0
    for row in map:
        for cell in row:
            # NOT unknown
            if cell != 127:
                score += 1
    return score


class ThreadedTCPServer(ThreadingMixIn, TCPServer):
    def server_bind(self: ThreadingTCPServer) -> None:
        self.socket.settimeout(RESPONSE_TIMEOUT)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.server_address)



class Sim():
    LOCAL_IPS: List[str] = []                               # List of ip addresses defined in configuration file
    LISTEN_ADDRESS: Tuple[str, int] = ()                    # Address bound to the listener's TCP socket server
    NET_PROC: roslaunch.parent.ROSLaunchParent = None       # ROS specific launch object - for starting sim network
    NET_SIM_LAUNCH_FILE: str = None                         # Path to sim network launch file
    thread_executor: ThreadPoolExecutor = None              # Executor for managing threaded operations
    thread_tasks: List[Future] = []                         # List of Future objects for obtaining of thread stats
    keep_runing: Lock = None                                # Mutex for threads to determin if they should keep running
    pub: rospy.Publisher = None                             # ROS Publisher for sending msg responses to other nodes
    sub: rospy.Subscriber = None                            # ROS Subscriber for listening to message requests
    namespace: str = ""                                     # Robot namespace for use in publishing robot specific topics
    scores_lock: Lock()
    local_map: np.ndarray = None
    neighbor_map_scores: List[Tuple(str, int)] = []
    local_score: int = 0

    def __init__(self: Sim, address: str, net_sim_launch_file: str = None, net_config: str = "testing.yaml", namespace: str = "/robot_X/", local_map: np.ndarray = None) -> None:
        path_to_config = check_output("find {0} -type f -name '{1}'".format(CATKIN_WS, net_config), shell=True)
        self.LOCAL_IPS = get_ip_list(path_to_config.decode(ENCODING).strip())
        self.NET_SIM_LAUNCH_FILE = net_sim_launch_file
        self.LISTEN_ADDRESS = (address, get_port_from(address, True))
        self.keep_runing = Lock()
        self.scores_lock: Lock = Lock()
        self.namespace = namespace
        self.local_map = local_map
        self.local_score = compute_score(local_map)
        for ip in self.LOCAL_IPS:
            self.neighbor_map_scores.append((ip, 0))
        

    def start(self: Sim) -> None:
        # Start simulated network
        # if self.NET_SIM_LAUNCH_FILE is not None and self.NET_PROC is None and not is_sim_network_running(self.LOCAL_IPS):
        #     self.NET_PROC = launch_sim_network(self.NET_SIM_LAUNCH_FILE, self.LOCAL_IPS)
        # Setup Pub/Sub topics
        self.register_ros_topics()
        # Launch listener and broadcaster threads
        self.executor: ThreadPoolExecutor = ThreadPoolExecutor(max_workers=2)
        self.keep_runing.acquire()
        self.thread_tasks = [
            self.executor.submit(self._listener),
            self.executor.submit(self._broadcaster),
        ]
        # Ensure all tasks are running
        for t in self.thread_tasks:
            while not t.running():
                print("NOT RUNNING")
                print("Done:", t.done())
                if t.done():
                    return
                time.sleep(QUICK_WAIT_TIMER)

    def stop(self: Sim) -> None:
        # if self.NET_SIM_LAUNCH_FILE is not None and self.NET_PROC is not None:
        #     terminate_sim_network(self.NET_PROC, self.LOCAL_IPS)
        # Signal threads to terminate
        self.keep_runing.release()
        # Wait for all thredads to stop
        self.executor.shutdown(wait=True)
        # Shutdown Pub/Sub channels
        self.unregister_ros_topics()

    def send_messsage(self: Sim, nic: str, destination: Tuple[str, int], message: Message) -> None:
        byte_buffer = 10000
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, 25, str(nic + '\0').encode(ENCODING))
            sock.settimeout(1)
            try:
                sock.connect(destination)
                if message.id == 0:
                    message: Merge = message
                    p = message.produce_payload()
                    sock.sendall(p)

                    # msg: Merge = Merge().consume_payload(sock.sock.recv(4096))
                    # print("**********************************************************************")
                    # print("**********************************************************************")
                    # print("**********************************************************************")
                    # print("**********************************************************************")
                    # print("**********************************************************************")

                    # new_map = mapmerge_pipeline(map1=message.map, map2=msg.map)
                    # score = compute_score(new_map)

                    # self.scores_lock.acquire()
                    # self.local_score = score
                    # self.local_map = new_map
                    # self.neighbor_map_scores[msg.source[0]] = msg.score
                    # self.scores_lock.release()


                    # print("\n\nRobot: {} successfully merged with {}\n\n".format(msg.target, msg.source))

                elif message.id == 1:
                    p: Ping = message
                    p.source = sock.getsockname()
                    p.destination = sock.getpeername()
                    sock.sendall(message.produce_payload())
                else:
                    raise Exception("Attempted to send unsupported message type")
                # response = message.consume_payload(sock.recv(4096))
                # response.handle()

            except Exception as e:
                print(e)
            sock.close()

    def override_handle(self: Sim, r: BaseRequestHandler) -> None:
        # print("handling request: ", self.request)
        # self.request.close()
        try:
            request: socket.socket = r.request
            raw_data: bytes = request.recv(40096)
            m_type = get_message_type(raw_data)
            m_struct: Message = None
            if m_type == 'Merge':
                print("---------- MERGE ")
                m_struct = Merge()
            elif m_type == 'Ping':
                print("---------- PING ")
                m_struct = Ping()
            else:
                print("Unrecognized message, dropping connection")
                return

            msg: Message = m_struct.consume_payload(raw_data)

            if isinstance(msg, Ping):
                msg.handle()
                resp = Ping(source=request.getsockname(), destination=request.getpeername())
                request.sendall(resp.produce_payload())
            elif isinstance(msg, Merge):

                new_map = mapmerge_pipeline(map1=self.local_map, map2=msg.map)
                score = compute_score(new_map)

                print("source", msg.source)
                self.scores_lock.acquire()
                self.local_score = score
                self.local_map = new_map
                self.neighbor_map_scores[msg.source[0]] = msg.score
                self.scores_lock.release()
            else:
                # print("Unrecognized message, dropping connection")
                return
        except Exception as e:
            print("========================================")
            print("========================================")
            print("========================================")
            print("========================================")
            print(e)

    def server(self) -> None:
        self.ThreadedTCPRequestHandler.handle = lambda x: self.override_handle(x)
        server = ThreadedTCPServer(self.LISTEN_ADDRESS, self.ThreadedTCPRequestHandler)
        with server:
            # Start a thread with the server -- that thread will then start one
            # more thread for each request
            server_thread = Thread(target=server.serve_forever)
            # Exit the server thread when the main thread terminates
            server_thread.daemon = True
            server_thread.start()
            print("Server loop running in thread:", server_thread.name)

            self.keep_runing.acquire()
            self.keep_runing.release()

            server.shutdown()
            server.socket.close()

    def _listener(self: Sim) -> None:
        # Blocks untill kill_thread_event is set
        print("Starting listener at :", self.LISTEN_ADDRESS)
        self.server()
        print("Finished listening at :", self.LISTEN_ADDRESS)

    # def __listener(self: Sim) -> None:
    #     # Blocks untill kill_thread_event is set
    #     print("Starting listener at :", self.LISTEN_ADDRESS)
    #     server(self.LISTEN_ADDRESS, self.keep_runing)
    #     print("Finished listening at :", self.LISTEN_ADDRESS)

    # TODO: Fix broadcaster
    # def _broadcaster(self: Sim) -> None:
    #     print("Starting broadcaster for :", self.LISTEN_ADDRESS[0])
    #     nic = get_interface_from_ip(self.LISTEN_ADDRESS[0])
    #     if nic == '':
    #         raise Exception("Broadcaster" + self.LISTEN_ADDRESS[0] + "could not retrieve a valid network interface!")
    #     local_addr = (self.LISTEN_ADDRESS[0], get_port_from(self.LISTEN_ADDRESS[0], False))
    #     while self.keep_runing.locked():
    #         neighbors = self.get_reachable_ips(nic)
    #         self.publish_nearby_robots(neighbors)
    #         for neighbor in neighbors:
    #             send_messsage(
    #                 nic=nic,
    #                 destination=neighbor,
    #                 message=Ping(
    #                     source=local_addr,
    #                     destination=neighbor
    #                 ))
    #         time.sleep(BROADCAST_INTERVAL)
    #     print("Finished broadcasting for :", self.LISTEN_ADDRESS[0])

    # keep track of how much map data each robot has
    # we have more information than a neighbor, send them a sync request
    def _broadcaster(self: Sim) -> None:
        print("Starting broadcaster for :", self.LISTEN_ADDRESS[0])
        nic = get_interface_from_ip(self.LISTEN_ADDRESS[0])
        if nic == '':
            raise Exception("Broadcaster" + self.LISTEN_ADDRESS[0] + "could not retrieve a valid network interface!")
        local_addr = (self.LISTEN_ADDRESS[0], get_port_from(self.LISTEN_ADDRESS[0], False))
        
        while self.keep_runing.locked():
            neighbors = self.get_reachable_ips(nic)
            # self.publish_nearby_robots(neighbors)
            for neighbor in neighbors:
                self.send_messsage(
                    nic=nic,
                    destination=neighbor,
                    message=Merge(
                        map=self.local_map,
                        source=local_addr,
                        target=neighbor,
                        score=self.local_score
                    ))
            time.sleep(BROADCAST_INTERVAL)
        print("Finished broadcasting for :", self.LISTEN_ADDRESS[0])

    def get_reachable_ips(self: Sim, nic: str) -> List[Tuple[str, int]]:
        neighbors = []
        for ip in self.LOCAL_IPS:
            if ip != self.LISTEN_ADDRESS[0]:
                destination = (ip, get_port_from(ip, True))
                # Create non-blocking socket, bound to the local address for broadcasting
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    sock.setsockopt(socket.SOL_SOCKET, 25, str(nic + '\0').encode('utf-8'))
                    sock.settimeout(RESPONSE_TIMEOUT)
                    try:
                        sock.connect(destination)
                        neighbors.append(sock.getpeername())
                        sock.close()
                    except Exception as e:
                        print("Robot {0} not in line-of-site".format(destination))
                        print(e)
                    sock.close()

        return neighbors

    def register_ros_topics(self: Sim) -> None:
        self.pub = rospy.Publisher(
            name=self.namespace + PUB_TOPIC,
            data_class=nearby,
            queue_size=10)

        # NOTE: The Subscriber callback only accepts two params (according to the docs)
        # rospy.Subscriber.callback: fn(msg, cb_args)
        # This forces us to use a lambda function to ignore the 'self'
        # source: http://docs.ros.org/en/melodic/api/rospy/html/rospy.topics.Subscriber-class.html
        self.sub = rospy.Subscriber(
            name=self.namespace + SUB_TOPIC,
            data_class=String,
            callback=lambda msg: self.listen_handler(msg, cb_args=None))

    def unregister_ros_topics(self: Sim) -> None:
        if self.pub is not None:
            self.pub.unregister()
        if self.sub is not None:
            self.sub.unregister()

    def listen_handler(self: Sim, msg: rospy.AnyMsg, cb_args: any) -> None:
        rospy.loginfo("GOT MESSAGE")
        print('got -> ', msg.data)

    def publish_nearby_robots(self: Sim, addresses: List[Tuple[str, int]]) -> None:
        if len(addresses) == 0:
            return
        addr_strings = []
        for addr in addresses:
            if addr[0] != self.LISTEN_ADDRESS[0]:
                addr_strings.append(addr_to_str(addr))
        payload = nearby()
        payload.remote_addresses = addr_strings
        payload.local_address = addr_to_str(self.LISTEN_ADDRESS)
        self.pub.publish(payload)
    
    class ThreadedTCPRequestHandler(BaseRequestHandler):
        def handle(self: BaseRequestHandler) -> None:
            pass


def is_sim_network_running(local_ips: List[str]) -> bool:
    num_devs = len(local_ips)
    out = str(check_output(["sudo", "-S", "ip", "rule", "list"]))
    if out.count('lookup') != num_devs * 2 + 3 or len(get_device_numbers()) != num_devs:
        return False
    # Ensure all ips are present
    for ip in local_ips:
        if out.count(ip) != 1:
            return False
    return True


# Returns ROS launch parent
def launch_sim_network(launch_file: str, local_ips: List[str]) -> roslaunch.parent.ROSLaunchParent:
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [launch_file],
        force_screen=False)
    launch.start()
    while not is_sim_network_running(local_ips):
        time.sleep(QUICK_WAIT_TIMER)
    return launch


def terminate_sim_network(launch: roslaunch.parent.ROSLaunchParent, local_ips: List[str]) -> None:
    launch.shutdown()
    for i in get_device_numbers():
        remove_net_tunnel(i)
    remove_net_rules()
    while is_sim_network_running(local_ips):
        time.sleep(QUICK_WAIT_TIMER)


def remove_net_tunnel(i: int) -> None:
    call(["sudo", "-S", "ip", "tuntap", "del", "dev", "tun" + str(i), "mode", "tun"])
    call(["sudo", "ip", "link", "delete", "dev", "tun" + str(i)])
    call(["sudo", "ip", "rule", "del", "table", str(i + 1)])
    call(["sudo", "ip", "rule", "del", "table", str(i + 101)])


def remove_net_rules() -> None:
    call(["sudo", "ip", "rule", "add", "pref", "0", "from", "all", "lookup", "local"])
    call(["sudo", "ip", "rule", "del", "pref", "10", "from", "all", "lookup", "local"])


# # Create SIMPLEST form of:
# # listener and broadcaster

# # Simply send and recieve strings

# # Create a Message Interface

# # Create a dummyMessage to impliment it (sends a string as a payload)
