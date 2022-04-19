import rospy
import trio
from typing import List, Tuple
from threading import Lock
from coms.p2p import Server, Client
from coms.constants import PUB_TOPIC, DEBUG_TOPIC, SUB_TOPIC
from coms.utils import publish_nearby_robots, debug
from std_msgs.msg import String
from coms.srv import NextMeeting, NextMeetingResponse, NextMeetingRequest, ReadyToMeet, ReadyToMeetRequest, ReadyToMeetResponse, NetInfo, NetInfoRequest, NetInfoResponse
from coms.msg import nearby

class Lite_Simulator():
    NEIGHBOR_IPS: List[str] = []
    LISTEN_ADDRESS: Tuple[str, int] = ()
    RUNNING: Lock
    def __init__(self, local_address: Tuple[str, int], neighbor_ips: List[str], namespace: str, role: str) -> None:
        self.LISTEN_ADDRESS = local_address
        self.NEIGHBOR_IPS = neighbor_ips
        self.RUNNING = Lock()
        self.namespace = namespace
        self.role = role
        self.next_meeting_service = rospy.Service("send_next_meeting", NextMeeting, self.send_next_meeting)
        self.get_ready_to_meet = rospy.Service("ready_to_meet", ReadyToMeet, self.get_ready_to_meet_status)
        self.info_service = rospy.Service("get_info", NetInfo, self.get_info)
        self.sub = rospy.Subscriber(
            name=namespace + SUB_TOPIC,
            data_class=String,
            callback=lambda msg: self.sub_handler(msg, cb_args=None))
        self.pub = rospy.Publisher(
            name=namespace + PUB_TOPIC,
            data_class=nearby,
            queue_size=20)
        self.debug = rospy.Publisher(
            name=namespace + DEBUG_TOPIC,
            data_class=String,
            queue_size=20)
    
    def get_info(self, req: NetInfoRequest) -> NetInfoResponse:
        local_ip, port = self.LISTEN_ADDRESS
        remote_ip: str = req.remote_ip
        state: str = req.state
        parent_ip: str = req.parent_ip
        child_ip: str = req.child_ip
        role: str = req.role
        debug(self.debug, f'Call to send_next_meeting ros service "{req}" [SERVICE INFO]')
        client = Client(local_ip, self.debug, self.namespace)
        info_dict = trio.run(client.info, remote_ip, port, state, local_ip, parent_ip, child_ip, role)
        response = NetInfoResponse()
        response.state = info_dict['state']
        response.ip = info_dict['address']
        response.parent_ip = info_dict['parent']
        response.child_ip = info_dict['child']
        response.role = info_dict['role']
        return response

    def send_next_meeting(self, req: NextMeetingRequest) -> NextMeetingResponse:
        local_ip, port = self.LISTEN_ADDRESS
        neighbor: str = req.robot_ip
        point: Tuple[int, int, int] = req.point
        time_to_meet: float = req.time
        debug(self.debug, f'Call to send_next_meeting ros service "robot_ip: {neighbor} | point: {point} | time: {time_to_meet}" [SERVICE SEND_NEXT_MEETING]')
        client = Client(local_ip, self.debug, self.namespace)
        return trio.run(client.next_meeting, neighbor, port, point, time_to_meet)
    
    def sub_handler(self, str_struct, cb_args=None) -> None:
        """
        We can do so many things with this topic.
        In order to trigger specific logic, we follow this message schema:

        PARAMS (all strings)
        role = relay | explorer
        status = true | false
        *ip* = 192.168.0.4 or any IP
        state = <undecided> # TODO: Select which states to broadcast

        MESSAGE                                         ACTION                  
        sync|remote_ip|role                             Performs map sync
        ready|remote_ip|status                          Query ready to meet
        info|remote_ip|state|parent_ip|child_ip|role    Retrieve robot information
        """
        local_ip, port = self.LISTEN_ADDRESS
        client = Client(local_ip, self.debug, self.namespace)
        msg:str = str_struct.data
        debug(self.debug, f"Recieving message from topic {self.namespace + SUB_TOPIC}: {msg} [TOPIC]")
        parts = msg.split('|')
        if parts[0] == 'sync':
            # sync|remote_ip|role
            neighbor = parts[1]
            role = parts[2]
            if role != self.role:
                debug(self.debug, f"Incorrect role. You gave {role} but the robot is {self.role} [TOPIC ERROR]")
                return
            return trio.run(client.sync, neighbor, port, role)
        elif parts[0] == 'ready':
            # ready|remote_ip|status
            neighbor = parts[1]
            status = (parts[2]).lower() == 'true'
            trio.run(client.ready_to_meet, neighbor, port, status)
        elif parts[0] == 'info':
            # info|remote_ip|state|parent_ip|child_ip|role
            remote_ip = parts[1]
            state = parts[2]
            parent_ip = parts[3]
            child_ip = parts[4]
            role = parts[5]
            if role != self.role:
                debug(self.debug, f"Incorrect role. You gave {role} but the robot is {self.role} [TOPIC ERROR]")
                return
            trio.run(client.info, remote_ip, port, state, local_ip, parent_ip, child_ip, role)
        else:
            debug(self.debug, f"Malformed topic message {self.namespace + SUB_TOPIC}: {msg} [TOPIC WARNING]")
    
    def get_ready_to_meet_status(self, req: ReadyToMeetRequest) -> ReadyToMeetResponse:
        ip, port = self.LISTEN_ADDRESS
        remote_ip = str(req.robot_ip)
        status = bool(req.status)
        debug(self.debug, f'Call to ready_to_meet ros service "robot_ip: {remote_ip}" [SERVICE READY_TO_MEET]')
        client = Client(ip, self.debug, self.namespace)
        return trio.run(client.ready_to_meet, remote_ip, port, status)

    async def _synchronizer(self) -> None:
        ip, port = self.LISTEN_ADDRESS
        client = Client(ip, self.debug, self.namespace)
        while True:
            for neighbor in self.NEIGHBOR_IPS:
                # Use same port as local listener
                if neighbor != ip:
                    did_sync = await client.sync(neighbor, port, role=self.role)
                    if did_sync:
                        debug(self.debug, f"Synchronizer merged with neighbor {neighbor} [SUCCESS]")
            await trio.sleep(2)
 
    async def _listener(self) -> None:
        ip, port = self.LISTEN_ADDRESS
        server = Server(ip, port, self.debug, self.namespace, self.role)
        await server.serve()

    async def _nearby_pinger(self) -> None:
        ip, port = self.LISTEN_ADDRESS
        client = Client(ip, self.debug, self.namespace)
        while True:
            nearby_ips = []
            for neighbor in self.NEIGHBOR_IPS:
                # Use same port as local listener
                success = await client.ping(neighbor, port)
                if success:
                    nearby_ips.append(neighbor)
            publish_nearby_robots(self.pub, ip, nearby_ips)
            await trio.sleep(0.2)
    
    async def runner(self) -> None:
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self._listener)
            nursery.start_soon(self._synchronizer)
            nursery.start_soon(self._nearby_pinger)

    def run(self) -> None:
        trio.run(self.runner)

        

