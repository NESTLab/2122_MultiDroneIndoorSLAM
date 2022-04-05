import rospy
import trio
from typing import List, Tuple
from threading import Lock
from coms.p2p import Server, Client
from coms.constants import PUB_TOPIC, DEBUG_TOPIC, SUB_TOPIC
from coms.utils import publish_nearby_robots, debug
from std_msgs.msg import String
from coms.srv import NextMeeting, NextMeetingResponse, NextMeetingRequest
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
    
    def send_next_meeting(self, req: NextMeetingRequest) -> NextMeetingResponse:
        local_ip, port = self.LISTEN_ADDRESS
        neighbor: str = req.robot_ip
        point: Tuple[int, int, int] = req.point
        time_to_meet: float = req.time
        client = Client(local_ip, self.debug, self.namespace)
        return trio.run(client.next_meeting, neighbor, port, point, time_to_meet)
    
    def sub_handler(self, str_struct, cb_args=None) -> None:
        """
        We can do so many things with this topic.
        In order to trigger specific logic, we follow this message schema:
        Message                     Action                  Params
        sync|192.168.0.4|role       Performs map sync       role = relay | explorer
        """
        msg:str = str_struct.data
        debug(self.debug, f"Recieving message from topic {self.namespace + SUB_TOPIC}: {msg} [TOPIC]")
        parts = msg.split('|')
        if parts[0] == 'sync':
            debug(self.debug, f"Recieving message from topic {self.namespace + SUB_TOPIC}: {msg} to sync [TOPIC]")
            neighbor = parts[1]
            role = parts[2]
            if role != self.role:
                debug(self.debug, f"Incorrect role. You gave {role} but the robot is {self.role} [TOPIC ERROR]")
                return
            local_ip, port = self.LISTEN_ADDRESS
            client = Client(local_ip, self.debug, self.namespace)
            trio.run(client.sync, neighbor, port, role)
        else:
            debug(self.debug, f"Malformed topic message {self.namespace + SUB_TOPIC}: {msg} [TOPIC WARNING]")
    
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

        

