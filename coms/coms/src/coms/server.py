import socket
import select
from typing import Tuple
from threading import current_thread, Lock, Thread
from socketserver import BaseRequestHandler, TCPServer, ThreadingMixIn, ThreadingTCPServer
from coms.constants import ENCODING, RESPONSE_TIMEOUT
from msg.message import Message
from msg.utils import get_message_type
from msg.ping import Ping
from msg.merge import Merge

def read_all(s: socket.socket) -> bytes:
    cunch_size = 1024
    return s.recv(cunch_size)

class ThreadedTCPRequestHandler(BaseRequestHandler):
    def handle(self: BaseRequestHandler) -> None:
        # print("handling request: ", self.request)
        # self.request.close()
        request: socket.socket = self.request
        raw_data: bytes = read_all(request)
        m_type = get_message_type(raw_data)
        m_struct: Message = None
        
        
        print("****************************************************")
        print("****************************************************")
        print(raw_data)
        print("****************************************************")
        print("****************************************************")
        
        if m_type == 'Merge':
            m_struct = Merge()
        elif m_type == 'Ping':
            m_struct = Ping()
        else:
            # print("Unrecognized message, dropping connection")
            self.request.close()
            return

        msg: Message = m_struct.consume_payload(raw_data)

        if isinstance(msg, Ping):
            msg.handle()
            resp = Ping(source=request.getsockname(), destination=request.getpeername())
            request.sendall(resp.produce_payload())
        elif isinstance(msg, Merge):
            print("Merge message")
            msg.handle()
            request.sendall(b'Merge')
        else:
            # print("Unrecognized message, dropping connection")
            return
            self.request.close()


class ThreadedTCPServer(ThreadingMixIn, TCPServer):
    def server_bind(self: ThreadingTCPServer) -> None:
        self.socket.settimeout(RESPONSE_TIMEOUT)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.server_address)


def send_messsage(nic: str, destination: Tuple[str, int], message: Message) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, 25, str(nic + '\0').encode(ENCODING))
        sock.settimeout(RESPONSE_TIMEOUT)
        try:
            sock.connect(destination)
            if message.id == 0:
                sock.sendall(message.produce_payload())
            elif message.id == 1:
                p: Ping = message
                p.source = sock.getsockname()
                p.destination = sock.getpeername()
                sock.sendall(message.produce_payload())
            else:
                raise Exception("Attempted to send unsupported message type")
            response = message.consume_payload(read_all(sock))
            response.handle()

        except Exception as e:
            print(e)
        sock.close()


def server(address: Tuple[str, int], keep_runing: Lock, handler) -> None:
    ThreadedTCPRequestHandler.handle = handler
    server = ThreadedTCPServer(address, ThreadedTCPRequestHandler)
    with server:
        # Start a thread with the server -- that thread will then start one
        # more thread for each request
        server_thread = Thread(target=server.serve_forever)
        # Exit the server thread when the main thread terminates
        server_thread.daemon = True
        server_thread.start()
        print("Server loop running in thread:", server_thread.name)

        keep_runing.acquire()
        keep_runing.release()

        server.shutdown()
        server.socket.close()
