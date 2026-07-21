from typing import Optional

from state import ssl_gc_referee_message_pb2 as referee_pb2

from .net import open_multicast_socket


def parse_referee_packet(raw: bytes) -> referee_pb2.Referee:
    msg = referee_pb2.Referee()
    msg.ParseFromString(raw)
    return msg


class RefereeReceiver:
    def __init__(self, group: str = "224.5.23.1", port: int = 10003):
        self.group = group
        self.port = port
        self.latest: Optional[referee_pb2.Referee] = None

    def handle_packet(self, raw: bytes) -> None:
        self.latest = parse_referee_packet(raw)

    def run_forever(self) -> None:
        sock = open_multicast_socket(self.group, self.port)
        while True:
            raw, _ = sock.recvfrom(65536)
            self.handle_packet(raw)
