import ssl_vision_wrapper_pb2

from .net import open_multicast_socket
from .world import WorldModel, update_from_detection_frame, update_from_geometry_data


def parse_wrapper_packet(raw: bytes) -> ssl_vision_wrapper_pb2.SSL_WrapperPacket:
    packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket()
    packet.ParseFromString(raw)
    return packet


def apply_wrapper_packet(world: WorldModel, packet: ssl_vision_wrapper_pb2.SSL_WrapperPacket) -> None:
    if packet.HasField("detection"):
        update_from_detection_frame(world, packet.detection)
    if packet.HasField("geometry"):
        update_from_geometry_data(world, packet.geometry)


class VisionReceiver:
    def __init__(self, world: WorldModel, group: str = "224.5.23.2", port: int = 10020):
        self.world = world
        self.group = group
        self.port = port

    def handle_packet(self, raw: bytes) -> None:
        apply_wrapper_packet(self.world, parse_wrapper_packet(raw))

    def run_forever(self) -> None:
        sock = open_multicast_socket(self.group, self.port)
        while True:
            raw, _ = sock.recvfrom(65536)
            self.handle_packet(raw)
