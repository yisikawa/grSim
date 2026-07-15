import math
import socket
from dataclasses import dataclass

import grSim_Packet_pb2


@dataclass
class RobotCommand:
    robot_id: int
    vel_x: float  # desired velocity, field (global) frame, m/s
    vel_y: float  # desired velocity, field (global) frame, m/s
    vel_angular: float  # rad/s
    robot_orientation: float  # current measured robot orientation, radians
    kick_speed: float = 0.0  # m/s, straight kick
    chip_speed: float = 0.0  # m/s, chip kick
    dribble: bool = False


def global_to_local(vel_x: float, vel_y: float, orientation: float) -> tuple:
    """Rotate a field-frame velocity into the robot's local frame (tangent=
    forward, normal=left), matching Robot::setSpeed in src/robot.cpp."""
    tangent = vel_x * math.cos(orientation) + vel_y * math.sin(orientation)
    normal = -vel_x * math.sin(orientation) + vel_y * math.cos(orientation)
    return tangent, normal


def build_packet(is_team_yellow: bool, commands: list) -> bytes:
    packet = grSim_Packet_pb2.grSim_Packet()
    packet.commands.timestamp = 0.0
    packet.commands.isteamyellow = is_team_yellow
    for cmd in commands:
        rc = packet.commands.robot_commands.add()
        rc.id = cmd.robot_id
        tangent, normal = global_to_local(cmd.vel_x, cmd.vel_y, cmd.robot_orientation)
        rc.veltangent = tangent
        rc.velnormal = normal
        rc.velangular = cmd.vel_angular
        rc.kickspeedx = cmd.kick_speed
        rc.kickspeedz = cmd.chip_speed
        rc.spinner = cmd.dribble
        rc.wheelsspeed = False
    return packet.SerializeToString()


class CommandSender:
    def __init__(self, host: str = "127.0.0.1", port: int = 20011):
        self.host = host
        self.port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    def send(self, is_team_yellow: bool, commands: list) -> None:
        data = build_packet(is_team_yellow, commands)
        self._sock.sendto(data, (self.host, self.port))
