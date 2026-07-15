import math

import grSim_Packet_pb2

from ssl_client.commands import RobotCommand, build_packet, global_to_local


def test_global_to_local_no_rotation_is_identity():
    tangent, normal = global_to_local(1.0, 2.0, 0.0)
    assert math.isclose(tangent, 1.0)
    assert math.isclose(normal, 2.0)


def test_global_to_local_facing_positive_y_moving_global_x_goes_right():
    # Facing +Y (orientation=pi/2) and moving in global +X is "to the robot's
    # right": zero forward (tangent) motion, negative normal (normal is
    # positive to the left). Matches the rotation used in src/robot.cpp:482-483.
    tangent, normal = global_to_local(1.0, 0.0, math.pi / 2)
    assert math.isclose(tangent, 0.0, abs_tol=1e-9)
    assert math.isclose(normal, -1.0, abs_tol=1e-9)


def test_build_packet_sets_team_color_and_round_trips_all_fields():
    commands = [
        RobotCommand(
            robot_id=2,
            vel_x=1.0,
            vel_y=0.0,
            vel_angular=0.5,
            robot_orientation=0.0,
            kick_speed=3.0,
            chip_speed=0.0,
            dribble=True,
        ),
    ]

    data = build_packet(is_team_yellow=True, commands=commands)

    parsed = grSim_Packet_pb2.grSim_Packet()
    parsed.ParseFromString(data)
    assert parsed.commands.isteamyellow is True
    rc = parsed.commands.robot_commands[0]
    assert rc.id == 2
    assert math.isclose(rc.veltangent, 1.0)
    assert math.isclose(rc.velnormal, 0.0, abs_tol=1e-9)
    assert math.isclose(rc.velangular, 0.5)
    assert math.isclose(rc.kickspeedx, 3.0)
    assert math.isclose(rc.kickspeedz, 0.0)
    assert rc.spinner is True
    assert rc.wheelsspeed is False


def test_build_packet_with_no_commands_is_still_valid():
    data = build_packet(is_team_yellow=False, commands=[])

    parsed = grSim_Packet_pb2.grSim_Packet()
    parsed.ParseFromString(data)
    assert parsed.commands.isteamyellow is False
    assert len(parsed.commands.robot_commands) == 0
