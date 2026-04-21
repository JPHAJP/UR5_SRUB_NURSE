import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.robot.robot_gateway import RobotGateway


def test_build_joint_path_program_blends_intermediate_points():
    program = RobotGateway._build_joint_path_program_deg(
        path_deg=[
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ],
        speed=1.0,
        acceleration=1.0,
        blend_radius=0.02,
        program_name="test_path",
    )

    assert "def test_path():" in program
    assert program.count("movej([") == 3
    assert program.count("r = 0.02)") == 2
    assert program.count("r = 0.0)") == 1
    assert "0.17453" in program
    assert "0.34907" in program


def test_greet_sends_single_blended_program_and_returns_home():
    commands = []

    gateway = RobotGateway.__new__(RobotGateway)
    gateway.config = SimpleNamespace(
        greet_sequence_deg=[
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ],
        home_joints_deg=[20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        greet_speed_rad_s=1.0,
        greet_acceleration_rad_s2=1.0,
        greet_blend_radius_m=0.02,
    )
    gateway.send_urscript = lambda command: (commands.append(command) or True, "ok")

    ok, _ = RobotGateway.greet(gateway)

    assert ok is True
    assert len(commands) == 1
    assert commands[0].count("movej([") == 3
    assert "greet_sequence()" in commands[0]
    assert commands[0].count("r = 0.02)") == 2
    assert commands[0].count("r = 0.0)") == 1
