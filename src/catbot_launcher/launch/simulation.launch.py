"""Compatibility alias for simulation on Orin with policy on Pi."""

from catbot_launcher.scenarios import build_launch


def generate_launch_description():
    return build_launch("simulation_pi")
