"""Launch the physical_orin scenario using shared configuration."""

from catbot_launcher.scenarios import build_launch


def generate_launch_description():
    return build_launch("physical_orin")
