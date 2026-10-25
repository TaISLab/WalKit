"""/scan -> /scan_filtered: the scan without the walker's own structure (self_filter.yaml, made by find_self_collision_boxes.py)."""
from walker_laser_filter.launching import chain_launch_description


def generate_launch_description():
    return chain_launch_description('self_laser_filter', 'self_filter.yaml', '/scan', '/scan_filtered')
