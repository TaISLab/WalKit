"""/scan_filtered -> /scan_nav: the scan without the active area, where the user's feet are (nav_filter.yaml), for the laser odometry
and navigation."""
from walker_laser_filter.launching import chain_launch_description


def generate_launch_description():
    return chain_launch_description('nav_laser_filter', 'nav_filter.yaml', '/scan_filtered', '/scan_nav')
