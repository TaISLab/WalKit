"""/scan_filtered -> /scan_feet: only the active area, where the user's feet are (feet_filter.yaml), for the step detectors."""
from walker_laser_filter.launching import chain_launch_description


def generate_launch_description():
    return chain_launch_description('feet_laser_filter', 'feet_filter.yaml', '/scan_filtered', '/scan_feet')
