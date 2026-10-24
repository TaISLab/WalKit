import os
from glob import glob

from setuptools import setup

package_name = 'walker_rviz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manuel Fernandez-Carmona',
    maintainer_email='manolofc@gmail.com',
    description='What the walker measures, drawn in rviz: handle loads, feet, support centroid, stability, gait; and the rviz views',
    license='Creative Commons',
    entry_points={
        'console_scripts': [
            'walker_markers = walker_rviz.node:main',
        ],
    },
)
