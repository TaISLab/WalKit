from setuptools import setup
import os
from glob import glob

package_name = 'walker_loads'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manuel Fernandez-Carmona',
    maintainer_email='manolofc@gmail.com',
    description='Plot walker handle forces in rviz',
    license='Creative Commons',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'partial_loads = walker_loads.partial_loads:main','plot_loads = walker_loads.plot_loads:main',
        ],
    },
)
