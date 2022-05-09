from setuptools import setup
import os
from glob import glob

package_name = 'walker_centroid_support'

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
    description='Calculates centroid support according to J.Ballesteros method in IROS 2018',
    license='Creative Commons',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'centroid_support = walker_centroid_support.centroid_support:main','plot_centroid = walker_centroid_support.plot_centroid:main',
        ],
    },
)
