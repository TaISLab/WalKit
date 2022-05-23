from setuptools import setup

package_name = 'walker_py_cluster'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manuel Fernandez-Carmona',
    maintainer_email='manolofc@gmail.com',
    description='Do crappy stuff',
    license='Creative Commons',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cluster_plot = walker_py_cluster.cluster_plot:main',
        ],
    },
)
