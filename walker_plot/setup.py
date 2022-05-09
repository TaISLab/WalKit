from setuptools import setup

package_name = 'walker_plot'

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
    description='Plot walker handle forces in rviz',
    license='Creative Commons',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plot_forces = walker_plot.plot_forces:main',
            'plot_active_area = walker_plot.plot_active_area:main',
        ],
    },
)
