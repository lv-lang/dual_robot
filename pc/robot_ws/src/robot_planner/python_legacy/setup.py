import os
from glob import glob

from setuptools import setup


package_name = 'robot_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=['global_planner', 'local_planner'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='Robot1 A* global planner and mecanum DWA local planner.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'astar_global_planner = global_planner.astar_global_planner:main',
            'dwa_lite_planner = local_planner.dwa_lite_node:main',
        ],
    },
)
