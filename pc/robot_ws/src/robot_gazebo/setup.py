from glob import glob
import os

from setuptools import setup


package_name = 'robot_gazebo'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.urdf.xacro'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='Gazebo hardware-replacement simulation assets for robot1.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_self_filter = robot_gazebo.scan_self_filter:main',
            'static_map_publisher = robot_gazebo.static_map_publisher:main',
            'sim_ackermann_base = robot_gazebo.sim_ackermann_base:main',
            'sim_mecanum_base = robot_gazebo.sim_mecanum_base:main',
            'tf_namespace_relay = robot_gazebo.tf_namespace_relay:main',
        ],
    },
)
