import os
from glob import glob

from setuptools import setup


package_name = 'robot_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'scripts'),
         glob(os.path.join('scripts', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='RViz, rosbag, and debugging assets for the robot project.',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'check_pwa_gateway_stack = robot_tools.check_pwa_gateway_stack:main',
            'check_robot_namespaces = robot_tools.check_robot_namespaces:main',
            'demo_event_keyboard = robot_tools.demo_event_keyboard:main',
            'run_dispatch_scenarios = robot_tools.run_dispatch_scenarios:main',
            'rviz_goal_to_nav2_action = robot_tools.rviz_goal_to_nav2_action:main',
            'seed_real_initial_poses = robot_tools.seed_real_initial_poses:main',
            'send_g1_nav_goals = robot_tools.send_g1_nav_goals:main',
            'send_nav_goal = robot_tools.send_nav_goal:main',
        ],
    },
)
