import os
from glob import glob

from setuptools import setup


package_name = 'robot_nav'


def _recursive_data_files(source_dir):
    data_files = []
    for root, _, files in os.walk(source_dir):
        if not files:
            continue
        data_files.append((
            os.path.join('share', package_name, root),
            [os.path.join(root, filename) for filename in files],
        ))
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'params'),
         glob(os.path.join('params', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'),
         glob(os.path.join('maps', '*.*'))),
        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*.rviz'))),
    ] + _recursive_data_files('rviz_rendering_overlay'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='Robot1 Nav2 bringup, DWB tuning parameters, maps, and RViz config.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)
