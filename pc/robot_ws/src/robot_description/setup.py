from setuptools import setup
import os
from glob import glob
package_name = 'robot_description'


def recursive_data_files(source_dir):
    data_files = []
    for root, _, files in os.walk(source_dir):
        selected = [os.path.join(root, name) for name in files]
        if selected:
            data_files.append((os.path.join('share', package_name, root), selected))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share',package_name,'urdf'),glob(os.path.join('urdf','*.*'))),
        (os.path.join('share',package_name,'rviz'),glob(os.path.join('rviz','*.rviz*'))),
    ] + recursive_data_files('meshes'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx-ros2',
    maintainer_email='nx-ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
