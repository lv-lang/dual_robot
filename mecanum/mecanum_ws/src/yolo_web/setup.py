from setuptools import setup

package_name = 'yolo_web'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/yolo_web.launch.py',
            'launch/yolo_system.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/box_order.yaml']),
        ('share/' + package_name + '/models', [
            'models/box_camera_best_bayese_640x640_nv12.bin',
            'models/fire_smoke_best_bayese_640x640_nv12.bin',
            'models/classes.txt',
            'models/fire_smoke.list',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@example.com',
    description='USB camera web YOLO detection for RDK X5.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'usb_camera_web_server = yolo_web.usb_camera_web_server:main',
        ],
    },
)
