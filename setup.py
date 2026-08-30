from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ida_data_logger'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IEEE Ege Mavi İnci Yazılım Ekibi',
    maintainer_email='ieeegesb@gmail.com',
    description='Video, telemetri CSV ve costmap bag kaydedici.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'video_logger       = ida_data_logger.video_logger:main',
            'lidar_video_logger = ida_data_logger.lidar_video_logger:main',
            'telemetry_logger   = ida_data_logger.telemetry_logger:main',
            'costmap_logger     = ida_data_logger.costmap_logger:main',
        ],
    },
)
