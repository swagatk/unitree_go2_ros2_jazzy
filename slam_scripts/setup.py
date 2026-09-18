import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'slam_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khaled',
    maintainer_email='khaledgabr77@gmail.com',
    description='Helper nodes to bridge the Go2 LiDAR to 2D LaserScan and broadcast odom->base_footprint TF for slam_toolbox mapping.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cloud_to_scan = slam_scripts.cloud_to_scan:main',
            'odom_to_tf = slam_scripts.odom_to_tf:main',
        ],
    },
)
