import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'crazyflie'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('test_runs/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('test_runs/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhishek',
    maintainer_email='abhishekenrhu39@gmail.com',
    description='ROS 2 package for Crazyflie integration',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'multiranger_mapper = test_runs.multiranger_mapper:main',
            'exploration_radio_bridge = test_runs.radio_bridge:main',
            'state_estimate_bridge = test_runs.radio_bridge:main',
            'range_monitor = test_runs.range_monitor:main',
            'replay_bag = test_runs.replay_bag:main',
            'map_localizer = test_runs.map_localizer:main',
            'publish_saved_map = test_runs.publish_saved_map:main',
        ],
    },
)
