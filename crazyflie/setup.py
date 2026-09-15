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
            glob('launch/*.launch.py') + glob('test_runs/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz') + glob('test_runs/*.rviz')),
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
            'reactive_explorer = test_runs.reactive_explorer:main',
            'mapper = crazyflie.mapping:main',
            'pose_estimator = crazyflie.pose_estimator:main',
            'esp_pose = crazyflie.pose:main',
            'esp_mapper = crazyflie.mapper:main',
            'udp_bridge = crazyflie.udp_bridge:main',
            'simple_mapper = chectest.simple_mapper:main',
            'udp_bridge_test = chectest.udp:main',
            'plot_deviation = chectest.plot_deviation:main',
            'crazyflie_connect = crazyflie.crazyflie_start:main',
            'event = chectest.event_recorder:main',
            'of_odometry = chectest.of_odometry:main',
            'drift_analyze = chectest.drift_analyze:main',
            'constraint_replay = chectest.constraint_replay:main',
            'batch_stats = chectest.batch_stats:main',
            'deviation = test_runs.deviation:main',
        ],

    },
)
