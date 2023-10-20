import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ping360_sonar_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pqdung',
    maintainer_email='qdpham@connect.ust.hk',
    description='A ROS 2 package for Blue Robotics Ping360 Sonar',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ping360 = ping360_sonar_py.ping360:main',
        ],
    },
)
