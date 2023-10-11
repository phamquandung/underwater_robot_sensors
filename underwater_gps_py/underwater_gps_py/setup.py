from setuptools import find_packages, setup

package_name = 'underwater_gps_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pqdung',
    maintainer_email='qdpham@connect.ust.hk',
    description='A ROS 2 package for WaterLinked Underwater GPS',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'undewater_gps = underwater_gps_py.underwater_gps:main',
        ],
    },
)
