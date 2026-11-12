from glob import glob

from setuptools import find_packages, setup

package_name = 'lidar_person_tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leonid Pomortsev',
    maintainer_email='pomortu3@gmail.com',
    description='Finds a walking person in the LiDAR scans of a parked robot',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'tracker_node = lidar_person_tracker.tracker_node:main',
        ],
    },
)
