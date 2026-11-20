from glob import glob

from setuptools import find_packages, setup

package_name = 'session_reporter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leonid Pomortsev',
    maintainer_email='pomortu3@gmail.com',
    description='Sends the session results as JSON over MQTT to the dashboard',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'reporter_node = session_reporter.reporter_node:main',
        ],
    },
)
