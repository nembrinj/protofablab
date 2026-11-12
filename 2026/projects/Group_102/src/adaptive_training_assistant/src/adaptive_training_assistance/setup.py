import os
from glob import glob

from setuptools import find_packages, setup

package_name = "adaptive-trainning-assistance"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="andriana",
    maintainer_email="andrianasma@gmail.com",
    description="Adaptive Training Assistant",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "person_detection = adaptive_training_assistance.person_detection_node:main",
            "positioning_controller = adaptive_training_assistance.positioning_controller_node:main",
            "servo_driver = adaptive_training_assistance.servo_driver_node:main",
            "robot_search_controller = adaptive_training_assistance.robot_search_controller:main",
            "exercise_counter = adaptive_training_assistance.exercise_counter_node:main",
            "training_logic = adaptive_training_assistance.training_logic_node:main",
            "training_dashboard = adaptive_training_assistance.training_dashboard_node:main",
        ],
    },
)
