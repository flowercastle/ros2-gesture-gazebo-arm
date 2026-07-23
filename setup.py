from setuptools import find_packages, setup
import os
from glob import glob

package_name = "gesture_gazebo_arm"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name]
        ),
        (
            "share/" + package_name,
            ["package.xml"]
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob("urdf/*.xacro")
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml")
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="王云峰",
    maintainer_email="1922639143@qq.com",
    description=(
    "A ROS 2 and Gazebo Sim robotic arm controlled "
    "through MediaPipe-based hand gesture recognition."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "test_arm_controller = gesture_gazebo_arm.test_arm_controller:main",
            "gesture_arm_controller = gesture_gazebo_arm.gesture_arm_controller:main",
        ],
    },
)