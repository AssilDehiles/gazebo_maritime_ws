from setuptools import setup, find_packages
import os
from glob import glob
package_name = "ros2_maritime"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz",   glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="assil",
    maintainer_email="assil@usv-ros2.local",
    description="Navigation autonome USV - PFE 2025/2026",
    license="MIT",
    entry_points={
        "console_scripts": [
            "imu_relay  = ros2_maritime.imu_relay:main",
            "heading    = ros2_maritime.heading:main",
        ],
    },
)
