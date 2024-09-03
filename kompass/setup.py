import os
from glob import glob

from setuptools import find_packages, setup

package_name = "kompass"

console_scripts = [
    "executable = kompass.executable:main",
    "turtlebot3_test = recipes.turtlebot3:kompass_bringup",
]

setup(
    name=package_name,
    version="0.1.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "params"),
            glob(os.path.join("params", "*.yaml*")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("simulation", "launch", "*.py*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Automatika Robotics",
    maintainer_email="contact@automatikarobotics.com",
    description="Kompass: Event-driven Navigation System",
    license="MIT License Copyright (c) 2024 Automatika Robotics",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": console_scripts,
    },
)
