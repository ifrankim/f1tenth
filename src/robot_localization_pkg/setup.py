from setuptools import setup
import os
from glob import glob

package_name = "robot_localization_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        # ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rilbasestation02",
    maintainer_email="fkgaieski@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            'simulated_imu = robot_localization_pkg.simulated_imu:main',
            'noisy_odom = robot_localization_pkg.noisy_odom:main',
            'tf_transform = robot_localization_pkg.tf_transform:main',
        ],
    },
)