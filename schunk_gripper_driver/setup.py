from setuptools import find_packages, setup
from glob import glob
import os

package_name = "schunk_gripper_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("lib", package_name), [package_name + "/driver.py"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pymodbus"],
    zip_safe=True,
    author="Stefan Scherzinger, Harry Arnst",
    author_email="stefan.scherzinger@de.schunk.com, harry.arnst@de.schunk.com",
    maintainer="Jonas Beil",
    maintainer_email="jonas.beil@de.schunk.com",
    description="ROS2 driver for SCHUNK`s EGU, EGK, and EZU mechatronic grippers",
    license="GPL-3.0-or-later",
    entry_points={
        "console_scripts": [],
    },
)
