from setuptools import find_namespace_packages, setup
import os
from glob import glob

package_name = "schunk_gripper_library"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_namespace_packages(exclude=["tests"]),
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob(package_name + "/config/*.json"),
        ),
    ],
    install_requires=[
        "pymodbus==3.8.6",
        "pyserial>=3.5",
        "httpx>=0.28.1",
        "netifaces2>=0.0.22",
        "empy>=3.3.4",
        "catkin_pkg>=1.1.0",
        "lark>=1.1.1"
    ],
    zip_safe=True,
    author="Stefan Scherzinger, Harry Arnst",
    author_email="stefan.scherzinger@de.schunk.com, harry.arnst@de.schunk.com",
    maintainer="Jonas Beil",
    maintainer_email="jonas.beil@de.schunk.com",
    description="Low-level driver library for Modbus RTU and Ethernet SCHUNK grippers",
    license="GPL-3.0-or-later",
    entry_points={
        "console_scripts": [],
    },
)
