from setuptools import find_namespace_packages, setup
import os
from glob import glob

package_name = "schunk_gripper_dummy"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_namespace_packages(exclude=["tests"]),
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("lib", package_name), ["start_dummy"]),
        (
            os.path.join("share", package_name, "config/grippers"),
            glob(package_name + "/config/grippers/*.json"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(package_name + "/config/*.json"),
        ),
    ],
    install_requires=[
        "setuptools",
        "fastapi",
        "uvicorn",
        "requests",
        "python-multipart",
    ],
    zip_safe=True,
    author="Stefan Scherzinger, Harry Arnst",
    author_email="stefan.scherzinger@de.schunk.com, harry.arnst@de.schunk.com",
    maintainer="Jonas Beil",
    maintainer_email="jonas.beil@de.schunk.com",
    description="A minimalist dummy for simulating the gripper's communication",
    license="GPL-3.0-or-later",
    entry_points={
        "console_scripts": [
            "start_dummy = schunk_gripper_dummy.main:main",
        ],
    },
    scripts=["start_dummy"],
)
