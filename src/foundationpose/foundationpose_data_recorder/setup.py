from glob import glob
from setuptools import find_packages, setup

package_name = "foundationpose_data_recorder"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="spaaaaace",
    maintainer_email="maintainer@example.com",
    description="Record synchronized ROS 2 RGB-D data in FoundationPose demo format.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "foundationpose_data_recorder = "
            "foundationpose_data_recorder.recorder_node:main",
        ],
    },
)
