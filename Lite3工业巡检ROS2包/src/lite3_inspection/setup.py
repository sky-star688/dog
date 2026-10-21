from glob import glob
from setuptools import find_packages, setup


package_name = "lite3_inspection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lite3 User",
    maintainer_email="user@example.com",
    description="Lite3 industrial inspection scaffold",
    license="MIT",
    entry_points={
        "console_scripts": [
            "cmd_vel_guard = lite3_inspection.cmd_vel_guard:main",
            "arm_serial_bridge = lite3_inspection.arm_serial_bridge:main",
            "vision_detector = lite3_inspection.vision_detector:main",
            "inspection_manager = lite3_inspection.inspection_manager:main",
        ],
    },
)
