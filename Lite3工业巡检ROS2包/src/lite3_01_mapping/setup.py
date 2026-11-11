from glob import glob
from setuptools import setup

package_name = "lite3_01_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dog 1 Team",
    maintainer_email="user@example.com",
    description="LiDAR SLAM + depth-camera patrol dog.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "patrol_manager = lite3_01_mapping.patrol_manager:main",
            "mapping_manager = lite3_01_mapping.mapping_manager:main",
            "map_bridge = lite3_01_mapping.map_bridge:main",
            "dog1_depth_projector = lite3_01_mapping.depth_projector:main",
            "dog1_vision_detector = lite3_01_mapping.vision_detector:main",
        ],
    },
)
